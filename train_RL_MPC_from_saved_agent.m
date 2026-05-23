function train_RL_MPC_from_saved_agent(savedAgentPath)
    rootDir = fileparts(mfilename('fullpath'));

    addpath(rootDir);
    addpath(fullfile(rootDir, 'fcns'));
    addpath(fullfile(rootDir, 'fcns_MPC'));
    addpath(fullfile(rootDir, 'RL Midtraining Logs'));

    rehash

    if exist('get_params', 'file') ~= 2
        error(['get_params.m was not found on the MATLAB path. ', ...
               'Check that the fcns or fcns_MPC folder exists under: %s'], rootDir);
    end

    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');

    if nargin < 1 || isempty(savedAgentPath)
        error('Please provide the full path to the saved agent .mat file.');
    end

    if ~exist(savedAgentPath, 'file')
        error('Saved agent file does not exist: %s', savedAgentPath);
    end

    S = load(cfgPath, 'env', 'cfg', 'lower_abs', 'upper_abs', 'initial_R');
    env = S.env;
    cfg = S.cfg;

    agent = localLoadSavedAgent(savedAgentPath);

    logsRoot = fullfile(rootDir, 'RL Midtraining Logs');
    if ~exist(logsRoot, 'dir')
        mkdir(logsRoot);
    end

    runStamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    runDir = fullfile(logsRoot, ['warmstart_run_' runStamp]);
    if ~exist(runDir, 'dir')
        mkdir(runDir);
    end

    logFile = fullfile(runDir, ['training_log_' runStamp '.txt']);
    finalAgentFile = fullfile(runDir, ['final_RL_agent_' runStamp '.mat']);

    savedAgentsDir = fullfile(runDir, 'saved_agents');
    if ~exist(savedAgentsDir, 'dir')
        mkdir(savedAgentsDir);
    end

    cfg.LOG.enable = true;
    cfg.RUN.enabled = true;
    cfg.RUN.root_dir = logsRoot;
    cfg.RUN.run_dir = runDir;
    cfg.RUN.run_stamp = runStamp;
    cfg.RUN.log_file = logFile;
    cfg.RUN.checkpoint_file = fullfile(runDir, ['rl_checkpoints_' runStamp '.mat']);
    cfg.RUN.chunk_csv = fullfile(runDir, ['rl_chunks_' runStamp '.csv']);
    cfg.RUN.decision_csv = fullfile(runDir, ['rl_decisions_' runStamp '.csv']);
    cfg.RUN.failure_csv = fullfile(runDir, ['rl_failures_' runStamp '.csv']);
    cfg.RUN.saved_agents_dir = savedAgentsDir;

    sourceAgentFile = savedAgentPath;
    save(fullfile(runDir, 'warmstart_source_agent.mat'), 'sourceAgentFile');

    save(cfgPath, 'cfg', 'agent', '-append');

    clear rlResetFunction rlStepFunction

    diary off
    diary(logFile)
    diary on

    cleanupObj = onCleanup(@() localCleanupTrain(cfgPath));

    fprintf('[WARMSTART TRAIN START]\n');
    fprintf('[SOURCE AGENT] %s\n', savedAgentPath);
    fprintf('[RUN DIR] %s\n', runDir);
    fprintf('[LOG FILE] %s\n', logFile);
    fprintf('[SAVED AGENTS DIR] %s\n', savedAgentsDir);
    fprintf('[MISSION] D=%.2f m, T=%.2f s, decisions=%d, chunks/decision=%d\n', ...
        cfg.MISSION.D_TARGET_M, cfg.MISSION_DURATION, cfg.EP_STEPS, cfg.APPLY_EVERY);

    trainOpts = rlTrainingOptions( ...
        'MaxEpisodes', 150, ...
        'MaxStepsPerEpisode', cfg.EP_STEPS, ...
        'ScoreAveragingWindowLength', 20, ...
        'Verbose', true, ...
        'Plots', 'training-progress', ...
        'SaveAgentCriteria', 'EpisodeFrequency', ...
        'SaveAgentValue', 1, ...
        'SaveAgentDirectory', savedAgentsDir);

    stats = train(agent, env, trainOpts);

    cfg.LOG.enable = false;
    cfg.RUN.enabled = false;
    save(cfgPath, 'cfg', '-append');

    save(fullfile(rootDir, 'final_RL_agent_warmstart.mat'), 'agent', 'stats', 'sourceAgentFile');
    save(finalAgentFile, 'agent', 'stats', 'sourceAgentFile');

    fprintf('[WARMSTART TRAIN END] final agent saved to %s\n', finalAgentFile);
    disp('Warm-start training complete.')
end

function agent = localLoadSavedAgent(savedAgentPath)
    vars = whos('-file', savedAgentPath);
    names = {vars.name};

    preferredNames = {'agent', 'saved_agent', 'trainedAgent', 'Agent'};

    for i = 1:numel(preferredNames)
        if any(strcmp(names, preferredNames{i}))
            Sagent = load(savedAgentPath, preferredNames{i});
            agent = Sagent.(preferredNames{i});
            fprintf('[LOAD AGENT] Loaded variable "%s" from %s\n', preferredNames{i}, savedAgentPath);
            return
        end
    end

    Sagent = load(savedAgentPath);
    names = fieldnames(Sagent);

    for i = 1:numel(names)
        candidate = Sagent.(names{i});
        if isa(candidate, 'rl.agent.AbstractAgent') || contains(class(candidate), 'rl')
            agent = candidate;
            fprintf('[LOAD AGENT] Loaded variable "%s" of class "%s" from %s\n', ...
                names{i}, class(candidate), savedAgentPath);
            return
        end
    end

    error('No RL agent object found in saved file: %s', savedAgentPath);
end

function localCleanupTrain(cfgPath)
    try
        diary off
    catch
    end

    try
        if exist(cfgPath, 'file')
            S = load(cfgPath, 'cfg');
            cfg = S.cfg;

            if isfield(cfg, 'LOG')
                cfg.LOG.enable = false;
            end

            if isfield(cfg, 'RUN')
                cfg.RUN.enabled = false;
            end

            save(cfgPath, 'cfg', '-append');
        end
    catch
    end
end