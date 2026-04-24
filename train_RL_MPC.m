function train_RL_MPC()
    rootDir = fileparts(mfilename('fullpath'));
    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');

    S = load(cfgPath, 'env', 'agent', 'cfg');
    env = S.env;
    agent = S.agent;
    cfg = S.cfg;

    logsRoot = fullfile(rootDir, 'RL Midtraining Logs');
    if ~exist(logsRoot, 'dir')
        mkdir(logsRoot);
    end

    runStamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    runDir = fullfile(logsRoot, ['run_' runStamp]);
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

    save(cfgPath, 'cfg', '-append');

    clear rlResetFunction rlStepFunction

    diary off
    diary(logFile)
    diary on

    cleanupObj = onCleanup(@() localCleanupTrain(cfgPath));

    fprintf('[TRAIN START] episodes=%d, decisions/episode=%d, chunks/decision=%d, chunks/episode=%d\n', ...
        600, cfg.EP_STEPS, cfg.APPLY_EVERY, cfg.EP_STEPS * cfg.APPLY_EVERY);
    fprintf('[RUN DIR] %s\n', runDir);
    fprintf('[LOG FILE] %s\n', logFile);
    fprintf('[SAVED AGENTS DIR] %s\n', savedAgentsDir);

    trainOpts = rlTrainingOptions( ...
        'MaxEpisodes', 10, ...
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

    save(fullfile(rootDir, 'final_RL_agent.mat'), 'agent', 'stats');
    save(finalAgentFile, 'agent', 'stats');

    fprintf('[TRAIN END] final agent saved to %s\n', finalAgentFile);
    disp('Training complete.')
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