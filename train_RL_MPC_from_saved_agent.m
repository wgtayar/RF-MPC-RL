function train_RL_MPC_from_saved_agent(savedAgentPath, maxEpisodes, noiseVariance, runLabel)
    rootDir = bootstrap_RF_MPC_RL();

    rehash

    if exist('get_params', 'file') ~= 2
        error(['get_params.m was not found on the MATLAB path. ', ...
               'Check that the fcns or fcns_MPC folder exists under: %s'], rootDir);
    end

    if nargin < 1 || isempty(savedAgentPath)
        savedAgentPath = fullfile(rootDir, 'RL Midtraining Logs', ...
            'Successful Agents', 'run_2026-07-03_05-38-23', 'Agent74.mat');
    end

    if nargin < 2 || isempty(maxEpisodes)
        maxEpisodes = 30;
    end

    if nargin < 3 || isempty(noiseVariance)
        noiseVariance = 0;
    end

    if nargin < 4 || isempty(runLabel)
        if noiseVariance == 0
            runLabel = 'zero_noise_diagnostic';
        else
            runLabel = sprintf('noiseVar_%0.3g', noiseVariance);
        end
    end

    if ~exist(savedAgentPath, 'file')
        error('Saved agent file does not exist: %s', savedAgentPath);
    end

    % Rebuild environment and cfg from the current codebase.
    % This is intentional: it guarantees action bounds, reward coefficients,
    % and logging fields match the current setup_RL_MPC.m.
    fprintf('[SETUP] Rebuilding rlEnv_MPC_R.mat using current setup_RL_MPC.m ...\n');
    setup_RL_MPC

    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');
    S = load(cfgPath, 'env', 'cfg', 'lower_abs', 'upper_abs', 'initial_R');
    env = S.env;
    cfg = S.cfg;
    rng(cfg.RNG_SEED, cfg.RNG_ALGORITHM);

    sourceExperimentId = '';
    if isfield(cfg, 'EXPERIMENT') && isfield(cfg.EXPERIMENT, 'id')
        sourceExperimentId = cfg.EXPERIMENT.id;
    end

    sourceRewardVersion = '';
    if isfield(cfg, 'REWARD') && isfield(cfg.REWARD, 'version')
        sourceRewardVersion = cfg.REWARD.version;
    end

    agent = localLoadSavedAgent(savedAgentPath);
    agent = localSetAgentNoise(agent, noiseVariance);

    [~, sourceAgentBase, ~] = fileparts(savedAgentPath);
    cleanRunLabel = localCleanLabel(runLabel);
    cleanAgentBase = localCleanLabel(sourceAgentBase);

    logsRoot = fullfile(rootDir, 'RL Midtraining Logs');
    if ~exist(logsRoot, 'dir')
        mkdir(logsRoot);
    end

    runStamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    runDirName = sprintf('warmstart_%s_%s_%s', runStamp, cleanAgentBase, cleanRunLabel);
    runDir = fullfile(logsRoot, runDirName);
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
    cfg.RUN.run_id = runDirName;
    cfg.RUN.log_file = logFile;
    cfg.RUN.checkpoint_file = fullfile(runDir, ['rl_checkpoints_' runStamp '.mat']);
    cfg.RUN.chunk_csv = fullfile(runDir, ['rl_chunks_' runStamp '.csv']);
    cfg.RUN.decision_csv = fullfile(runDir, ['rl_decisions_' runStamp '.csv']);
    cfg.RUN.failure_csv = fullfile(runDir, ['rl_failures_' runStamp '.csv']);
    cfg.RUN.saved_agents_dir = savedAgentsDir;
    cfg.RUN.qp_failure_dir = fullfile(runDir, 'qp_failures');
    if ~exist(cfg.RUN.qp_failure_dir, 'dir')
        mkdir(cfg.RUN.qp_failure_dir);
    end
    cfg.RUN.experiment_id = '';
    cfg.RUN.reward_version = '';

    cfg.WARMSTART.enabled = true;
    cfg.WARMSTART.source_agent_path = savedAgentPath;
    cfg.WARMSTART.source_agent_name = sourceAgentBase;
    cfg.WARMSTART.max_episodes = maxEpisodes;
    cfg.WARMSTART.noise_variance = noiseVariance;
    cfg.WARMSTART.noise_std = sqrt(max(noiseVariance, 0));
    cfg.WARMSTART.run_label = runLabel;
    cfg.WARMSTART.mode = 'warmstart_training';

    % Keep the reward version unchanged, but make the experiment id run-specific.
    if isfield(cfg, 'EXPERIMENT')
        cfg.EXPERIMENT.parent_id = sourceExperimentId;
        cfg.EXPERIMENT.id = sprintf('%s_warmstart_%s_%s', ...
            sourceExperimentId, cleanAgentBase, cleanRunLabel);
        cfg.EXPERIMENT.description = sprintf([ ...
            'Warm-start from %s using current stability-recovery reward. ', ...
            'maxEpisodes=%d, noiseVariance=%.12g, noiseStd=%.12g.'], ...
            sourceAgentBase, maxEpisodes, noiseVariance, sqrt(max(noiseVariance, 0)));
    end

    if isfield(cfg, 'EXPERIMENT') && isfield(cfg.EXPERIMENT, 'id')
        cfg.RUN.experiment_id = cfg.EXPERIMENT.id;
    end

    if isfield(cfg, 'REWARD') && isfield(cfg.REWARD, 'version')
        cfg.RUN.reward_version = cfg.REWARD.version;
    end

    sourceAgentFile = savedAgentPath;
    save(fullfile(runDir, 'warmstart_source_agent.mat'), 'sourceAgentFile');
    cfgSnapshotFile = fullfile(runDir, ['cfg_snapshot_' runStamp '.mat']);
    manifestFile = fullfile(runDir, ['run_manifest_' runStamp '.txt']);
    runInfo = struct();
    runInfo.run_id = cfg.RUN.run_id;
    runInfo.training_mode = 'warmstart';
    runInfo.max_episodes = maxEpisodes;
    runInfo.random_seed = cfg.RNG_SEED;
    runInfo.noise_variance = noiseVariance;
    runInfo.noise_decay = 0;
    runInfo.source_agent = savedAgentPath;
    runInfo.run_label = runLabel;
    runInfo.start_time = char(datetime('now', 'TimeZone', 'local', ...
        'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX'));
    save(cfgSnapshotFile, 'cfg');
    write_run_manifest(manifestFile, cfg, runInfo);

    save(cfgPath, 'cfg', 'agent', '-append');

    clear rlResetFunction rlStepFunction

    diary off
    diary(logFile)
    diary on

    cleanupObj = onCleanup(@() localCleanupTrain(cfgPath)); %#ok<NASGU>

    fprintf('[WARMSTART TRAIN START]\n');
    fprintf('[SOURCE AGENT] %s\n', savedAgentPath);
    fprintf('[SOURCE REWARD VERSION] %s\n', sourceRewardVersion);
    fprintf('[RUN LABEL] %s\n', runLabel);
    fprintf('[RUN DIR] %s\n', runDir);
    fprintf('[LOG FILE] %s\n', logFile);
    fprintf('[SAVED AGENTS DIR] %s\n', savedAgentsDir);
    fprintf('[NOISE] variance=%.12g, std=%.12g\n', noiseVariance, sqrt(max(noiseVariance, 0)));
    fprintf('[MISSION] D=%.2f m, T=%.2f s, decisions=%d, chunks/decision=%d\n', ...
        cfg.MISSION.D_TARGET_M, cfg.MISSION_DURATION, cfg.EP_STEPS, cfg.APPLY_EVERY);
    fprintf('[ACTION LIMITS] DR_MAX=%.6g, gamma_v=[%.3f %.3f], gamma_a=[%.3f %.3f], v_cap=%.3f m/s\n', ...
        cfg.DR_MAX, cfg.GAMMA_V_MIN, cfg.GAMMA_V_MAX, cfg.GAMMA_A_MIN, cfg.GAMMA_A_MAX, ...
        cfg.V_MIN + cfg.GAMMA_V_MAX * (cfg.V_MAX - cfg.V_MIN));

    trainOpts = rlTrainingOptions( ...
        'MaxEpisodes', maxEpisodes, ...
        'MaxStepsPerEpisode', cfg.EP_STEPS, ...
        'ScoreAveragingWindowLength', min(20, maxEpisodes), ...
        'Verbose', true, ...
        'Plots', 'training-progress', ...
        'SaveAgentCriteria', 'EpisodeFrequency', ...
        'SaveAgentValue', 1, ...
        'SaveAgentDirectory', savedAgentsDir);

    wallClockStart = tic;
    stats = train(agent, env, trainOpts);

    runInfo.end_time = char(datetime('now', 'TimeZone', 'local', ...
        'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX'));
    runInfo.wall_clock_seconds = toc(wallClockStart);
    write_run_manifest(manifestFile, cfg, runInfo);
    save(cfgSnapshotFile, 'cfg', 'runInfo');

    cfg.LOG.enable = false;
    cfg.RUN.enabled = false;
    save(cfgPath, 'cfg', '-append');

    save(fullfile(rootDir, 'final_RL_agent_warmstart.mat'), ...
        'agent', 'stats', 'sourceAgentFile', 'maxEpisodes', 'noiseVariance', 'runLabel');
    save(finalAgentFile, ...
        'agent', 'stats', 'sourceAgentFile', 'maxEpisodes', 'noiseVariance', 'runLabel');

    fprintf('[WARMSTART TRAIN END] final agent saved to %s\n', finalAgentFile);
    disp('Warm-start training complete.')
end

function agent = localSetAgentNoise(agent, noiseVariance)
    try
        if isprop(agent, 'AgentOptions') && isprop(agent.AgentOptions, 'NoiseOptions')
            agent.AgentOptions.NoiseOptions.Variance = noiseVariance;
            agent.AgentOptions.NoiseOptions.VarianceDecayRate = 0;
            fprintf('[NOISE OVERRIDE] Set agent.AgentOptions.NoiseOptions.Variance = %.12g\n', noiseVariance);
        else
            warning('Agent does not expose AgentOptions.NoiseOptions. Noise was not overridden.');
        end
    catch ME
        warning('train_RL_MPC_from_saved_agent:NoiseOverrideFailed', ...
            'Could not override agent noise options: %s', ME.message);
    end
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

function label = localCleanLabel(label)
    label = char(label);
    label = regexprep(label, '[^a-zA-Z0-9_\-]+', '_');
    label = regexprep(label, '_+', '_');
    label = regexprep(label, '^_+|_+$', '');
    if isempty(label)
        label = 'run';
    end
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
