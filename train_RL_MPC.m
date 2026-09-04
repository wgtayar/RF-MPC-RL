function train_RL_MPC(maxEpisodes)
%train_RL_MPC Train a scratch agent using the active generated configuration.

    rootDir = bootstrap_RF_MPC_RL();
    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');

    S = load(cfgPath, 'env', 'agent', 'cfg');
    env = S.env;
    agent = S.agent;
    cfg = S.cfg;

    if nargin < 1 || isempty(maxEpisodes)
        maxEpisodes = cfg.TRAIN.default_max_episodes;
    end
    validateattributes(maxEpisodes, {'numeric'}, ...
        {'scalar', 'integer', 'positive', 'finite'}, mfilename, 'maxEpisodes');
    rng(cfg.RNG_SEED, cfg.RNG_ALGORITHM);

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
    cfg.RUN.run_id = ['run_' runStamp];
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
    
    cfg.RUN.experiment_id = "";
    cfg.RUN.reward_version = "";
    
    if isfield(cfg, 'EXPERIMENT') && isfield(cfg.EXPERIMENT, 'id')
        cfg.RUN.experiment_id = cfg.EXPERIMENT.id;
    end
    
    if isfield(cfg, 'REWARD') && isfield(cfg.REWARD, 'version')
        cfg.RUN.reward_version = cfg.REWARD.version;
    end
    
    cfgSnapshotFile = fullfile(runDir, ['cfg_snapshot_' runStamp '.mat']);
    manifestFile = fullfile(runDir, ['run_manifest_' runStamp '.txt']);
    runInfo = struct();
    runInfo.run_id = cfg.RUN.run_id;
    runInfo.training_mode = 'scratch';
    runInfo.max_episodes = maxEpisodes;
    runInfo.random_seed = cfg.RNG_SEED;
    runInfo.noise_variance = cfg.TRAIN.noise_variance;
    runInfo.noise_decay = cfg.TRAIN.noise_decay;
    runInfo.start_time = char(datetime('now', 'TimeZone', 'local', ...
        'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX'));
    save(cfgSnapshotFile, 'cfg');
    write_run_manifest(manifestFile, cfg, runInfo);
    
    save(cfgPath, 'cfg', '-append');

    clear rlResetFunction rlStepFunction

    diary off
    diary(logFile)
    diary on

    cleanupObj = onCleanup(@() localCleanupTrain(cfgPath)); %#ok<NASGU>
    wallClockStart = tic;

    fprintf('[TRAIN START] episodes=%d, decisions/episode=%d, chunks/decision=%d, chunks/episode=%d\n', ...
        maxEpisodes, cfg.EP_STEPS, cfg.APPLY_EVERY, cfg.EP_STEPS * cfg.APPLY_EVERY);
    fprintf('[RUN DIR] %s\n', runDir);
    fprintf('[LOG FILE] %s\n', logFile);
    fprintf('[SAVED AGENTS DIR] %s\n', savedAgentsDir);
    localAppendLifecycleLog(logFile, sprintf( ...
        '[TRAIN START] time=%s episodes=%d seed=%d run_dir=%s', ...
        runInfo.start_time, maxEpisodes, cfg.RNG_SEED, runDir));

    trainOpts = rlTrainingOptions( ...
        'MaxEpisodes', maxEpisodes, ...
        'MaxStepsPerEpisode', cfg.EP_STEPS, ...
        'ScoreAveragingWindowLength', 20, ...
        'Verbose', true, ...
        'Plots', 'training-progress', ...
        'SaveAgentCriteria', 'EpisodeFrequency', ...
        'SaveAgentValue', 1, ...
        'SaveAgentDirectory', savedAgentsDir);

    stats = train(agent, env, trainOpts);

    runInfo.end_time = char(datetime('now', 'TimeZone', 'local', ...
        'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX'));
    runInfo.wall_clock_seconds = toc(wallClockStart);
    write_run_manifest(manifestFile, cfg, runInfo);
    save(cfgSnapshotFile, 'cfg', 'runInfo');

    cfg.LOG.enable = false;
    cfg.RUN.enabled = false;
    save(cfgPath, 'cfg', '-append');

    save(fullfile(rootDir, 'final_RL_agent.mat'), 'agent', 'stats');
    save(finalAgentFile, 'agent', 'stats');

    fprintf('[TRAIN END] final agent saved to %s\n', finalAgentFile);
    localAppendLifecycleLog(logFile, sprintf( ...
        '[TRAIN END] time=%s wall_clock_seconds=%.6f final_agent=%s', ...
        runInfo.end_time, runInfo.wall_clock_seconds, finalAgentFile));
    disp('Training complete.')
end

function localAppendLifecycleLog(logFile, message)
    fid = fopen(logFile, 'a');
    if fid < 0
        warning('train_RL_MPC:LifecycleLogOpenFailed', ...
            'Could not append lifecycle record to %s.', logFile);
        return
    end
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '%s\n', message);
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
