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
    
    cfg.RUN.experiment_id = "";
    cfg.RUN.reward_version = "";
    
    if isfield(cfg, 'EXPERIMENT') && isfield(cfg.EXPERIMENT, 'id')
        cfg.RUN.experiment_id = cfg.EXPERIMENT.id;
    end
    
    if isfield(cfg, 'REWARD') && isfield(cfg.REWARD, 'version')
        cfg.RUN.reward_version = cfg.REWARD.version;
    end
    
    save(fullfile(runDir, ['cfg_snapshot_' runStamp '.mat']), 'cfg');
    write_run_manifest(fullfile(runDir, ['run_manifest_' runStamp '.txt']), cfg, 'scratch', '');
    
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

function write_run_manifest(manifestPath, cfg, trainMode, sourceAgentPath)
    fid = fopen(manifestPath, 'w');
    if fid < 0
        warning('Could not write run manifest: %s', manifestPath);
        return
    end

    cleanupObj = onCleanup(@() fclose(fid));

    fprintf(fid, 'train_mode: %s\n', trainMode);
    fprintf(fid, 'source_agent: %s\n', sourceAgentPath);

    if isfield(cfg, 'EXPERIMENT') && isfield(cfg.EXPERIMENT, 'id')
        fprintf(fid, 'experiment_id: %s\n', cfg.EXPERIMENT.id);
    end

    if isfield(cfg, 'EXPERIMENT') && isfield(cfg.EXPERIMENT, 'description')
        fprintf(fid, 'experiment_description: %s\n', cfg.EXPERIMENT.description);
    end

    if isfield(cfg, 'REWARD') && isfield(cfg.REWARD, 'version')
        fprintf(fid, 'reward_version: %s\n', cfg.REWARD.version);
    end

    if isfield(cfg, 'REWARD') && isfield(cfg.REWARD, 'description')
        fprintf(fid, 'reward_description: %s\n', cfg.REWARD.description);
    end

    fprintf(fid, 'mission_target_m: %.6f\n', cfg.MISSION.D_TARGET_M);
    fprintf(fid, 'mission_duration_s: %.6f\n', cfg.MISSION_DURATION);
    fprintf(fid, 'episode_steps: %d\n', cfg.EP_STEPS);
    fprintf(fid, 'chunk_duration_s: %.6f\n', cfg.CHUNK_DURATION);
    fprintf(fid, 'apply_every: %d\n', cfg.APPLY_EVERY);

    fprintf(fid, 'gamma_v_min: %.6f\n', cfg.GAMMA_V_MIN);
    fprintf(fid, 'gamma_v_max: %.6f\n', cfg.GAMMA_V_MAX);
    fprintf(fid, 'v_min: %.6f\n', cfg.V_MIN);
    fprintf(fid, 'v_max: %.6f\n', cfg.V_MAX);
    fprintf(fid, 'v_exec_cap: %.6f\n', cfg.V_MIN + cfg.GAMMA_V_MAX * (cfg.V_MAX - cfg.V_MIN));

    fprintf(fid, 'gamma_a_min: %.6f\n', cfg.GAMMA_A_MIN);
    fprintf(fid, 'gamma_a_max: %.6f\n', cfg.GAMMA_A_MAX);
    fprintf(fid, 'dr_max: %.6f\n', cfg.DR_MAX);

    fprintf(fid, 'battery_parallel: %d\n', cfg.BATTERY.n_parallel);
    fprintf(fid, 'battery_terminal_margin: %.6f\n', cfg.BATTERY.terminal_margin);

    fprintf(fid, 'reward_w_pace: %.6f\n', cfg.REWARD.w_pace);
    fprintf(fid, 'reward_w_shortfall: %.6f\n', cfg.REWARD.w_shortfall);
    fprintf(fid, 'reward_w_ahead: %.6f\n', cfg.REWARD.w_ahead);
    fprintf(fid, 'reward_w_risk: %.6f\n', cfg.REWARD.w_risk);
    fprintf(fid, 'reward_final_soc_bonus: %.6f\n', cfg.REWARD.final_soc_bonus);

    if isfield(cfg.REWARD, 'ADAPT')
        fprintf(fid, 'adapt_enable: %d\n', cfg.REWARD.ADAPT.enable);
        fprintf(fid, 'adapt_w_efficiency_bonus: %.6f\n', cfg.REWARD.ADAPT.w_efficiency_bonus);
        fprintf(fid, 'adapt_w_excess_speed: %.6f\n', cfg.REWARD.ADAPT.w_excess_speed);
        fprintf(fid, 'adapt_w_cap_use: %.6f\n', cfg.REWARD.ADAPT.w_cap_use);
        fprintf(fid, 'adapt_w_current_conserve: %.6f\n', cfg.REWARD.ADAPT.w_current_conserve);
        fprintf(fid, 'adapt_w_terminal_soc: %.6f\n', cfg.REWARD.ADAPT.w_terminal_soc);
        fprintf(fid, 'adapt_I_conserve_high: %.6f\n', cfg.REWARD.ADAPT.I_conserve_high);
        fprintf(fid, 'adapt_I_conserve_low: %.6f\n', cfg.REWARD.ADAPT.I_conserve_low);
        fprintf(fid, 'adapt_I_conserve_scale: %.6f\n', cfg.REWARD.ADAPT.I_conserve_scale);
    end
end