function [initialObs, logged] = rlResetFunction()
    persistent episodeCounter

    rootDir = fileparts(mfilename('fullpath'));
    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');
    snapPath = fullfile(rootDir, 'SimSnapshot_RL.mat');
    lastRPath = fullfile(rootDir, 'LastR.mat');

    S = load(cfgPath, 'cfg', 'lower_abs', 'upper_abs', 'initial_R');
    cfg = S.cfg;
    lower_abs = S.lower_abs;
    upper_abs = S.upper_abs;
    initial_R = S.initial_R;

    if exist(snapPath, 'file')
        delete(snapPath);
    end

    clear fcn_FSM fcn_FSM_bound fcn_gen_XdUd dynamics_SRB

    if isfield(cfg, 'RESET_R_EACH_EPISODE') && cfg.RESET_R_EACH_EPISODE
        last_R = initial_R;
        save(lastRPath, 'last_R');
    else
        if exist(lastRPath, 'file')
            Slast = load(lastRPath, 'last_R');
            last_R = Slast.last_R;
        else
            last_R = initial_R;
            save(lastRPath, 'last_R');
        end
    end

    [v_req, a_req] = sample_reference_command(cfg);

    battery = struct();
    battery.metric_type = cfg.BATTERY.metric_type;
    battery.metric_value = cfg.BATTERY.metric_init;
    battery.margin_norm = cfg.BATTERY.SOC_init;
    battery.soc_pct = 100 * cfg.BATTERY.SOC_init;
    battery.n_series = cfg.BATTERY.n_series;
    battery.n_parallel = cfg.BATTERY.n_parallel;

    if isempty(episodeCounter)
        episodeCounter = 0;
    end

    if isfield(cfg, 'LOG') && cfg.LOG.enable
        episodeCounter = episodeCounter + 1;
    end

    logged = struct();
    logged.episode_idx = episodeCounter;
    logged.step_idx = 0;
    logged.last_R = last_R;
    logged.v_req = v_req;
    logged.a_req = a_req;
    logged.v_exec = v_req;
    logged.a_exec = a_req;
    logged.prev_gamma_v = cfg.GAMMA_V_MISSION;
    logged.prev_gamma_a = cfg.GAMMA_A_MIN;
    logged.prev_Ieq_window = 0;
    logged.distance_m = 0;
    logged.battery = battery;
    logged.window = struct();
    logged.episode_charge_total = 0;
    logged.episode_time_total = 0;

    initialObs = build_rl_observation( ...
        0, 0, battery, 0, 0, ...
        v_req, a_req, cfg.MISSION.D_TARGET_M / cfg.MISSION_DURATION, a_req, ...
        last_R, lower_abs, upper_abs, cfg, ...
        0, 1, 0, 0, ...
        cfg.GAMMA_V_MISSION, cfg.GAMMA_A_MIN, 0); % CoM speed = 0, stance ratio = 1, state norm proxy = 0, FSM proxy = 0

    if isfield(cfg, 'LOG') && cfg.LOG.enable && cfg.LOG.print_episode
        fprintf('\n[EP %d START] horizon=%.0f s, decisions=%d, chunks/decision=%d, SOC0=%.2f%%, v_req=%.3f, a_req=%.3f\n', ...
            logged.episode_idx, cfg.MISSION_DURATION, cfg.EP_STEPS, cfg.APPLY_EVERY, battery.soc_pct, v_req, a_req);
    end
end