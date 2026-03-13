function [reward, info] = compute_rl_reward(window, cfg)
    nt = window.tracking_error_mean / cfg.TRACK_REF;
    nu = window.control_effort_mean / cfg.EFFORT_REF;

    if isfinite(window.Ieq_window)
        Ieq_norm = window.Ieq_window / cfg.IEQ_REF;
    else
        Ieq_norm = 1;
    end

    ev = (window.v_req - window.v_exec) / max(window.v_req, 1e-6);
    ea = (window.a_req - window.a_exec) / max(window.a_req, 1e-6);
    cmd_penalty = cfg.REWARD.w_vcmd * ev^2 + cfg.REWARD.w_acmd * ea^2;

    soc_now = window.battery.margin_norm;
    dsoc = max(0, (window.soc_start_pct - window.soc_end_pct) / 100);

    low_batt = 1 / (1 + exp(cfg.REWARD.batt_slope * (soc_now - cfg.REWARD.batt_thresh)));
    w_energy = cfg.REWARD.w_energy_high * (1 - low_batt) + cfg.REWARD.w_energy_low * low_batt;
    w_cmd = cfg.REWARD.w_cmd_high * (1 - low_batt) + cfg.REWARD.w_cmd_low * low_batt;

    reward = cfg.REWARD.w_progress ...
        + cfg.REWARD.w_soc * soc_now ...
        - cfg.REWARD.w_track * nt ...
        - cfg.REWARD.w_effort * nu ...
        - w_energy * Ieq_norm ...
        - cfg.REWARD.w_dsoc * dsoc ...
        - cfg.REWARD.w_low_soc * low_batt ...
        - w_cmd * cmd_penalty;

    if window.completed_episode
        reward = reward + cfg.REWARD.completion_bonus;
    end

    if strcmp(window.terminal_reason, 'infeasible')
        reward = reward - (cfg.REWARD.infeasible_base + cfg.REWARD.infeasible_remaining * (1 - window.progress_frac));
    elseif strcmp(window.terminal_reason, 'battery_terminal')
        reward = reward - (cfg.REWARD.battery_base + cfg.REWARD.battery_remaining * (1 - window.progress_frac));
    end

    info = struct();
    info.nt = nt;
    info.nu = nu;
    info.Ieq_norm = Ieq_norm;
    info.cmd_penalty = cmd_penalty;
    info.low_batt = low_batt;
    info.dsoc = dsoc;
    info.progress_frac = window.progress_frac;
end