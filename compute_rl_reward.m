function [reward, info] = compute_rl_reward(window, cfg)
    nt = window.tracking_error_mean / cfg.TRACK_REF;
    nu = window.control_effort_mean / cfg.EFFORT_REF;

    if isfinite(window.Ieq_window)
        Ieq_norm = window.Ieq_window / cfg.IEQ_REF;
    else
        Ieq_norm = 1;
    end

    dsoc = max(0, (window.soc_start_pct - window.soc_end_pct) / 100);

    dist_step_norm = window.window_distance_m / max(cfg.MISSION.WINDOW_TARGET_M, eps);
    lag_frac = max(0, window.lag_frac);

    slow_pen = pos((cfg.REWARD.v_floor_soft - window.v_exec) / ...
        max(cfg.REWARD.v_floor_soft - cfg.V_MIN, eps));

    risk_I = pos((window.Ieq_window - cfg.REWARD.risk_I_thr) / cfg.REWARD.risk_I_scale);
    risk_track = pos((window.tracking_error_mean - cfg.REWARD.risk_track_thr) / cfg.REWARD.risk_track_scale);
    risk_v = pos((window.v_exec - cfg.REWARD.risk_v_thr) / cfg.REWARD.risk_v_scale);
    risk_a = pos((window.a_exec - cfg.REWARD.risk_a_thr) / cfg.REWARD.risk_a_scale);
    risk_dv = pos((window.delta_v_exec - cfg.REWARD.risk_dv_thr) / cfg.REWARD.risk_dv_scale);
    risk_dgv = pos((window.delta_gamma_v - cfg.REWARD.risk_dgv_thr) / cfg.REWARD.risk_dgv_scale);
    risk_r2 = pos(((-window.dR2) - cfg.REWARD.risk_r2_thr) / cfg.REWARD.risk_r2_scale);

    risk_score = ...
        cfg.REWARD.alpha_I * risk_I + ...
        cfg.REWARD.alpha_track * risk_track + ...
        cfg.REWARD.alpha_v * risk_v + ...
        cfg.REWARD.alpha_a * risk_a + ...
        cfg.REWARD.alpha_dv * risk_dv + ...
        cfg.REWARD.alpha_dgv * risk_dgv + ...
        cfg.REWARD.alpha_r2 * risk_r2;

    reward = ...
        cfg.REWARD.w_dist * dist_step_norm ...
        - cfg.REWARD.w_lag * lag_frac ...
        - cfg.REWARD.w_risk * risk_score ...
        - cfg.REWARD.w_I * Ieq_norm ...
        - cfg.REWARD.w_dsoc * dsoc ...
        - cfg.REWARD.w_track * nt ...
        - cfg.REWARD.w_effort * nu ...
        - cfg.REWARD.w_slow * slow_pen;

    if strcmp(window.terminal_reason, 'mission_complete')
        reward = reward ...
            + cfg.REWARD.complete_bonus ...
            + cfg.REWARD.early_bonus * (1 - window.time_frac) ...
            + cfg.REWARD.final_soc_bonus * window.battery.margin_norm;

    elseif strcmp(window.terminal_reason, 'infeasible')
        reward = reward ...
            - (cfg.REWARD.infeasible_base + cfg.REWARD.infeasible_remaining * (1 - window.progress_frac));

    elseif strcmp(window.terminal_reason, 'battery_terminal')
        reward = reward ...
            - (cfg.REWARD.battery_base + cfg.REWARD.battery_remaining * (1 - window.progress_frac));

    elseif strcmp(window.terminal_reason, 'time_limit')
        reward = reward ...
            - (cfg.REWARD.time_limit_base + cfg.REWARD.time_limit_remaining * (1 - window.progress_frac));
    end

    info = struct();
    info.nt = nt;
    info.nu = nu;
    info.Ieq_norm = Ieq_norm;
    info.dsoc = dsoc;
    info.dist_step_norm = dist_step_norm;
    info.lag_frac = lag_frac;
    info.slow_pen = slow_pen;
    info.risk_score = risk_score;
    info.risk_I = risk_I;
    info.risk_track = risk_track;
    info.risk_v = risk_v;
    info.risk_a = risk_a;
    info.risk_dv = risk_dv;
    info.risk_dgv = risk_dgv;
    info.risk_r2 = risk_r2;
    info.progress_frac = window.progress_frac;
    info.time_frac = window.time_frac;
end

function y = pos(x)
    y = max(0, x);
end