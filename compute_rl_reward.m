function [reward, info] = compute_rl_reward(window, cfg)
    nt_raw = window.tracking_error_mean / cfg.TRACK_REF;
    nu_raw = window.control_effort_mean / cfg.EFFORT_REF;

    nt = min(max(nt_raw, 0), cfg.REWARD.nt_cap);
    nu = min(max(nu_raw, 0), cfg.REWARD.nu_cap);

    if isfinite(window.Ieq_window)
        Ieq_norm = window.Ieq_window / cfg.IEQ_REF;
    else
        Ieq_norm = 1;
    end

    dsoc = max(0, (window.soc_start_pct - window.soc_end_pct) / 100);

    q_pace = max(0, window.window_distance_m) / max(cfg.MISSION.WINDOW_TARGET_M, eps);
    lag_frac = max(0, window.lag_frac);

    pace_shortfall = pos(1 - q_pace);
    pace_ahead = pos(q_pace - 1);

    pace_reward = ...
        cfg.REWARD.w_pace * q_pace ...
        - cfg.REWARD.w_shortfall * pace_shortfall ...
        + cfg.REWARD.w_ahead * log(1 + pace_ahead);

    lag_penalty = ...
        cfg.REWARD.w_lag_linear * lag_frac ...
        + cfg.REWARD.w_lag_quad * lag_frac^2;

    soc_frac = window.soc_end_pct / 100;
    soc_stress = pos((cfg.REWARD.soc_safe_thresh - soc_frac) / ...
        max(cfg.REWARD.soc_safe_thresh - cfg.REWARD.soc_terminal_thresh, eps));

    soc_penalty_gain = 1 + cfg.REWARD.soc_gate_strength * soc_stress^2;

    battery_penalty = soc_penalty_gain * ...
        (cfg.REWARD.w_I * Ieq_norm + cfg.REWARD.w_dsoc * dsoc);

    slow_pen = pos((cfg.REWARD.v_floor_soft - window.v_exec) / ...
        max(cfg.REWARD.v_floor_soft - cfg.V_MIN, eps));

    risk_I = capped_pos( ...
        (window.Ieq_window - cfg.REWARD.risk_I_thr) / cfg.REWARD.risk_I_scale, ...
        cfg.REWARD.risk_component_cap);

    track_log = log1p(max(window.tracking_error_mean, 0) / cfg.REWARD.risk_track_ref);
    track_thr_log = log1p(cfg.REWARD.risk_track_thr / cfg.REWARD.risk_track_ref);
    risk_track = capped_pos(track_log - track_thr_log, cfg.REWARD.risk_component_cap);

    risk_a = capped_pos( ...
        (window.a_exec - cfg.REWARD.risk_a_thr) / cfg.REWARD.risk_a_scale, ...
        cfg.REWARD.risk_component_cap);

    risk_dv = capped_pos( ...
        (window.delta_v_exec - cfg.REWARD.risk_dv_thr) / cfg.REWARD.risk_dv_scale, ...
        cfg.REWARD.risk_component_cap);

    risk_dgv = capped_pos( ...
        (window.delta_gamma_v - cfg.REWARD.risk_dgv_thr) / cfg.REWARD.risk_dgv_scale, ...
        cfg.REWARD.risk_component_cap);

    risk_r2 = capped_pos( ...
        ((-window.dR2) - cfg.REWARD.risk_r2_thr) / cfg.REWARD.risk_r2_scale, ...
        cfg.REWARD.risk_component_cap);

    dyn_gate = clamp01((window.v_exec - cfg.REWARD.dynamic_gate_v_thr) / ...
        max(cfg.REWARD.dynamic_gate_v_scale, eps));

    risk_static = ...
        cfg.REWARD.alpha_I * risk_I + ...
        cfg.REWARD.alpha_track * risk_track + ...
        cfg.REWARD.alpha_a * risk_a;

    risk_transition = dyn_gate * ( ...
        cfg.REWARD.alpha_dv * risk_dv + ...
        cfg.REWARD.alpha_dgv * risk_dgv + ...
        cfg.REWARD.alpha_r2 * risk_r2);

    risk_score = risk_static + risk_transition;

    reward = ...
        pace_reward ...
        - lag_penalty ...
        - cfg.REWARD.w_risk * risk_score ...
        - battery_penalty ...
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
            - (cfg.REWARD.infeasible_base ...
            + cfg.REWARD.infeasible_remaining * (1 - window.progress_frac) ...
            + cfg.REWARD.infeasible_lag * lag_frac);

    elseif strcmp(window.terminal_reason, 'battery_terminal')
        reward = reward ...
            - (cfg.REWARD.battery_base ...
            + cfg.REWARD.battery_remaining * (1 - window.progress_frac));

    elseif strcmp(window.terminal_reason, 'time_limit')
        reward = reward ...
            - (cfg.REWARD.time_limit_base ...
            + cfg.REWARD.time_limit_remaining * (1 - window.progress_frac));
    end

    info = struct();
    info.nt_raw = nt_raw;
    info.nu_raw = nu_raw;
    info.nt = nt;
    info.nu = nu;
    info.Ieq_norm = Ieq_norm;
    info.dsoc = dsoc;

    info.q_pace = q_pace;
    info.pace_shortfall = pace_shortfall;
    info.pace_ahead = pace_ahead;
    info.pace_reward = pace_reward;

    info.lag_frac = lag_frac;
    info.lag_penalty = lag_penalty;

    info.soc_frac = soc_frac;
    info.soc_stress = soc_stress;
    info.soc_penalty_gain = soc_penalty_gain;
    info.battery_penalty = battery_penalty;

    info.slow_pen = slow_pen;

    info.risk_score = risk_score;
    info.risk_static = risk_static;
    info.risk_transition = risk_transition;
    info.dyn_gate = dyn_gate;

    info.risk_I = risk_I;
    info.risk_track = risk_track;
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

function y = capped_pos(x, capVal)
    y = min(max(x, 0), capVal);
end

function y = clamp01(x)
    y = min(max(x, 0), 1);
end