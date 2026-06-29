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

    lag_frac = max(0, window.lag_frac);

    soc_frac = window.soc_end_pct / 100;
    soc_stress = pos((cfg.REWARD.soc_safe_thresh - soc_frac) / ...
        max(cfg.REWARD.soc_safe_thresh - cfg.REWARD.soc_terminal_thresh, eps));
    soc_stress = min(max(soc_stress, 0), 1);
    
    k_after = max(1, min(cfg.EP_STEPS, round(window.time_frac * cfg.EP_STEPS)));
    k_before = max(k_after - 1, 0);
    windows_left_including_current = max(cfg.EP_STEPS - k_before, 1);
    
    distance_remaining_at_start = max(cfg.MISSION.D_TARGET_M - window.distance_start_m, 0);
    dynamic_window_target_m = distance_remaining_at_start / windows_left_including_current;
    
    dynamic_window_target_m = max(dynamic_window_target_m, cfg.REWARD.min_window_target_m);
    
    effective_window_target_m = ...
        (1 - soc_stress) * cfg.MISSION.WINDOW_TARGET_M + ...
        soc_stress * dynamic_window_target_m;
    
    q_pace = max(0, window.window_distance_m) / max(effective_window_target_m, eps);
    
    pace_shortfall = pos(1 - q_pace);
    pace_ahead = pos(q_pace - 1);
    
    ahead_gain = cfg.REWARD.w_ahead * (1 - 0.75 * soc_stress);
    
    pace_reward = ...
        cfg.REWARD.w_pace * (q_pace - 1) ...
        - cfg.REWARD.w_shortfall * pace_shortfall^2 ...
        + ahead_gain * log(1 + pace_ahead);
    
    lag_penalty = ...
        cfg.REWARD.w_lag_linear * lag_frac ...
        + cfg.REWARD.w_lag_quad * lag_frac^2;
    
    soc_penalty_gain = 1 + cfg.REWARD.soc_gate_strength * soc_stress^2;
    
    I_budget = ...
        (1 - soc_stress) * cfg.REWARD.I_budget_high + ...
        soc_stress * cfg.REWARD.I_budget_low;
    
    I_budget_excess = capped_pos( ...
        (window.Ieq_window - I_budget) / cfg.REWARD.I_budget_scale, ...
        cfg.REWARD.risk_component_cap);
    
    battery_penalty = soc_penalty_gain * ...
        (cfg.REWARD.w_I * Ieq_norm + cfg.REWARD.w_dsoc * dsoc) ...
        + cfg.REWARD.w_I_budget * I_budget_excess;

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

    if isfield(window, 'state_norm_proxy')
        state_norm_proxy = window.state_norm_proxy;
    else
        state_norm_proxy = 0;
    end
    
    if isfield(window, 'com_speed_mag')
        com_speed_mag = window.com_speed_mag;
    else
        com_speed_mag = 0;
    end
    
    risk_state = capped_pos( ...
        (state_norm_proxy - cfg.REWARD.risk_state_thr) / cfg.REWARD.risk_state_scale, ...
        cfg.REWARD.risk_component_cap);
    
    risk_com = capped_pos( ...
        (com_speed_mag - cfg.REWARD.risk_com_thr) / cfg.REWARD.risk_com_scale, ...
        cfg.REWARD.risk_component_cap);
    
    required_v_now = dynamic_window_target_m / max(cfg.CHUNK_DURATION * cfg.APPLY_EVERY, eps);

    behind_gate = clamp01( ...
        (lag_frac - cfg.REWARD.behind_lag_free) / ...
        max(cfg.REWARD.behind_lag_width, eps));
    
    v_shortfall = capped_pos( ...
        (required_v_now - window.v_exec) / ...
        max(cfg.REWARD.v_shortfall_scale, eps), ...
        cfg.REWARD.risk_component_cap);
    
    mission_guard_penalty = ...
        cfg.REWARD.w_v_shortfall * behind_gate * v_shortfall^2;
    
    risk_excess_speed = capped_pos( ...
        (window.v_exec - required_v_now - cfg.REWARD.v_margin_above_req) / cfg.REWARD.v_excess_scale, ...
        cfg.REWARD.risk_component_cap);
    
    risk_speed_state = risk_excess_speed * (risk_state + 0.5 * risk_com);

    dyn_gate = clamp01((window.v_exec - cfg.REWARD.dynamic_gate_v_thr) / ...
        max(cfg.REWARD.dynamic_gate_v_scale, eps));

    risk_static = ...
        cfg.REWARD.alpha_I * risk_I + ...
        cfg.REWARD.alpha_track * risk_track + ...
        cfg.REWARD.alpha_a * risk_a + ...
        cfg.REWARD.alpha_state * risk_state + ...
        cfg.REWARD.alpha_com * risk_com;
    
    risk_transition = dyn_gate * ( ...
        cfg.REWARD.alpha_dv * risk_dv + ...
        cfg.REWARD.alpha_dgv * risk_dgv + ...
        cfg.REWARD.alpha_r2 * risk_r2 + ...
        cfg.REWARD.alpha_speed_state * risk_speed_state);
    
    risk_score = risk_static + risk_transition;

    adaptive_conservation_penalty = 0;
    adaptive_excess_speed_penalty = 0;
    adaptive_cap_penalty = 0;
    adaptive_current_penalty = 0;
    adaptive_efficiency_reward = 0;
    adaptive_terminal_soc_bonus = 0;
    schedule_balance = window.progress_frac - window.time_frac;
    schedule_gate = 0;
    conservation_gate = 0;
    excess_speed_adapt = 0;
    cap_use_adapt = 0;
    I_conserve_budget = NaN;
    I_conserve_excess = 0;
    I_conserve_margin = 0;
    effective_v_cap = cfg.V_MIN + cfg.GAMMA_V_MAX * (cfg.V_MAX - cfg.V_MIN);
    
    if isfield(cfg.REWARD, 'ADAPT') && cfg.REWARD.ADAPT.enable
        schedule_gate = clamp01( ...
            (schedule_balance + cfg.REWARD.ADAPT.lag_tolerance) / ...
            max(cfg.REWARD.ADAPT.schedule_gate_width + cfg.REWARD.ADAPT.lag_tolerance, eps));
    
        soc_conserve_multiplier = ...
            cfg.REWARD.ADAPT.conserve_above50_weight + ...
            (1 - cfg.REWARD.ADAPT.conserve_above50_weight) * soc_stress;
    
        conservation_gate = schedule_gate * soc_conserve_multiplier;
    
        excess_speed_adapt = capped_pos( ...
            (window.v_exec - required_v_now - cfg.REWARD.ADAPT.v_excess_slack) / ...
            max(cfg.REWARD.ADAPT.v_excess_scale, eps), ...
            cfg.REWARD.risk_component_cap);
    
        cap_use_adapt = capped_pos( ...
            (window.v_exec - (effective_v_cap - cfg.REWARD.ADAPT.cap_band)) / ...
            max(cfg.REWARD.ADAPT.cap_band, eps), ...
            cfg.REWARD.risk_component_cap);
    
        I_conserve_budget = ...
            (1 - soc_stress) * cfg.REWARD.ADAPT.I_conserve_high + ...
            soc_stress * cfg.REWARD.ADAPT.I_conserve_low;
    
        I_conserve_excess = capped_pos( ...
            (window.Ieq_window - I_conserve_budget) / ...
            max(cfg.REWARD.ADAPT.I_conserve_scale, eps), ...
            cfg.REWARD.risk_component_cap);
        
        I_conserve_margin = capped_pos( ...
            (I_conserve_budget - window.Ieq_window) / ...
            max(cfg.REWARD.ADAPT.I_conserve_scale, eps), ...
            cfg.REWARD.risk_component_cap);
        
        adaptive_efficiency_reward = ...
            cfg.REWARD.ADAPT.w_efficiency_bonus * conservation_gate * I_conserve_margin;
        
        adaptive_excess_speed_penalty = ...
            cfg.REWARD.ADAPT.w_excess_speed * conservation_gate * excess_speed_adapt^2;
    
        adaptive_cap_penalty = ...
            cfg.REWARD.ADAPT.w_cap_use * conservation_gate * cap_use_adapt^2;
    
        adaptive_current_penalty = ...
            cfg.REWARD.ADAPT.w_current_conserve * conservation_gate * I_conserve_excess^2;
    
        adaptive_conservation_penalty = ...
            adaptive_excess_speed_penalty + ...
            adaptive_cap_penalty + ...
            adaptive_current_penalty;
    end

    reward = ...
        pace_reward ...
        - lag_penalty ...
        - cfg.REWARD.w_risk * risk_score ...
        - battery_penalty ...
        + adaptive_efficiency_reward ...
        - adaptive_conservation_penalty ...
        - cfg.REWARD.w_track * nt ...
        - cfg.REWARD.w_effort * nu ...
        - cfg.REWARD.w_slow * slow_pen ...
        - mission_guard_penalty;

    if strcmp(window.terminal_reason, 'mission_complete')
        adaptive_terminal_soc_bonus = 0;
    
        reward = reward ...
            + cfg.REWARD.complete_bonus ...
            + cfg.REWARD.early_bonus * (1 - window.time_frac);

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

    info.dynamic_window_target_m = dynamic_window_target_m;
    info.effective_window_target_m = effective_window_target_m;
    info.I_budget = I_budget;
    info.I_budget_excess = I_budget_excess;
    info.risk_state = risk_state;
    info.risk_com = risk_com;
    info.risk_excess_speed = risk_excess_speed;
    info.risk_speed_state = risk_speed_state;
    info.required_v_now = required_v_now;

    info.schedule_balance = schedule_balance;
    info.schedule_gate = schedule_gate;
    info.conservation_gate = conservation_gate;
    
    info.effective_v_cap = effective_v_cap;
    info.excess_speed_adapt = excess_speed_adapt;
    info.cap_use_adapt = cap_use_adapt;
    info.I_conserve_budget = I_conserve_budget;
    info.I_conserve_excess = I_conserve_excess;

    info.I_conserve_margin = I_conserve_margin;
    info.adaptive_efficiency_reward = adaptive_efficiency_reward;
    
    info.adaptive_conservation_penalty = adaptive_conservation_penalty;
    info.adaptive_excess_speed_penalty = adaptive_excess_speed_penalty;
    info.adaptive_cap_penalty = adaptive_cap_penalty;
    info.adaptive_current_penalty = adaptive_current_penalty;
    info.adaptive_terminal_soc_bonus = adaptive_terminal_soc_bonus;
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