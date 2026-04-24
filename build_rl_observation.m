function obs = build_rl_observation( ...
    nt, nu, battery, dist_progress_frac, lag_frac, ...
    v_req, a_req, v_exec, a_exec, R_now, lower_abs, upper_abs, cfg, ...
    com_speed_mag, tst_ratio, state_norm_proxy, fsm_proxy, ...
    prev_gamma_v, prev_gamma_a, prev_Ieq_window)

    obs = [
        nt
        nu
        clamp01(battery.margin_norm)
        clamp01(dist_progress_frac)
        clamp01(lag_frac)
        normalize(v_req, cfg.V_MIN, cfg.V_MAX)
        normalize(a_req, cfg.A_MIN, cfg.A_MAX)
        normalize(v_exec, cfg.V_MIN, cfg.V_MAX)
        normalize(a_exec, cfg.A_MIN, cfg.A_MAX)
        normalize(R_now(1), lower_abs(1), upper_abs(1))
        normalize(R_now(2), lower_abs(2), upper_abs(2))
        normalize(R_now(3), lower_abs(3), upper_abs(3))
        normalize(com_speed_mag, 0, cfg.OBS.COM_SPEED_MAX)
        normalize(tst_ratio, cfg.OBS.TST_RATIO_MIN, cfg.OBS.TST_RATIO_MAX)
        normalize(state_norm_proxy, 0, cfg.OBS.STATE_NORM_MAX)
        clamp01(fsm_proxy)
        normalize(prev_gamma_v, cfg.GAMMA_V_MIN, cfg.GAMMA_V_MAX)
        normalize(prev_gamma_a, cfg.GAMMA_A_MIN, cfg.GAMMA_A_MAX)
        normalize(prev_Ieq_window, 0, cfg.IEQ_REF)
    ];
end

function y = normalize(x, xmin, xmax)
    y = (x - xmin) / max(xmax - xmin, eps);
    y = clamp01(y);
end

function y = clamp01(x)
    y = min(max(x, 0), 1);
end