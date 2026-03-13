function obs = build_rl_observation(nt, nu, battery, progress_frac, v_req, a_req, v_exec, a_exec, R_now, lower_abs, upper_abs, cfg)
    obs = [
        nt
        nu
        clamp01(battery.margin_norm)
        clamp01(progress_frac)
        normalize(v_req, cfg.V_MIN, cfg.V_MAX)
        normalize(a_req, cfg.A_MIN, cfg.A_MAX)
        normalize(v_exec, cfg.V_MIN, cfg.V_MAX)
        normalize(a_exec, cfg.A_MIN, cfg.A_MAX)
        normalize(R_now(1), lower_abs(1), upper_abs(1))
        normalize(R_now(2), lower_abs(2), upper_abs(2))
        normalize(R_now(3), lower_abs(3), upper_abs(3))
    ];
end

function y = normalize(x, xmin, xmax)
    y = (x - xmin) / max(xmax - xmin, eps);
    y = clamp01(y);
end

function y = clamp01(x)
    y = min(max(x, 0), 1);
end