function [v_exec, a_exec, gov] = apply_command_governor(v_req, a_req, gamma_v, gamma_a, cfg)
    gamma_v = min(max(gamma_v, cfg.GAMMA_V_MIN), cfg.GAMMA_V_MAX);
    gamma_a = min(max(gamma_a, cfg.GAMMA_A_MIN), cfg.GAMMA_A_MAX);

    v_exec = v_req * gamma_v;
    a_exec = a_req * gamma_a;

    v_exec = min(max(v_exec, cfg.V_MIN), cfg.V_MAX);

    a_lo = max(cfg.A_MIN, v_exec / cfg.TACC_MAX);
    a_hi = min(cfg.A_MAX, v_exec / cfg.TACC_MIN);

    if a_lo > a_hi
        a_lo = cfg.A_MIN;
        a_hi = cfg.A_MAX;
    end

    a_exec = min(max(a_exec, a_lo), a_hi);

    gov = struct();
    gov.gamma_v = gamma_v;
    gov.gamma_a = gamma_a;
    gov.a_lo = a_lo;
    gov.a_hi = a_hi;
end