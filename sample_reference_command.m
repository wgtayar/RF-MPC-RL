function [v_req, a_req] = sample_reference_command(cfg)
    if isfield(cfg, 'RANDOMIZE_REQUEST') && ~cfg.RANDOMIZE_REQUEST
        v_req = cfg.V_REQ_FIXED;
        a_req = cfg.A_REQ_FIXED;
        return
    end

    v_req = cfg.V_MIN + (cfg.V_MAX - cfg.V_MIN) * rand();
    a_lo = max(cfg.A_MIN, v_req / cfg.TACC_MAX);
    a_hi = min(cfg.A_MAX, v_req / cfg.TACC_MIN);
    a_req = a_lo + (a_hi - a_lo) * rand();
end