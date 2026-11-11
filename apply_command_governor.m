function [v_exec, a_exec, gov] = apply_command_governor(v_req, a_req, gamma_v, gamma_a, cfg)
    gamma_v = min(max(gamma_v, cfg.GAMMA_V_MIN), cfg.GAMMA_V_MAX);
    gamma_a = min(max(gamma_a, cfg.GAMMA_A_MIN), cfg.GAMMA_A_MAX);

    v_exec = cfg.V_MIN + gamma_v * (cfg.V_MAX - cfg.V_MIN);
    v_exec = min(max(v_exec, cfg.V_MIN), cfg.V_MAX);

    if isfield(cfg,'ACCELERATION')
        assert(strcmp(cfg.ACCELERATION.action_model,'bounded_transition_v1'), ...
            'governor:AccelerationModel','Unknown acceleration action model.');
        a_lo = max(cfg.A_MIN,a_req*cfg.GAMMA_A_MIN);
        a_hi = min(cfg.A_MAX,a_req*cfg.GAMMA_A_MAX);
        assert(all(isfinite([a_lo,a_hi,cfg.GAMMA_A_MIN,cfg.GAMMA_A_MAX])) && ...
            a_lo>0 && a_hi>a_lo && cfg.GAMMA_A_MAX>cfg.GAMMA_A_MIN, ...
            'governor:AccelerationBounds','Require ordered, positive transition acceleration bounds.');
        fraction = (gamma_a-cfg.GAMMA_A_MIN)/(cfg.GAMMA_A_MAX-cfg.GAMMA_A_MIN);
        a_exec = a_lo+fraction*(a_hi-a_lo);
    else
        % Historical startup-duration rule, unchanged for archived profiles.
        a_exec = a_req * gamma_a;
        a_lo = max(cfg.A_MIN, v_exec / cfg.TACC_MAX);
        a_hi = min(cfg.A_MAX, v_exec / cfg.TACC_MIN);
        if a_lo > a_hi
            a_lo = cfg.A_MIN;
            a_hi = cfg.A_MAX;
        end
        a_exec = min(max(a_exec, a_lo), a_hi);
    end

    gov = struct();
    gov.gamma_v = gamma_v;
    gov.gamma_a = gamma_a;
    gov.v_req = v_req;
    gov.a_req = a_req;
    gov.a_lo = a_lo;
    gov.a_hi = a_hi;
    if isfield(cfg,'ACCELERATION')
        gov.acceleration_action_model = cfg.ACCELERATION.action_model;
        gov.acceleration_semantics = 'Positive desired-ramp magnitude; reference state determines acceleration or deceleration sign.';
    end
end