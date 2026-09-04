function state = initialize_mpc_replay_state(cfg, gait)
%initialize_mpc_replay_state Construct a complete deterministic replay state.

    if nargin < 2
        gait = 0;
    end
    bootstrap_RF_MPC_RL();
    reset_mpc_case_state();
    p = get_params(gait);
    if gait == 1
        [~, Xt, Ut] = fcn_bound_ref_traj(p);
    else
        [Xt, Ut] = fcn_gen_XdUd(0, [], ones(4, 1), p);
    end

    state = struct();
    state.schema_version = 'mpc_replay_state_v1';
    state.gait = gait;
    state.t = 0;
    state.Xt = Xt;
    state.Ut = Ut;
    state.fsm_internal_state = struct();
    state.knee_proxy_state = init_knee_proxy_state();
    state.knee_template = load_knee_template( ...
        cfg.PROXY.kneeCsv, cfg.PROXY.betaMc);
    kneeData = readmatrix(cfg.PROXY.kneeCsv);
    state.knee_parameters = struct( ...
        'Tmc', kneeData(end, 1) - kneeData(1, 1), ...
        'alpha', cfg.PROXY.alpha, ...
        'beta', cfg.PROXY.beta, ...
        'Kt', cfg.PROXY.Kt, ...
        'eta', cfg.PROXY.eta, ...
        'Nknee', cfg.PROXY.Nknee);
    state.hip_current_A = joint_torque_to_current( ...
        cfg.PROXY.tauHip4_joint, cfg.PROXY.Nhip, ...
        cfg.PROXY.eta, cfg.PROXY.Kt);
    state.current_time = [];
    state.current_total = [];
    state.battery = struct( ...
        'metric_type', cfg.BATTERY.metric_type, ...
        'metric_value', cfg.BATTERY.metric_init, ...
        'margin_norm', cfg.BATTERY.SOC_init, ...
        'soc_pct', 100*cfg.BATTERY.SOC_init, ...
        'n_series', cfg.BATTERY.n_series, ...
        'n_parallel', cfg.BATTERY.n_parallel);
    rDiagonal = diag(p.R);
    state.R = rDiagonal(1:3);
    state.previous_action = nan(5, 1);
    state.decision_bookkeeping = struct();
    state.mpc_warm_start = [];
end
