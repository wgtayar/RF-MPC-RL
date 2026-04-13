function out = run_MPC_simulation(R_weights, gait, v_cmd, a_cmd, cfg)
    if nargin < 2 || isempty(gait)
        gait = 0;
    end

    p = get_params(gait);
    p.R = diag(R_weights);
    p.vel_d = [v_cmd; 0];
    p.acc_d = a_cmd;
    p.yaw_d = 0;

    fail_reason = '';

    dt_sim = p.simTimeStep;
    max_iter = floor(cfg.CHUNK_DURATION / dt_sim);
    rootDir = fileparts(mfilename('fullpath'));
    snap_path = fullfile(rootDir, 'SimSnapshot_RL.mat');

    kneeCsv = cfg.PROXY.kneeCsv;
    betaMc = cfg.PROXY.betaMc;
    kneeTpl = load_knee_template(kneeCsv, betaMc);

    Dmc = readmatrix(kneeCsv);
    Tmc = Dmc(end,1) - Dmc(1,1);

    kneeParams = struct();
    kneeParams.Tmc = Tmc;
    kneeParams.alpha = cfg.PROXY.alpha;
    kneeParams.beta = cfg.PROXY.beta;
    kneeParams.Kt = cfg.PROXY.Kt;
    kneeParams.eta = cfg.PROXY.eta;
    kneeParams.Nknee = cfg.PROXY.Nknee;

    Ihip4 = joint_torque_to_current(cfg.PROXY.tauHip4_joint, cfg.PROXY.Nhip, cfg.PROXY.eta, cfg.PROXY.Kt);

    if exist(snap_path, 'file')
        Ssnap = load(snap_path, 'Sim');
        Sim = Ssnap.Sim;
        Xt = Sim.Xt;
        Ut = Sim.Ut;
    else
        if gait == 1
            [p, Xt, Ut] = fcn_bound_ref_traj(p);
        else
            [Xt, Ut] = fcn_gen_XdUd(0, [], [1;1;1;1], p);
        end

        Sim = struct();
        Sim.t = 0;
        Sim.Xt = Xt;
        Sim.Ut = Ut;
        Sim.current_time = [];
        Sim.current_total = [];
        Sim.battery = struct();
        Sim.battery.metric_type = cfg.BATTERY.metric_type;
        Sim.battery.metric_value = cfg.BATTERY.metric_init;
        Sim.battery.margin_norm = cfg.BATTERY.SOC_init;
        Sim.battery.soc_pct = 100 * cfg.BATTERY.SOC_init;
        Sim.battery.n_series = cfg.BATTERY.n_series;
        Sim.battery.n_parallel = cfg.BATTERY.n_parallel;
        Sim.kneeProxyState = init_knee_proxy_state();
    end

    if ~isfield(Sim, 'current_time')
        Sim.current_time = [];
    end
    if ~isfield(Sim, 'current_total')
        Sim.current_total = [];
    end
    if ~isfield(Sim, 'battery') || isempty(Sim.battery)
        Sim.battery.metric_type = cfg.BATTERY.metric_type;
        Sim.battery.metric_value = cfg.BATTERY.metric_init;
        Sim.battery.margin_norm = cfg.BATTERY.SOC_init;
        Sim.battery.soc_pct = 100 * cfg.BATTERY.SOC_init;
        Sim.battery.n_series = cfg.BATTERY.n_series;
        Sim.battery.n_parallel = cfg.BATTERY.n_parallel;
    end
    if ~isfield(Sim, 'kneeProxyState') || isempty(Sim.kneeProxyState)
        Sim.kneeProxyState = init_knee_proxy_state();
    end

    qp_options = optimoptions('quadprog', 'Display', 'off');

    tracking_error = zeros(max_iter, 1);
    control_effort = zeros(max_iter, 1);
    state_norm = nan(max_iter, 1);
    input_norm = nan(max_iter, 1);
    com_speed = nan(max_iter, 1);
    qp_exitflag = nan(max_iter, 1);
    fsm_leg1 = nan(max_iter, 1);
    iter_time = nan(max_iter, 1);
    knee_t = nan(max_iter, 1);
    knee_tau4 = nan(max_iter, 1);
    knee_I4 = nan(max_iter, 1);
    
    feasible = true;
    fail_iter = NaN;
    fail_time_s = NaN;
    fail_state_norm = NaN;
    fail_input_norm = NaN;
    fail_com_speed = NaN;
    fail_fsm_leg1 = NaN;
    fail_h_rcond = NaN;
    fail_Aineq_rows = NaN;
    fail_Aineq_cols = NaN;
    fail_Aeq_rows = NaN;
    fail_Aeq_cols = NaN;

    try
        for ii = 1:max_iter
            t0_abs = Sim.t + dt_sim * (ii - 1);
            t_hor = t0_abs + p.Tmpc * (0:p.predHorizon-1);

            iter_time(ii) = t0_abs;

            if gait == 1
                [FSM, Xd, Ud, Xt] = fcn_FSM_bound(t_hor, Xt, p);
            else
                [FSM, Xd, Ud, Xt] = fcn_FSM(t_hor, Xt, p);
            end

            fsm_leg1(ii) = FSM(1);

            [Sim.kneeProxyState, kneeOut] = knee_proxy_step(t0_abs, FSM(1), Sim.kneeProxyState, kneeTpl, kneeParams);
            knee_t(ii) = kneeOut.t;
            knee_tau4(ii) = kneeOut.tau4;
            knee_I4(ii) = kneeOut.I4;

            [H, g, Aineq, bineq, Aeq, beq] = fcn_get_QP_form_eta(Xt, Ut, Xd, Ud, p);
            
            fail_h_rcond = rcond(H);
            fail_Aineq_rows = size(Aineq,1);
            fail_Aineq_cols = size(Aineq,2);
            fail_Aeq_rows = size(Aeq,1);
            fail_Aeq_cols = size(Aeq,2);

            [zval, ~, exitflag] = quadprog(H, g, Aineq, bineq, Aeq, beq, [], [], [], qp_options);

            qp_exitflag(ii) = exitflag;

            if exitflag <= 0 || isempty(zval)
                feasible = false;
                fail_reason = 'quadprog';
                fail_iter = ii;
                fail_time_s = t0_abs;
                fail_state_norm = norm(Xt);
                fail_input_norm = norm(Ut);
                fail_com_speed = norm(Xt(4:5));
                fail_fsm_leg1 = FSM(1);
                break
            end

            Ut = Ut + zval(1:12);

            [u_ext, p_ext] = fcn_get_disturbance(t0_abs, p);
            p.p_ext = p_ext;
            u_ext = 0 * u_ext;

            [~, X_chunk] = ode45(@(t, X) dynamics_SRB(t, X, Ut, Xd, u_ext, p), [t0_abs, t0_abs + dt_sim], Xt);
            Xt = X_chunk(end,:).';

            if any(isnan(Xt)) || any(isinf(Xt))
                feasible = false;
                fail_reason = 'state_invalid';
                break
            end

            tracking_error(ii) = sum((Xt - Xd(:,1)).^2);
            control_effort(ii) = sum(Ut.^2);

            state_norm(ii) = norm(Xt);
            input_norm(ii) = norm(Ut);
            com_speed(ii) = norm(Xt(4:5));
        end
    catch ME
        feasible = false;
        fail_reason = ME.identifier;
    end

    out = struct();

    if ~feasible
        Sim.Xt = Xt;
        Sim.Ut = Ut;
        save(snap_path, 'Sim');
    
        out.tracking_error_total = NaN;
        out.control_effort_total = NaN;
        out.charge_total = 0;
        out.current_duration = 0;
        out.battery = Sim.battery;
        out.feasible = false;
        out.fail_reason = fail_reason;
    
        out.fail_iter = fail_iter;
        out.fail_time_s = fail_time_s;
        out.fail_state_norm = fail_state_norm;
        out.fail_input_norm = fail_input_norm;
        out.fail_com_speed = fail_com_speed;
        out.fail_fsm_leg1 = fail_fsm_leg1;
        out.fail_h_rcond = fail_h_rcond;
        out.fail_Aineq_rows = fail_Aineq_rows;
        out.fail_Aineq_cols = fail_Aineq_cols;
        out.fail_Aeq_rows = fail_Aeq_rows;
        out.fail_Aeq_cols = fail_Aeq_cols;
        out.qp_exitflag_last = qp_exitflag(max(find(isfinite(qp_exitflag),1,'last'),1));
    
        out.state_norm_end = norm(Xt);
        out.input_norm_end = norm(Ut);
        out.com_speed_end = norm(Xt(4:5));
        out.fsm_leg1_end = NaN;
    
        return
    end

    valid = isfinite(knee_t) & isfinite(knee_I4);
    t_chunk = knee_t(valid);
    I_knee = knee_I4(valid);

    if numel(t_chunk) >= 2
        [t_chunk, idx] = sort(t_chunk);
        I_knee = I_knee(idx);
        I_total = I_knee + Ihip4;

        if ~isempty(Sim.current_time)
            keep = t_chunk > Sim.current_time(end);
            t_chunk = t_chunk(keep);
            I_total = I_total(keep);
        end

        Sim.current_time = [Sim.current_time; t_chunk];
        Sim.current_total = [Sim.current_total; I_total];

        if numel(t_chunk) >= 2
            charge_total = trapz(t_chunk, abs(I_total));
            current_duration = t_chunk(end) - t_chunk(1);
        else
            charge_total = 0;
            current_duration = 0;
        end
    else
        charge_total = 0;
        current_duration = 0;
    end

    battery = evaluate_battery_feedback(Sim.current_time, Sim.current_total, cfg.BATTERY, Sim.battery);
    Sim.battery = battery;
    Sim.bms_input = battery.bms_input;

    Sim.t = Sim.t + cfg.CHUNK_DURATION;
    Sim.Xt = Xt;
    Sim.Ut = Ut;

    save(snap_path, 'Sim');

    out.tracking_error_total = sum(tracking_error);
    out.control_effort_total = sum(control_effort);
    out.charge_total = charge_total;
    out.current_duration = current_duration;
    out.battery = battery;
    out.feasible = true;
    out.fail_reason = '';

    out.fail_iter = NaN;
    out.fail_time_s = NaN;
    out.fail_state_norm = NaN;
    out.fail_input_norm = NaN;
    out.fail_com_speed = NaN;
    out.fail_fsm_leg1 = NaN;
    out.fail_h_rcond = NaN;
    out.fail_Aineq_rows = NaN;
    out.fail_Aineq_cols = NaN;
    out.fail_Aeq_rows = NaN;
    out.fail_Aeq_cols = NaN;
    out.qp_exitflag_last = qp_exitflag(max(find(isfinite(qp_exitflag),1,'last'),1));
    
    out.state_norm_end = norm(Xt);
    out.input_norm_end = norm(Ut);
    out.com_speed_end = norm(Xt(4:5));
    out.fsm_leg1_end = fsm_leg1(max(find(isfinite(fsm_leg1),1,'last'),1));
end