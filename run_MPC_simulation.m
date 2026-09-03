function out = run_MPC_simulation(R_weights, gait, v_cmd, a_cmd, cfg, diagnosticContext)
    bootstrap_RF_MPC_RL();
    if nargin < 2 || isempty(gait)
        gait = 0;
    end
    if nargin < 6 || isempty(diagnosticContext)
        diagnosticContext = struct();
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

    x_start = Xt(1);

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
    fsm_all = nan(max_iter, 4);
    iter_time = nan(max_iter, 1);
    knee_t = nan(max_iter, 1);
    knee_tau4 = nan(max_iter, 1);
    knee_I4 = nan(max_iter, 1);
    Tst_log = nan(max_iter, 1);
    Tsw_log = nan(max_iter, 1);
    
    feasible = true;
    fail_iter = NaN;
    fail_time_s = NaN;
    fail_state_norm = NaN;
    fail_input_norm = NaN;
    fail_com_speed = NaN;
    fail_fsm_leg1 = NaN;
    fail_fsm_all = nan(1, 4);
    fail_h_rcond = NaN;
    fail_Aineq_rows = NaN;
    fail_Aineq_cols = NaN;
    fail_Aeq_rows = NaN;
    fail_Aeq_cols = NaN;
    failureQpId = "";
    failureQpFile = "";
    failureQP = struct();
    qpSolveCount = 0;
    qpFailedCount = 0;
    inequalityMarginMin = inf;
    activeInequalityCount = 0;
    nearActiveInequalityCount = 0;
    violatedInequalityCount = 0;
    equalityResidualNormMax = 0;
    equalityResidualMaxAbs = 0;
    minimumMarginByLegType = inf(4, 6);
    lastSuccessfulXt = Xt;
    lastSuccessfulUt = Ut;
    lastSuccessfulXd = [];
    lastSuccessfulUd = [];
    lastSuccessfulFSM = nan(4, 1);
    lastQpOutput = struct();
    lastLambda = struct();

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

            fsm_all(ii,:) = FSM(:).';

            [Sim.kneeProxyState, kneeOut] = knee_proxy_step(t0_abs, FSM(1), Sim.kneeProxyState, kneeTpl, kneeParams);
            knee_t(ii) = kneeOut.t;
            knee_tau4(ii) = kneeOut.tau4;
            knee_I4(ii) = kneeOut.I4;
            
            if isfield(Sim.kneeProxyState, 'Tst') && isfinite(Sim.kneeProxyState.Tst)
                Tst_log(ii) = Sim.kneeProxyState.Tst;
            end
            if isfield(Sim.kneeProxyState, 'Tsw') && isfinite(Sim.kneeProxyState.Tsw)
                Tsw_log(ii) = Sim.kneeProxyState.Tsw;
            end

            XtQp = Xt;
            UtQp = Ut;
            [H, g, Aineq, bineq, Aeq, beq] = fcn_get_QP_form_eta(XtQp, UtQp, Xd, Ud, p);
            
            fail_h_rcond = rcond(H);
            fail_Aineq_rows = size(Aineq,1);
            fail_Aineq_cols = size(Aineq,2);
            fail_Aeq_rows = size(Aeq,1);
            fail_Aeq_cols = size(Aeq,2);

            [zval, ~, exitflag, qpOutput, lambda] = quadprog( ...
                H, g, Aineq, bineq, Aeq, beq, [], [], [], qp_options);

            qp_exitflag(ii) = exitflag;
            qpSolveCount = qpSolveCount + 1;
            lastQpOutput = qpOutput;
            lastLambda = lambda;

            if exitflag <= 0 || isempty(zval)
                feasible = false;
                fail_reason = 'quadprog';
                qpFailedCount = qpFailedCount + 1;
                fail_iter = ii;
                fail_time_s = t0_abs;
                fail_state_norm = norm(Xt);
                fail_input_norm = norm(Ut);
                fail_com_speed = norm(Xt(4:5));
                fail_fsm_leg1 = FSM(1);
                fail_fsm_all = FSM(:).';
                failureQP = localBuildFailureQp(XtQp, UtQp, Xt, Xd, Ud, FSM, p, ...
                    H, g, Aineq, bineq, Aeq, beq, exitflag, qpOutput, lambda, ...
                    fail_reason, fail_iter, fail_time_s, v_cmd, a_cmd, ...
                    lastSuccessfulXt, lastSuccessfulUt, lastSuccessfulXd, ...
                    lastSuccessfulUd, lastSuccessfulFSM, diagnosticContext);
                break
            end

            [activeTolerance, nearActiveTolerance] = localDiagnosticTolerances(cfg);
            qpDiagnostics = compute_qp_diagnostics(zval, Aineq, bineq, Aeq, beq, ...
                activeTolerance, nearActiveTolerance);
            inequalityMarginMin = min(inequalityMarginMin, ...
                qpDiagnostics.inequality_margin_min);
            activeInequalityCount = activeInequalityCount + ...
                qpDiagnostics.active_inequality_count;
            nearActiveInequalityCount = nearActiveInequalityCount + ...
                qpDiagnostics.near_active_inequality_count;
            violatedInequalityCount = violatedInequalityCount + ...
                qpDiagnostics.violated_inequality_count;
            equalityResidualNormMax = max(equalityResidualNormMax, ...
                qpDiagnostics.equality_residual_norm);
            equalityResidualMaxAbs = max(equalityResidualMaxAbs, ...
                qpDiagnostics.equality_residual_max_abs);
            minimumMarginByLegType = min(minimumMarginByLegType, ...
                qpDiagnostics.minimum_margin_by_leg_type);

            Ut = Ut + zval(1:12);

            [u_ext, p_ext] = fcn_get_disturbance(t0_abs, p);
            p.p_ext = p_ext;
            u_ext = 0 * u_ext;

            [~, X_chunk] = ode45(@(t, X) dynamics_SRB(t, X, Ut, Xd, u_ext, p), [t0_abs, t0_abs + dt_sim], Xt);
            Xt = X_chunk(end,:).';

            if any(isnan(Xt)) || any(isinf(Xt))
                feasible = false;
                fail_reason = 'state_invalid';
                fail_iter = ii;
                fail_time_s = t0_abs + dt_sim;
                fail_state_norm = norm(Xt);
                fail_input_norm = norm(Ut);
                fail_com_speed = norm(Xt(4:5));
                fail_fsm_leg1 = FSM(1);
                fail_fsm_all = FSM(:).';
                failureQP = localBuildFailureQp(XtQp, UtQp, Xt, Xd, Ud, FSM, p, ...
                    H, g, Aineq, bineq, Aeq, beq, exitflag, qpOutput, lambda, ...
                    fail_reason, fail_iter, fail_time_s, v_cmd, a_cmd, ...
                    lastSuccessfulXt, lastSuccessfulUt, lastSuccessfulXd, ...
                    lastSuccessfulUd, lastSuccessfulFSM, diagnosticContext);
                break
            end

            tracking_error(ii) = sum((Xt - Xd(:,1)).^2);
            control_effort(ii) = sum(Ut.^2);

            state_norm(ii) = norm(Xt);
            input_norm(ii) = norm(Ut);
            com_speed(ii) = norm(Xt(4:5));
            lastSuccessfulXt = Xt;
            lastSuccessfulUt = Ut;
            lastSuccessfulXd = Xd;
            lastSuccessfulUd = Ud;
            lastSuccessfulFSM = FSM;
        end
    catch ME
        feasible = false;
        fail_reason = ME.identifier;
        if exist('ii', 'var')
            fail_iter = ii;
        end
        if exist('t0_abs', 'var')
            fail_time_s = t0_abs;
        end
        if exist('FSM', 'var')
            fail_fsm_all = FSM(:).';
            fail_fsm_leg1 = FSM(1);
        end
        caughtExitflag = NaN;
        if exist('exitflag', 'var')
            caughtExitflag = exitflag;
        end
        if exist('H', 'var') && exist('Aineq', 'var') && exist('Aeq', 'var') && ...
                exist('XtQp', 'var') && exist('UtQp', 'var') && ...
                exist('Xd', 'var') && exist('Ud', 'var') && exist('FSM', 'var')
            failureQP = localBuildFailureQp(XtQp, UtQp, Xt, Xd, Ud, FSM, p, ...
                H, g, Aineq, bineq, Aeq, beq, ...
                caughtExitflag, lastQpOutput, lastLambda, ...
                fail_reason, fail_iter, fail_time_s, v_cmd, a_cmd, ...
                lastSuccessfulXt, lastSuccessfulUt, lastSuccessfulXd, ...
                lastSuccessfulUd, lastSuccessfulFSM, diagnosticContext);
            failureQP.exception = ME;
        end
    end

    if ~feasible && ~isempty(fieldnames(failureQP))
        [failureQpId, failureQpFile] = ...
            save_qp_failure_snapshot(cfg, diagnosticContext, failureQP);
    end

    out = struct();

    if isinf(inequalityMarginMin)
        inequalityMarginMin = NaN;
    end
    minimumMarginByLegType(isinf(minimumMarginByLegType)) = NaN;
    lastFsmRow = find(all(isfinite(fsm_all), 2), 1, 'last');
    if isempty(lastFsmRow)
        fsmAllEnd = nan(1, 4);
    else
        fsmAllEnd = fsm_all(lastFsmRow, :);
    end
    stateReference = lastSuccessfulXd;
    if ~feasible && isfield(failureQP, 'Xd')
        stateReference = failureQP.Xd;
    end
    stateComponents = localStateComponents(Xt, stateReference);
    out.failure_qp_id = failureQpId;
    out.failure_qp_file = failureQpFile;
    out.qp_solve_count = qpSolveCount;
    out.qp_failed_count = qpFailedCount;
    out.inequality_margin_min = inequalityMarginMin;
    out.active_inequality_count = activeInequalityCount;
    out.near_active_inequality_count = nearActiveInequalityCount;
    out.violated_inequality_count = violatedInequalityCount;
    out.equality_residual_norm_max = equalityResidualNormMax;
    out.equality_residual_max_abs = equalityResidualMaxAbs;
    out.minimum_margin_by_leg_type = minimumMarginByLegType;
    out.fail_fsm_all = fail_fsm_all;
    out.fsm_all_end = fsmAllEnd;
    out.contact_all_end = fsmAllEnd == 1;
    out.Tst_commanded = p.Tst;
    out.Tsw_commanded = p.Tsw;
    out.state_components = stateComponents;

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
        out.qp_exitflag_last = localLastFinite(qp_exitflag);
    
        out.state_norm_end = norm(Xt);
        out.input_norm_end = norm(Ut);
        out.com_speed_end = norm(Xt(4:5));
        out.fsm_leg1_end = fsmAllEnd(1);
        
        lastTst = find(isfinite(Tst_log), 1, 'last');
        lastTsw = find(isfinite(Tsw_log), 1, 'last');
        
        if isempty(lastTst)
            out.Tst_end = NaN;
        else
            out.Tst_end = Tst_log(lastTst);
        end
        
        if isempty(lastTsw)
            out.Tsw_end = NaN;
        else
            out.Tsw_end = Tsw_log(lastTsw);
        end

        out.x_end = Xt(1);
        out.dx_forward = Xt(1) - x_start;
        
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
    out.qp_exitflag_last = localLastFinite(qp_exitflag);
    
    out.state_norm_end = norm(Xt);
    out.input_norm_end = norm(Ut);
    out.com_speed_end = norm(Xt(4:5));
    out.fsm_leg1_end = fsmAllEnd(1);
    
    lastTst = find(isfinite(Tst_log), 1, 'last');
    lastTsw = find(isfinite(Tsw_log), 1, 'last');
    
    if isempty(lastTst)
        out.Tst_end = NaN;
    else
        out.Tst_end = Tst_log(lastTst);
    end
    
    if isempty(lastTsw)
        out.Tsw_end = NaN;
    else
        out.Tsw_end = Tsw_log(lastTsw);
    end

    out.x_end = Xt(1);
    out.dx_forward = Xt(1) - x_start;
end

function failureQP = localBuildFailureQp( ...
        XtQp, UtQp, propagatedXt, Xd, Ud, FSM, p, H, g, Aineq, bineq, ...
        Aeq, beq, exitflag, qpOutput, lambda, failReason, failIter, ...
        failTime, vCmd, aCmd, lastXt, lastUt, lastXd, lastUd, lastFSM, context)
    failureQP = struct();
    failureQP.fail_reason = failReason;
    failureQP.fail_iter = failIter;
    failureQP.fail_time_s = failTime;
    failureQP.exitflag = exitflag;
    failureQP.quadprog_output = qpOutput;
    failureQP.lambda = lambda;
    failureQP.H = H;
    failureQP.g = g;
    failureQP.Aineq = Aineq;
    failureQP.bineq = bineq;
    failureQP.Aeq = Aeq;
    failureQP.beq = beq;
    failureQP.Xt = XtQp;
    failureQP.Ut = UtQp;
    failureQP.propagated_Xt = propagatedXt;
    failureQP.Xd = Xd;
    failureQP.Ud = Ud;
    failureQP.FSM = FSM;
    failureQP.contact_state = FSM == 1;
    failureQP.v_cmd = vCmd;
    failureQP.a_cmd = aCmd;
    failureQP.R = p.R;
    failureQP.Q = p.Q;
    failureQP.Qf = p.Qf;
    failureQP.mu = p.mu;
    failureQP.prediction_horizon = p.predHorizon;
    failureQP.Tmpc = p.Tmpc;
    failureQP.Tst_commanded = p.Tst;
    failureQP.Tst_realized = min(p.Tst, 0.2 / norm(XtQp(4:5)));
    failureQP.Tsw_commanded = p.Tsw;
    failureQP.last_successful_Xt = lastXt;
    failureQP.last_successful_Ut = lastUt;
    failureQP.last_successful_Xd = lastXd;
    failureQP.last_successful_Ud = lastUd;
    failureQP.last_successful_FSM = lastFSM;
    failureQP.context = context;
    try
        [failureQP.A, failureQP.B, failureQP.d] = ...
            fcn_get_ABD_eta(XtQp, UtQp, p);
    catch
        failureQP.A = [];
        failureQP.B = [];
        failureQP.d = [];
    end
    failureQP.rank_Aeq = rank(Aeq);
    failureQP.rcond_H = rcond(H);
    try
        failureQP.condest_H = condest(sparse(H));
    catch
        failureQP.condest_H = NaN;
    end
    symmetricH = (H + H.') / 2;
    hEigenvalues = eig(symmetricH);
    failureQP.H_eigenvalues = hEigenvalues;
    failureQP.H_min_eigenvalue = min(hEigenvalues);
    failureQP.H_max_eigenvalue = max(hEigenvalues);
    failureQP.H_negative_eigenvalue_count = nnz(hEigenvalues < -1e-10);
    failureQP.H_near_zero_eigenvalue_count = nnz(abs(hEigenvalues) <= 1e-10);
    failureQP.state_components = localStateComponents(XtQp, Xd);
end

function [activeTolerance, nearActiveTolerance] = localDiagnosticTolerances(cfg)
    activeTolerance = 1e-7;
    nearActiveTolerance = 1e-5;
    if isfield(cfg, 'DIAGNOSTICS')
        if isfield(cfg.DIAGNOSTICS, 'active_margin_tolerance')
            activeTolerance = cfg.DIAGNOSTICS.active_margin_tolerance;
        end
        if isfield(cfg.DIAGNOSTICS, 'near_active_margin_tolerance')
            nearActiveTolerance = cfg.DIAGNOSTICS.near_active_margin_tolerance;
        end
    end
end

function components = localStateComponents(Xt, Xd)
    components = struct();
    try
        if isempty(Xd)
            components = decompose_srb_state(Xt);
        else
            components = decompose_srb_state(Xt, Xd(:, 1));
        end
    catch
        components.full_state_norm = NaN;
        components.global_position_norm = NaN;
        components.global_position_x = NaN;
        components.global_position_y = NaN;
        components.global_position_z = NaN;
        components.linear_velocity_norm = NaN;
        components.orientation_error_rad = NaN;
        components.orientation_error_vector_norm = NaN;
        components.rotation_orthogonality_error = NaN;
        components.angular_velocity_norm = NaN;
        components.foot_position_global_norm = NaN;
        components.foot_position_relative_norm = NaN;
        components.position_invariant_state_norm = NaN;
    end
end

function value = localLastFinite(values)
    idx = find(isfinite(values), 1, 'last');
    if isempty(idx)
        value = NaN;
    else
        value = values(idx);
    end
end
