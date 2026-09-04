function [state, out] = simulate_mpc_horizon(state, control, cfg, options)
%simulate_mpc_horizon Continue RF-MPC from an explicit, restorable state.

    if nargin < 4
        options = struct();
    end
    options = localDefaults(options, cfg);
    if state.t == 0 && isempty(fieldnames(state.fsm_internal_state))
        reset_mpc_case_state();
    end
    p = get_params(state.gait);
    rWeights = control.R(:);
    if numel(rWeights) == 3
        rWeights = repmat(rWeights, 4, 1);
    end
    p.R = diag(rWeights);
    p.vel_d = [control.v_cmd; 0];
    p.acc_d = control.a_cmd;
    p.yaw_d = 0;

    dt = p.simTimeStep;
    numberSteps = max(0, round(options.duration_s/dt));
    traceRows = repmat(localEmptyTrace(), numberSteps, 1);
    traceCount = 0;
    failureProblem = struct();
    failureSolver = struct();
    firstProblem = struct();
    terminalReason = "horizon_complete";
    initialState = state;
    initialTime = state.t;
    chargeStart = localTrapzCharge(state.current_time, state.current_total);
    solveCount = 0;
    failedSolveCount = 0;

    for iteration = 1:numberSteps
        time = initialTime + dt*(iteration - 1);
        useOverride = iteration == 1 && ...
            ~isempty(fieldnames(options.initial_problem_override));
        if useOverride
            override = options.initial_problem_override;
            state.Xt = override.Xt;
            state.Ut = override.Ut;
            Xd = override.Xd;
            Ud = override.Ud;
            FSM = override.FSM;
            H = override.H;
            g = override.g;
            Aineq = override.Aineq;
            bineq = override.bineq;
            Aeq = override.Aeq;
            beq = override.beq;
        else
            timeHorizon = time + p.Tmpc*(0:p.predHorizon-1);
            if state.gait == 1
                [FSM, Xd, Ud, state.Xt, state.fsm_internal_state] = ...
                    fcn_FSM_bound(timeHorizon, state.Xt, p, ...
                    state.fsm_internal_state);
            else
                [FSM, Xd, Ud, state.Xt, state.fsm_internal_state] = ...
                    fcn_FSM(timeHorizon, state.Xt, p, ...
                    state.fsm_internal_state);
            end
            [H, g, Aineq, bineq, Aeq, beq] = ...
                fcn_get_QP_form_eta(state.Xt, state.Ut, Xd, Ud, p);
        end

        kneeCurrent = NaN;
        if options.update_proxy
            [state.knee_proxy_state, kneeOut] = knee_proxy_step( ...
                time, FSM(1), state.knee_proxy_state, ...
                state.knee_template, state.knee_parameters);
            kneeCurrent = kneeOut.I4 + state.hip_current_A;
        end

        XtQp = state.Xt;
        UtQp = state.Ut;
        problem = struct('H', H, 'g', g, 'Aineq', Aineq, ...
            'bineq', bineq, 'Aeq', Aeq, 'beq', beq);
        if iteration == 1 && options.capture_first_problem
            firstProblem = problem;
            firstProblem.Xt = XtQp;
            firstProblem.Ut = UtQp;
            firstProblem.Xd = Xd;
            firstProblem.Ud = Ud;
            firstProblem.FSM = FSM;
            firstProblem.fsm_internal_state = state.fsm_internal_state;
            firstProblem.time_s = time;
        end
        strategy = localStrategy(options, time);
        solverOptions = struct('classify_failure', options.classify_failure);
        solver = solve_mpc_qp(problem, strategy, solverOptions);
        solveCount = solveCount + 1;

        capture = options.capture_trace && time >= options.trace_start_time_s;
        if capture
            traceCount = traceCount + 1;
            traceRows(traceCount) = localTraceBefore( ...
                time, iteration, strategy, solver, XtQp, UtQp, Xd, Ud, ...
                FSM, H, Aeq, kneeCurrent, control);
        end

        if ~solver.success
            failedSolveCount = failedSolveCount + 1;
            terminalReason = solver.classification;
            failureProblem = problem;
            failureProblem.Xt = XtQp;
            failureProblem.Ut = UtQp;
            failureProblem.Xd = Xd;
            failureProblem.Ud = Ud;
            failureProblem.FSM = FSM;
            failureProblem.fsm_internal_state = state.fsm_internal_state;
            failureProblem.time_s = time;
            failureSolver = solver;
            break
        end

        state.Ut = state.Ut + solver.z(1:12);
        [uExt, pExt] = fcn_get_disturbance(time, p);
        p.p_ext = pExt;
        uExt = 0*uExt;
        [~, stateHistory] = ode45( ...
            @(t, X) dynamics_SRB(t, X, state.Ut, Xd, uExt, p), ...
            [time, time + dt], state.Xt);
        state.Xt = stateHistory(end, :).';
        state.t = initialTime + dt*iteration;

        if capture
            traceRows(traceCount) = localTraceAfter( ...
                traceRows(traceCount), state.Xt, state.Ut, Xd);
        end
        if any(~isfinite(state.Xt))
            terminalReason = "invalid_state";
            break
        end

        if options.update_battery && isfinite(kneeCurrent)
            state.current_time(end+1, 1) = time;
            state.current_total(end+1, 1) = kneeCurrent;
        end
    end

    if options.update_battery && numel(state.current_time) >= 2
        state.battery = evaluate_battery_feedback( ...
            state.current_time, state.current_total, cfg.BATTERY, ...
            initialState.battery);
    end
    state.R = control.R(1:3);
    if isfield(control, 'action')
        state.previous_action = control.action(:);
    end

    traceRows = traceRows(1:traceCount);
    if isempty(traceRows)
        trace = struct2table(repmat(localEmptyTrace(), 0, 1));
    else
        trace = struct2table(traceRows);
    end
    out = struct();
    out.trace = trace;
    out.terminal_reason = terminalReason;
    out.completed_horizon = terminalReason == "horizon_complete";
    out.requested_duration_s = options.duration_s;
    out.survived_duration_s = state.t - initialState.t;
    out.qp_solve_count = solveCount;
    out.qp_failed_count = failedSolveCount;
    out.failure_problem = failureProblem;
    out.failure_solver = failureSolver;
    out.first_problem = firstProblem;
    out.initial_state = localCompactState(initialState);
    out.final_state = localCompactState(state);
    out.charge_As = localTrapzCharge( ...
        state.current_time, state.current_total) - chargeStart;
    out.Ieq_A = out.charge_As/max(out.survived_duration_s, eps);
end

function options = localDefaults(options, cfg)
    options = localSetDefault(options, 'duration_s', cfg.CHUNK_DURATION);
    options = localSetDefault(options, 'solver_strategy', "default");
    options = localSetDefault(options, 'switch_time_s', inf);
    options = localSetDefault(options, 'capture_trace', false);
    options = localSetDefault(options, 'trace_start_time_s', -inf);
    options = localSetDefault(options, 'classify_failure', true);
    options = localSetDefault(options, 'capture_first_problem', false);
    options = localSetDefault(options, 'initial_problem_override', struct());
    options = localSetDefault(options, 'update_proxy', true);
    options = localSetDefault(options, 'update_battery', false);
end

function value = localSetDefault(value, field, defaultValue)
    if ~isfield(value, field)
        value.(field) = defaultValue;
    end
end

function strategy = localStrategy(options, time)
    strategy = string(options.solver_strategy);
    if strategy == "default_then_active_set"
        if time >= options.switch_time_s - 10*eps(max(abs(time), 1))
            strategy = "active_set_feasible_point";
        else
            strategy = "default";
        end
    end
end

function row = localEmptyTrace()
    row = struct( ...
        'time_s', NaN, 'iteration', NaN, 'solver_strategy', "", ...
        'solver_classification', "", 'solver_exitflag', NaN, ...
        'solver_iterations', NaN, 'solver_wall_time_s', NaN, ...
        'objective', NaN, 'first_order_opt', NaN, ...
        'constraint_violation', NaN, 'kkt_stationarity_inf', NaN, ...
        'kkt_complementarity_inf', NaN, 'rcond_H', NaN, ...
        'condest_H', NaN, 'rank_Aeq', NaN, ...
        'inequality_margin_min', NaN, 'equality_residual_max_abs', NaN, ...
        'active_inequality_count', NaN, 'near_active_inequality_count', NaN, ...
        'fsm_leg1', NaN, 'fsm_leg2', NaN, 'fsm_leg3', NaN, 'fsm_leg4', NaN, ...
        'orientation_error_before_rad', NaN, 'angular_velocity_before', NaN, ...
        'linear_velocity_error_before', NaN, 'com_velocity_before', NaN, ...
        'position_invariant_norm_before', NaN, 'Ut_norm_before', NaN, ...
        'orientation_error_after_rad', NaN, 'angular_velocity_after', NaN, ...
        'linear_velocity_error_after', NaN, 'com_velocity_after', NaN, ...
        'position_invariant_norm_after', NaN, 'Ut_norm_after', NaN, ...
        'force_correction_norm', NaN, 'desired_force_norm', NaN, ...
        'knee_plus_hip_current_A', NaN, ...
        'dR1', NaN, 'dR2', NaN, 'dR3', NaN, ...
        'gamma_v', NaN, 'gamma_a', NaN, ...
        'R1', NaN, 'R2', NaN, 'R3', NaN, ...
        'v_cmd', NaN, 'a_cmd', NaN);
end

function row = localTraceBefore(time, iteration, strategy, solver, ...
        Xt, Ut, Xd, Ud, FSM, H, Aeq, current, control)
    row = localEmptyTrace();
    row.time_s = time;
    row.iteration = iteration;
    row.solver_strategy = strategy;
    row.solver_classification = solver.classification;
    row.solver_exitflag = solver.exitflag;
    row.solver_iterations = solver.iterations;
    row.solver_wall_time_s = solver.wall_time_s;
    row.objective = solver.objective;
    row.first_order_opt = solver.first_order_opt;
    row.constraint_violation = solver.constraint_violation;
    row.kkt_stationarity_inf = solver.kkt_stationarity_inf;
    row.kkt_complementarity_inf = solver.kkt_complementarity_inf;
    row.rcond_H = rcond(H);
    row.condest_H = 1/max(row.rcond_H, realmin);
    row.rank_Aeq = rank(Aeq);
    if solver.success
        row.inequality_margin_min = solver.diagnostics.inequality_margin_min;
        row.equality_residual_max_abs = ...
            solver.diagnostics.equality_residual_max_abs;
        row.active_inequality_count = ...
            solver.diagnostics.active_inequality_count;
        row.near_active_inequality_count = ...
            solver.diagnostics.near_active_inequality_count;
        row.force_correction_norm = norm(solver.z(1:12));
    end
    row.fsm_leg1 = FSM(1);
    row.fsm_leg2 = FSM(2);
    row.fsm_leg3 = FSM(3);
    row.fsm_leg4 = FSM(4);
    before = decompose_srb_state(Xt, Xd(:, 1));
    row.orientation_error_before_rad = before.orientation_error_rad;
    row.angular_velocity_before = before.angular_velocity_norm;
    row.linear_velocity_error_before = norm(Xt(4:6) - Xd(4:6, 1));
    row.com_velocity_before = norm(Xt(4:6));
    row.position_invariant_norm_before = ...
        before.position_invariant_state_norm;
    row.Ut_norm_before = norm(Ut);
    row.desired_force_norm = norm(Ud(:, 1));
    row.knee_plus_hip_current_A = current;
    row.R1 = control.R(1);
    row.R2 = control.R(2);
    row.R3 = control.R(3);
    row.v_cmd = control.v_cmd;
    row.a_cmd = control.a_cmd;
    if isfield(control, 'action') && numel(control.action) >= 5
        row.dR1 = control.action(1);
        row.dR2 = control.action(2);
        row.dR3 = control.action(3);
        row.gamma_v = control.action(4);
        row.gamma_a = control.action(5);
    end
end

function row = localTraceAfter(row, Xt, Ut, Xd)
    after = decompose_srb_state(Xt, Xd(:, 1));
    row.orientation_error_after_rad = after.orientation_error_rad;
    row.angular_velocity_after = after.angular_velocity_norm;
    row.linear_velocity_error_after = norm(Xt(4:6) - Xd(4:6, 1));
    row.com_velocity_after = norm(Xt(4:6));
    row.position_invariant_norm_after = after.position_invariant_state_norm;
    row.Ut_norm_after = norm(Ut);
end

function compact = localCompactState(state)
    compact = struct('time_s', state.t, 'Xt', state.Xt, 'Ut', state.Ut, ...
        'fsm_internal_state', state.fsm_internal_state, ...
        'knee_proxy_state', state.knee_proxy_state, ...
        'battery', state.battery, 'R', state.R, ...
        'previous_action', state.previous_action);
end

function charge = localTrapzCharge(time, current)
    if numel(time) < 2
        charge = 0;
    else
        charge = trapz(time, abs(current));
    end
end
