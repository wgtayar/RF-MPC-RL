function [tracking_error_total, control_effort_total] = run_MPC_simulation(R_weights, gait)
    if nargin < 1 || isempty(R_weights)
        gait = 0;
        p = get_params(gait);
        R_weights = diag(p.R);
    end
    if nargin < 2
        gait = 0;
    end

    % Get params and replace R
    p = get_params(gait);
    p.R = diag(R_weights);

    disp("=== Start of MPC Simulation ===");
    disp("R_weights:");
    disp(R_weights);

    dt_sim = p.simTimeStep;
    SimTimeDuration = 1;
    MAX_ITER = floor(SimTimeDuration / dt_sim);

    if gait == 1
        [p, Xt, Ut] = fcn_bound_ref_traj(p);
    else
        [Xt, Ut] = fcn_gen_XdUd(0, [], [1;1;1;1], p);
    end

    tracking_error = [];
    control_effort = [];

    tstart = 0;
    tend = dt_sim;

    % Setup quadprog options
    qp_options = optimoptions('quadprog', ...
        'Display', 'off', ...
        'ConstraintTolerance', 1e-5, ...
        'OptimalityTolerance', 1e-5, ...
        'MaxIterations', 1000, ...
        'StepTolerance', 1e-8);

    try
        for ii = 1:MAX_ITER
            fprintf("Iteration %d/%d\n", ii, MAX_ITER);

            t_ = dt_sim * (ii-1) + p.Tmpc * (0:p.predHorizon-1);
            if gait == 1
                [FSM, Xd, Ud, Xt] = fcn_FSM_bound(t_, Xt, p);
            else
                [FSM, Xd, Ud, Xt] = fcn_FSM(t_, Xt, p);
            end

            [H, g, Aineq, bineq, Aeq, beq] = fcn_get_QP_form_eta(Xt, Ut, Xd, Ud, p);

            % Solve QP
            [zval, ~, exitflag, output] = quadprog(H, g, Aineq, bineq, Aeq, beq, [], [], [], qp_options);

            if exitflag <= 0
                disp("=== QUADPROG FAILURE ===");
                fprintf("Reason: %s\n", output.message);
                % Penalize failure proportional to progress
                progress = -MAX_ITER / ii;
                tracking_error_total = 1e3;
                control_effort_total = 7.5e7;
                return;
            end

            Ut = Ut + zval(1:12);

            [u_ext, p_ext] = fcn_get_disturbance(tstart, p);
            p.p_ext = p_ext;
            u_ext = 0*u_ext;

            [t, X] = ode45(@(t,X)dynamics_SRB(t, X, Ut, Xd(:,1), u_ext, p), [tstart,tend], Xt);

            Xt = X(end,:)';

            % Check for NaNs or Infs
            if any(isnan(Xt)) || any(isinf(Xt))
                disp("=== DYNAMICS FAILURE ===");
                progress = ii / MAX_ITER;
                tracking_error_total = 1e6;
                control_effort_total = 1e6;
                return;
            end

            tracking_error = [tracking_error; sum((Xt - Xd(:,1)).^2)];
            control_effort = [control_effort; sum(Ut.^2)];
            progress = 0.1;

            tstart = tend;
            tend = tstart + dt_sim;
        end

        % Return total cost and progress
        tracking_error_total = sum(tracking_error);
        control_effort_total = sum(control_effort);
        progress = 1.0; % Successfully completed
        disp('Tracking error for this iteration:')
        disp(tracking_error_total)
        disp('Control Effort for this iteration:')
        disp(control_effort_total)
        disp('Progress for this iteration:')
        disp(progress)

    catch ME
        disp("=== EXCEPTION CAUGHT ===");
        fprintf("Error: %s\n", ME.message);
        progress = ii / MAX_ITER;
        tracking_error_total = 1e3;
        control_effort_total = 1e7;
    end
end
