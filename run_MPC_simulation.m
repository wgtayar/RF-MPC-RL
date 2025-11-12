function [tracking_error_total, control_effort_total, I_total] = run_MPC_simulation(R_weights, gait)
    if nargin < 2 || isempty(gait), gait = 0; end
    p = get_params(gait);

    if nargin < 1 || isempty(R_weights)
        R_weights = diag(p.R);
    end
    p.R = diag(R_weights);

    dt_sim = p.simTimeStep;
    SimTimeDuration = 1; % chunk duration [s]
    MAX_ITER = floor(SimTimeDuration / dt_sim);

    snap_path = 'SimSnapshot_RL.mat';

    % --- Init or resume boundary state ---
    if exist(snap_path, 'file')
        S = load(snap_path, 'Sim');
        Sim = S.Sim;
        Xt = Sim.Xt; % plant state at last boundary
        Ut = Sim.Ut; % last control
        t_abs = Sim.t; % absolute time at start
        
        if isfield(Sim,'t_power')
            t_power = Sim.t_power; 
        else
            t_power = []; 
        end
        
        if isfield(Sim,'Pc_all')
            Pc_all = Sim.Pc_all;
        else
            Pc_all = [];
        end
        
        if isfield(Sim,'I_all')
            I_all = Sim.I_all;
        else
            I_all = []; 
        end
    else
        if gait == 1
            [p, Xt, Ut] = fcn_bound_ref_traj(p);
        else
            [Xt, Ut] = fcn_gen_XdUd(0, [], [1;1;1;1], p);
        end

        Sim = struct();
        Sim.t = 0.0;
        Sim.Xt = Xt;
        Sim.Ut = Ut;
        
        t_abs = Sim.t;
        t_power = [];
        Pc_all = [];
        I_all = [];
    end

    qp_options = optimoptions('quadprog', ...
        'Display', 'off', ...
        'ConstraintTolerance', 1e-5, ...
        'OptimalityTolerance', 1e-5, ...
        'MaxIterations', 1000, ...
        'StepTolerance', 1e-8);

    tracking_error = [];
    control_effort = [];

    tout = [];
    Xout = [];
    Uout = [];

    try
        for ii = 1:MAX_ITER
            
            t0_abs = t_abs + dt_sim * (ii-1);
            t_hor  = t0_abs + p.Tmpc * (0:p.predHorizon-1);

            % Reference / FSM from absolute time
            if gait == 1
                [FSM, Xd, Ud, Xt] = fcn_FSM_bound(t_hor, Xt, p);
            else
                [FSM, Xd, Ud, Xt] = fcn_FSM(t_hor, Xt, p);
            end

            % Build QP and solve
            [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta(Xt, Ut, Xd, Ud, p);
            [zval, ~, exitflag, output] = quadprog(H, g, Aineq, bineq, Aeq, beq, [], [], [], qp_options);
            if exitflag <= 0
                % Penalize failures but keep snapshot consistent
                warning("quadprog failed: %s", output.message);
                tracking_error_total  = 1e3;
                control_effort_total  = 7.5e7;
                Sim.Pc_all = Pc_all;
                Sim.I_all = I_all;
                save(snap_path, 'Sim');  % keep previous state
                return;
            end

            % Apply first control correction (delta formulation)
            Ut = Ut + zval(1:12);

            % External disturbance (disabled here)
            [u_ext, p_ext] = fcn_get_disturbance(t0_abs, p);
            p.p_ext = p_ext;
            u_ext = 0 * u_ext;

            % Simulate dynamics over one step
            [t_chunk, X_chunk] = ode45(@(t,X) dynamics_SRB(t, X, Ut, Xd(:,1), u_ext, p), ...
                           [t0_abs, t0_abs + dt_sim], Xt);

            Xt = X_chunk(end,:).';

            if any(isnan(Xt)) || any(isinf(Xt))
                warning("dynamics produced NaN/Inf");
                tracking_error_total = 1e6;
                control_effort_total = 1e6;
                Sim.t_power = Pc_all;
                Sim.I_all = I_all;
                save(snap_path, 'Sim');  % keep previous
                return;
            end

            tracking_error = [tracking_error; sum((Xt - Xd(:,1)).^2)];
            control_effort = [control_effort; sum(Ut.^2)];

            if ii == 1
                idx = 1:length(t_chunk);
            else
                idx = 2:length(t_chunk);
            end

            tout = [tout; t_chunk(idx)];
            Xout = [Xout; X_chunk(idx,:)];
            Uout = [Uout; repmat(Ut.', numel(idx), 1)];
        end

        % Advance snapshot
        Sim.t  = t_abs + MAX_ITER * dt_sim;
        Sim.Xt = Xt;
        Sim.Ut = Ut;

        if ~isempty(tout)
            [tpc_chunk, Pc_chunk] = power_calc(tout, Xout, Uout, struct('BI', p.J));

            t_power = [t_power; tpc_chunk];
            Pc_all = [Pc_all; Pc_chunk];

            Iopts = struct();
            Iopts.I0 = 1.0;
            Iopts.eta = 1.4;
            Iopts.clipNeg = false;
            Iopts.smoothWin = 0.0;

            stats_pred = Ibus_pred(t_power, Pc_all, 20, Iopts);
            I_all = stats_pred.Ipred(:);
        end

        Sim.t_power = t_power;
        Sim.Pc_all = Pc_all;
        Sim.I_all = I_all;

        save("m_i_values.mat", "I_all")

        I_total = sum(I_all);
        tracking_error_total = sum(tracking_error);
        control_effort_total = sum(control_effort);

        % disp('I total:')
        % disp(I_total)
        % disp('Tracking error total:')
        % disp(tracking_error_total)
        % disp('Control effort total:')
        % disp(control_effort_total)

        save(snap_path, 'Sim');

    catch ME
        fprintf("run_MPC_simulation exception: %s", ME.message);
        Sim.t_power = t_power;
        Sim.Pc_all = Pc_all;
        Sim.I_all = I_all;
        save(snap_path, 'Sim');
        tracking_error_total = 1e3;
        control_effort_total = 1e7;
    end
end
