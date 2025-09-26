% function [tracking_error_total, control_effort_total] = run_MPC_simulation(R_weights, gait)
% 
%     if nargin < 2 || isempty(gait), gait = 0; end
%     p = get_params(gait);
%     if nargin < 1 || isempty(R_weights)
%         R_weights = diag(p.R);
%     end
%     p.R = diag(R_weights);
% 
%     dt_sim = p.simTimeStep;
%     SimTimeDuration = 1;
%     MAX_ITER = floor(SimTimeDuration / dt_sim);
% 
%     snap_path = 'SimSnapshot_RL.mat';
% 
%     % --- Init or resume boundary state ---
%     if exist(snap_path, 'file')
%         S = load(snap_path, 'Sim');
%         Sim = S.Sim;
%         Xt = Sim.Xt; % plant state at last run's boundary
%         Ut = Sim.Ut; % last control
%         t_abs = Sim.t; % absolute time at chunk start
%     else
%         if gait == 1
%             [p, Xt, Ut] = fcn_bound_ref_traj(p);
%         else
%             [Xt, Ut] = fcn_gen_XdUd(0, [], [1;1;1;1], p);
%         end
%         Sim = struct('t', 0.0, 'Xt', Xt, 'Ut', Ut);
%         t_abs = Sim.t;
%     end
% 
%     % Quadprog options
%     qp_options = optimoptions('quadprog', ...
%         'Display', 'off', ...
%         'ConstraintTolerance', 1e-5, ...
%         'OptimalityTolerance', 1e-5, ...
%         'MaxIterations', 1000, ...
%         'StepTolerance', 1e-8);
% 
%     tracking_error = [];
%     control_effort = [];
%     disp('t:')
%     disp(t_abs)
% 
%     try
%         for ii = 1:MAX_ITER
%             % Absolute horizon time stamps (continuous across episodes)
%             t0_abs = t_abs + dt_sim * (ii-1);
%             t_hor  = t0_abs + p.Tmpc * (0:p.predHorizon-1);
% 
%             % FSM / refs from absolute time
%             if gait == 1
%                 [FSM, Xd, Ud, Xt] = fcn_FSM_bound(t_hor, Xt, p);
%             else
%                 [FSM, Xd, Ud, Xt] = fcn_FSM(t_hor, Xt, p);
%             end
% 
%             % Build QP from scratch for current boundary state & refs
%             [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta(Xt, Ut, Xd, Ud, p);
% 
%             [zval, ~, exitflag, output] = quadprog(H, g, Aineq, bineq, Aeq, beq, [], [], [], qp_options);
%             if exitflag <= 0
%                 disp("=== QUADPROG FAILURE ===");
%                 fprintf("Reason: %s\n", output.message);
%                 tracking_error_total  = 1e3;
%                 control_effort_total  = 7.5e7;
%                 % do not advance snapshot if a failure occurs
%                 save(snap_path, 'Sim');
%                 return;
%             end
% 
%             Ut = Ut + zval(1:12);
% 
%             [u_ext, p_ext] = fcn_get_disturbance(t0_abs, p);
%             p.p_ext = p_ext;
%             u_ext = 0 * u_ext;
% 
%             [~, X] = ode45(@(t,X) dynamics_SRB(t, X, Ut, Xd(:,1), u_ext, p), ...
%                            [t0_abs, t0_abs + dt_sim], Xt);
% 
%             Xt = X(end,:)';
% 
%             if any(isnan(Xt)) || any(isinf(Xt))
%                 disp("=== DYNAMICS FAILURE ===");
%                 tracking_error_total = 1e6;
%                 control_effort_total = 1e6;
%                 save(snap_path, 'Sim');  % keep previous Sim (don’t advance)
%                 return;
%             end
% 
%             tracking_error = [tracking_error; sum((Xt - Xd(:,1)).^2)];
%             control_effort = [control_effort; sum(Ut.^2)];
%         end
% 
%         Sim.t  = t_abs + MAX_ITER * dt_sim; % increment absolute time
%         Sim.Xt = Xt; % keep terminal state
%         Sim.Ut = Ut; % keep last control
%         save(snap_path, 'Sim');
% 
%         tracking_error_total = sum(tracking_error);
%         control_effort_total = sum(control_effort);
% 
%         % fprintf('Tracking error: %.6g | Control effort: %.6g | t_abs -> %.3f\n', ...
%         %     tracking_error_total, control_effort_total, Sim.t);
% 
%     catch ME
%         disp("=== EXCEPTION CAUGHT ===");
%         fprintf("Error: %s\n", ME.message);
%         save(snap_path, 'Sim');
%         tracking_error_total = 1e3;
%         control_effort_total = 1e7;
%     end
% end


function [tracking_error_total, control_effort_total] = run_MPC_simulation(R_weights, gait)
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
        S   = load(snap_path, 'Sim');
        Sim = S.Sim;
        Xt  = Sim.Xt; % plant state at last boundary
        Ut  = Sim.Ut; % last control
        t_abs = Sim.t; % absolute time at start
    else
        if gait == 1
            [p, Xt, Ut] = fcn_bound_ref_traj(p);
        else
            [Xt, Ut] = fcn_gen_XdUd(0, [], [1;1;1;1], p);
        end
        Sim = struct('t', 0.0, 'Xt', Xt, 'Ut', Ut);
        t_abs = Sim.t;
    end

    qp_options = optimoptions('quadprog', ...
        'Display', 'off', ...
        'ConstraintTolerance', 1e-5, ...
        'OptimalityTolerance', 1e-5, ...
        'MaxIterations', 1000, ...
        'StepTolerance', 1e-8);

    tracking_error = [];
    control_effort = [];

    try
        for ii = 1:MAX_ITER
            % Absolute time stamps across the horizon
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
                save(snap_path, 'Sim');  % keep previous state
                return;
            end

            % Apply first control correction (delta formulation)
            Ut = Ut + zval(1:12);

            % External disturbance (disabled here)
            [u_ext, p_ext] = fcn_get_disturbance(t0_abs, p);
            p.p_ext = p_ext;
            u_ext   = 0 * u_ext;

            % Integrate dynamics over one step
            [~, X] = ode45(@(t,X) dynamics_SRB(t, X, Ut, Xd(:,1), u_ext, p), ...
                           [t0_abs, t0_abs + dt_sim], Xt);

            Xt = X(end,:).';

            if any(isnan(Xt)) || any(isinf(Xt))
                warning("dynamics produced NaN/Inf");
                tracking_error_total = 1e6;
                control_effort_total = 1e6;
                save(snap_path, 'Sim');  % keep previous
                return;
            end

            tracking_error = [tracking_error; sum((Xt - Xd(:,1)).^2)];
            control_effort = [control_effort; sum(Ut.^2)];
        end

        % Advance snapshot
        Sim.t  = t_abs + MAX_ITER * dt_sim;
        Sim.Xt = Xt;
        Sim.Ut = Ut;
        save(snap_path, 'Sim');

        tracking_error_total = sum(tracking_error);
        control_effort_total = sum(control_effort);

    catch ME
        fprintf("run_MPC_simulation exception: %s", ME.message);
        save(snap_path, 'Sim');
        tracking_error_total = 1e3;
        control_effort_total = 1e7;
    end
end
