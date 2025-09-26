% function [nextObs, reward, isDone, loggedSignals] = rlStepFunction(R_weights_unique, loggedSignals)
%     persistent last_R_weights_unique
% 
%     snap_path = 'SimSnapshot_RL.mat';
%     first_run = ~exist(snap_path,'file');
% 
%     if first_run
%         % First call starts from defaults
%         gait = 0;
%         p = get_params(gait);
%         R_weights_unique = [p.R(1,1); p.R(2,2); p.R(3,3)];
%     else
%         if isempty(last_R_weights_unique)
%             last_R_weights_unique = R_weights_unique;
%         else
%             delta = 0.1 * last_R_weights_unique;
%             R_weights_unique = min(max(R_weights_unique, ...
%                                        last_R_weights_unique - delta), ...
%                                        last_R_weights_unique + delta);
%         end
%     end
% 
%     R_weights = repmat(R_weights_unique, [4, 1]);
% 
%     % Run MPC simulation
%     disp('R_weights for MPC simulation:');
%     disp(R_weights');
%     [tracking_error_total, control_effort_total] = run_MPC_simulation(R_weights, 0);
% 
%     % Normalize errors
%     % scaled_tracking_error = log(1 + tracking_error_total);
%     % scaled_control_effort = log(1 + control_effort_total);
% 
%     % For a 5s simulation:
%     % T_ref = 10;
%     % U_ref = 7.5e5;
% 
%     % For a 0.5s simulation:
%     % T_ref = 0.5;
%     % U_ref = 6e4;
% 
%     % For a 1s simulation:
%     T_ref = 1.5;
%     U_ref = 1.5e5;
% 
%     % norm_tracking_error = tracking_error_total / T_ref;
%     % norm_control_effort = control_effort_total / U_ref;
% 
%     norm_tracking_error = log1p(tracking_error_total / T_ref);
%     norm_control_effort = log1p(control_effort_total / U_ref);
% 
%     disp('Tracking error:')
%     disp(tracking_error_total)
% 
%     disp('Control effort error:')
%     disp(control_effort_total)
% 
%     disp('Norm tracking error:')
%     disp(norm_tracking_error)
% 
%     disp('Norm control effort:')
%     disp(norm_control_effort)
% 
%     % Reward function
%     alpha = 10;  % weight for tracking error
%     beta = 1; % weight for control effort
% 
%     % Encourage finishing all iterations and penalize early failures
%     reward = -(alpha * norm_tracking_error + beta * norm_control_effort);
% 
%     % Clip reward to prevent extreme values
%     % reward = max(min(reward, 1e3), -1e3);
% 
%     % Save current weights for next episode
%     last_R_weights_unique = R_weights_unique;
% 
%     % Next observation
%     nextObs = [norm_tracking_error; norm_control_effort];
%     isDone = true;
%     loggedSignals = [];
% end
% 


function [nextObs, reward, isDone, logged] = rlStepFunction(dR_frac, logged)
% v2.1: Lazy-load defaults so validation runs even before cfg is saved.

    % episode counter & cached config
    persistent stepCount cfg lower_abs upper_abs
    if isempty(stepCount); stepCount = 0; end

    % If we don't have cfg/bounds yet (first validation call), build safe defaults.
    if isempty(cfg) || isempty(lower_abs) || isempty(upper_abs)
        gait = 0;
        p = get_params(gait);
        initR = [p.R(1,1); p.R(2,2); p.R(3,3)];
        lower_abs = 0.01 * initR;
        upper_abs = 5.00  * initR;
        cfg.EP_STEPS    = 10;
        cfg.APPLY_EVERY = 10;

        % If a saved config exists, prefer it.
        if exist('rlEnv_MPC_R.mat','file')
            try
                S0 = load('rlEnv_MPC_R.mat');
                if isfield(S0,'cfg');        cfg        = S0.cfg;        end
                if isfield(S0,'lower_abs');  lower_abs  = S0.lower_abs;  end
                if isfield(S0,'upper_abs');  upper_abs  = S0.upper_abs;  end
            catch
                % stay with defaults
            end
        end
    end

    % load/persist weights
    if exist('LastR.mat','file')
        S = load('LastR.mat','last_R');
        last_R = S.last_R;
    else
        % first-ever call: seed from params
        gait = 0; p = get_params(gait);
        last_R = [p.R(1,1); p.R(2,2); p.R(3,3)];
        save('LastR.mat','last_R');
    end

    % apply new action only at episode boundary (held action)
    shouldApply = (mod(stepCount, cfg.APPLY_EVERY) == 0);

    if shouldApply
        % dR_frac ∈ [-0.10, 0.10]
        R_new = last_R .* (1 + dR_frac);
        R_new = min(max(R_new, lower_abs), upper_abs);
    else
        R_new = last_R;
    end

    % run one simulation chunk; snapshot advances
    R_all = repmat(R_new,[4,1]);
    [te, ue] = run_MPC_simulation(R_all, 0);

    % costs & improvement-only reward (no log compress)
    T_ref = 1.5;
    U_ref = 1.5e5;
    nt = te / T_ref;
    nu = ue / U_ref;
    alpha = 0.6;  beta = 0.4;
    J = alpha*nt + beta*nu;

    if ~isfield(logged,'prevCost') || isempty(logged.prevCost)
        reward = 0; % baseline on first step of episode
    else
        reward_scale = 100;
        denom = max(1e-3, abs(logged.prevCost));
        reward = reward_scale * (logged.prevCost - J) / denom;
    end
    logged.prevCost = J;

    % phase features
    t_abs = get_snapshot_time_or_zero();
    omega = get_gait_omega();

    % next observation
    nextObs = [nt; nu; R_new./upper_abs; sin(omega*t_abs); cos(omega*t_abs)];

    % persist weights only when applying
    if shouldApply
        last_R = R_new;
        save('LastR.mat','last_R');
        fprintf('dR_norm=%.3f | J=%.4f | nt=%.4f | nu=%.4f | reward=%.4f\n', ...
        norm(dR_frac), J, nt, nu, reward);
    end

    % ---------- termination ----------
    stepCount = stepCount + 1;
    isDone = (stepCount >= cfg.EP_STEPS);
    if isDone
        stepCount = 0;
    end
end
