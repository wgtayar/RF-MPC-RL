function [nextObs, reward, isDone, logged] = rlStepFunction(dR_frac, logged)

    % episode counter
    persistent stepCount cfg lower_abs upper_abs
    if isempty(stepCount); stepCount = 0; end

    if isempty(cfg) || isempty(lower_abs) || isempty(upper_abs)
        gait = 0;
        p = get_params(gait);
        initR = [p.R(1,1); p.R(2,2); p.R(3,3)];
        lower_abs = 0.01 * initR;
        upper_abs = 5.00 * initR;
        cfg.EP_STEPS = 10;
        cfg.APPLY_EVERY = 10;

        % If a saved config exists, prefer it.
        if exist('rlEnv_MPC_R.mat','file')
            try
                S0 = load('rlEnv_MPC_R.mat');
                if isfield(S0,'cfg'); cfg = S0.cfg; end
                if isfield(S0,'lower_abs');  lower_abs = S0.lower_abs; end
                if isfield(S0,'upper_abs');  upper_abs = S0.upper_abs; end
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

    % apply new action only at episode boundary
    shouldApply = (mod(stepCount, cfg.APPLY_EVERY) == 0);

    if shouldApply
        % dR_frac ∈ [-0.10, 0.10]
        R_new = last_R .* (1 + dR_frac);
        R_new = min(max(R_new, lower_abs), upper_abs);
        disp('New R:')
        disp(R_new)
    else
        R_new = last_R;
    end

    % run one simulation chunk; snapshot advances
    R_all = repmat(R_new,[4,1]);
    [te, ue, i_tot] = run_MPC_simulation(R_all, 0);

    T_ref = 1.5;
    U_ref = 1.5e5;
    I_iterations = 4120;
    % I_ref = 10.102;
    I_ref = 1.0526;
    nt = te / T_ref;
    nu = ue / U_ref;
    i_consumed = i_tot / I_iterations;

    mt = i_consumed - I_ref;
    % alpha = 0.6;  beta = 0.4;
    % J = alpha*nt + beta*nu;

    disp('I consumed')
    disp(i_consumed)

    disp('mt')
    disp(mt)

    phi_m = log(1 + mt/0.88) - exp(-mt/0.88); % 0.88 is g(X)

    phi_trunc = fix(phi_m * 10^4) / 10^4;

    if ~isfield(logged,'prevCost') || isempty(logged.prevCost)
        reward = 0; % baseline on first step of episode
    else
        reward = phi_m;
        disp('Reward:')
        disp(reward)
    end

    logged.prevCost = phi_m;

    % next observation
    nextObs = [nt; nu; i_consumed; R_new./upper_abs];

    % persist weights only when applying
    if shouldApply
        last_R = R_new;
        save('LastR.mat','last_R');
        fprintf('dR_norm=%.3f | phi_m=%.4f | nt=%.4f | nu=%.4f | reward=%.4f\n', ...
        norm(dR_frac), phi_m, nt, nu, reward);
    end

    % ---------- termination ----------
    stepCount = stepCount + 1;
    isDone = (stepCount >= cfg.EP_STEPS);
    if isDone
        stepCount = 0;
    end
end
