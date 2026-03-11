function [nextObs, reward, isDone, logged] = rlStepFunction(dR_frac, logged)

    persistent stepCount cfg lower_abs upper_abs R_episode
    if isempty(stepCount); stepCount = 0; end

    if isempty(cfg) || isempty(lower_abs) || isempty(upper_abs)
        gait = 0;
        p = get_params(gait);
        initR = [p.R(1,1); p.R(2,2); p.R(3,3)];
    
        % New bounds
        lower_abs = 0.5 * initR;
        upper_abs = 3.0 * initR;
    
        cfg.EP_STEPS = 30;
        cfg.APPLY_EVERY = 30;
    
        % Had a problem with the paths, i think it is now fixed (changed
        % the path to only include this directory)
        if exist('rlEnv_MPC_R.mat','file')
            try
                S0 = load('rlEnv_MPC_R.mat');
                if isfield(S0,'lower_abs'), lower_abs = S0.lower_abs; end
                if isfield(S0,'upper_abs'), upper_abs = S0.upper_abs; end
            catch
            end
        end
    end

    if nargin >= 2 && isfield(logged,'last_R') && ~isempty(logged.last_R)
        last_R = logged.last_R;
    elseif exist('LastR.mat','file')
        S = load('LastR.mat','last_R');
        last_R = S.last_R;
    else
        gait = 0; 
        p = get_params(gait);
        last_R = [p.R(1,1); p.R(2,2); p.R(3,3)];
        save('LastR.mat','last_R');
    end

    shouldApply = (stepCount == 0);

    if shouldApply
        R_new = last_R .* (1 + dR_frac);
        R_new = min(max(R_new, lower_abs), upper_abs);

        % Store R for the rest of the episode
        R_episode = R_new;

        disp('New R for this episode:')
        disp(R_new)
        disp('R bounds [lower upper]:')
        disp([lower_abs upper_abs])

    else
        if isempty(R_episode)
            R_episode = last_R;
        end
        R_new = R_episode;
    end

    R_all = repmat(R_new,[4,1]);
    [te, ue, I_chunk_mean, SOC_current, feasible] = run_MPC_simulation(R_all, 0);

    % DEBUG: log RL step & MPC call
    dbg_step = struct();
    dbg_step.timestamp = datestr(now,'yyyy-mm-dd HH:MM:SS.FFF');
    dbg_step.dR_frac = dR_frac;
    dbg_step.R_new = R_new;
    dbg_step.last_R = last_R;
    dbg_step.shouldApply = shouldApply;
    dbg_step.stepCount = stepCount;
    dbg_step.EP_STEPS = cfg.EP_STEPS;
    dbg_step.nt = te / 1.5;
    dbg_step.nu = ue / 1.5e5;
    dbg_step.I_chunk_mean = I_chunk_mean;
    dbg_step.SOC_current = SOC_current;
    dbg_step.feasible = feasible;

    % Trying to read current episode index
    if exist('RL_meta.mat','file')
        try
            M = load('RL_meta.mat','ep_idx');
            if isfield(M,'ep_idx')
                dbg_step.ep_idx = M.ep_idx;
            end
        catch
        end
    end

    if exist('RL_step_log.mat','file')
        Slog = load('RL_step_log.mat','RL_steps');
        RL_steps = Slog.RL_steps;
        RL_steps(end+1) = dbg_step;
    else
        RL_steps = dbg_step;
    end
    save('RL_step_log.mat','RL_steps');
    % End debug

    T_ref = 16.21;
    U_ref = 5.4e4;
    % I_ref = 1.526;

    nt = te / T_ref;
    nu = ue / U_ref;

    % mean current
    i_consumed = I_chunk_mean;

    SOC_min = 0.2;
    mt = SOC_current - SOC_min;

    disp('I consumed')
    disp(i_consumed)
    
    disp('SOC_current:')
    disp(SOC_current)

    disp('mt:')
    disp(mt)

    w_soc = 1.0;
    w_nt = 0.01;
    w_nu = 0.01;

    z = 1 + mt/0.88;
    z = max(z, 1e-6); % avoid log of <= 0
    arg_exp = -mt/0.88;
    arg_exp = max(min(arg_exp, 50), -50);
    phi_m = log(z) - exp(arg_exp);

    if ~isfinite(phi_m) || ~isreal(phi_m)
        phi_m = -50;
    end
    phi_m = max(min(phi_m, 5), -50);

    reward = w_soc * phi_m - w_nt * nt - w_nu * nu;

    % infeasible branch
    if ~feasible
        reward = -50;
        logged.prevCost = phi_m;
        nextObs = [nt; nu; SOC_current; R_new./upper_abs];
        stepCount = 0;
        isDone = true;
        R_episode = [];

        disp('Reward (infeasible):')
        disp(reward)
        return;
    end

    updateBestRLog(R_new, SOC_current, te, ue, reward);

    % feasible branch
    disp('Reward:')
    disp(reward)
    
    logged.prevCost = phi_m;
    logged.last_R = R_new;

    % next observation (obs dimension = 6 as R is 3x1)
    nextObs = [nt; nu; SOC_current; R_new./upper_abs];

    if shouldApply
        last_R = R_new;
        save('LastR.mat','last_R');
        fprintf('dR_norm=%.3f | phi_m=%.4f | nt=%.4f | nu=%.4f | reward=%.4f\n', ...
            norm(dR_frac), phi_m, nt, nu, reward);
        fprintf('Episode R chosen: [%g %g %g]\n', R_new);
    end

    % Termination for normal RL episode
    stepCount = stepCount + 1;
    isDone = (stepCount >= cfg.EP_STEPS);
    if isDone
        stepCount = 0;
        R_episode = [];
    end
end

