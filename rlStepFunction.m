function [nextObs, reward, isDone, logged] = rlStepFunction(action, logged)
    rootDir = fileparts(mfilename('fullpath'));
    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');
    lastRPath = fullfile(rootDir, 'LastR.mat');

    S = load(cfgPath, 'cfg', 'lower_abs', 'upper_abs');
    cfg = S.cfg;
    lower_abs = S.lower_abs;
    upper_abs = S.upper_abs;

    dR_frac = action(1:3);
    gamma_v = action(4);
    gamma_a = action(5);

    R_new = logged.last_R .* (1 + dR_frac);
    R_new = min(max(R_new, lower_abs), upper_abs);
    R_all = repmat(R_new, [4, 1]);

    [v_exec, a_exec] = apply_command_governor(logged.v_req, logged.a_req, gamma_v, gamma_a, cfg);

    decision_idx = logged.step_idx + 1;
    total_chunks = cfg.EP_STEPS * cfg.APPLY_EVERY;
    soc_start = logged.battery.soc_pct;

    te_all = zeros(cfg.APPLY_EVERY, 1);
    ue_all = zeros(cfg.APPLY_EVERY, 1);
    Q_all = zeros(cfg.APPLY_EVERY, 1);
    T_all = zeros(cfg.APPLY_EVERY, 1);
    feasible = true;
    fail_reason = '';
    battery = logged.battery;

    for k = 1:cfg.APPLY_EVERY
        simOut = run_MPC_simulation(R_all, 0, v_exec, a_exec, cfg);

        te_all(k) = simOut.tracking_error_total;
        ue_all(k) = simOut.control_effort_total;
        Q_all(k) = simOut.charge_total;
        T_all(k) = simOut.current_duration;
        battery = simOut.battery;

        chunk_abs = (decision_idx - 1) * cfg.APPLY_EVERY + k;
        mission_time = chunk_abs * cfg.CHUNK_DURATION;

        if T_all(k) > 0
            Ieq_chunk = Q_all(k) / T_all(k);
        else
            Ieq_chunk = cfg.IEQ_REF;
        end

        if isfield(cfg, 'LOG') && cfg.LOG.enable && cfg.LOG.print_chunk
            fprintf('[EP %d | DEC %d/%d | CHUNK %d/%d | %d/%d] t=%.1f s, Q=%.3f A*s, Ieq=%.3f A, SOC=%.2f%%\n', ...
                logged.episode_idx, decision_idx, cfg.EP_STEPS, chunk_abs, total_chunks, k, cfg.APPLY_EVERY, ...
                mission_time, Q_all(k), Ieq_chunk, battery.soc_pct);
        end

        if ~simOut.feasible
            feasible = false;
            fail_reason = simOut.fail_reason;

            if isfield(cfg, 'LOG') && cfg.LOG.enable
                fprintf('[EP %d | DEC %d/%d | CHUNK %d/%d] FAIL reason=%s\n', ...
                    logged.episode_idx, decision_idx, cfg.EP_STEPS, chunk_abs, total_chunks, fail_reason);
            end
            break
        end
    end

    T_window = sum(T_all);
    if T_window > 0
        Ieq_window = sum(Q_all) / T_window;
    else
        Ieq_window = cfg.IEQ_REF;
    end

    valid_idx = T_all > 0;

    window = struct();
    window.feasible = feasible;

    if any(valid_idx)
        window.tracking_error_mean = mean(te_all(valid_idx), 'omitnan');
        window.control_effort_mean = mean(ue_all(valid_idx), 'omitnan');
    else
        window.tracking_error_mean = NaN;
        window.control_effort_mean = NaN;
    end

    if isempty(window.tracking_error_mean) || isnan(window.tracking_error_mean)
        window.tracking_error_mean = 1e3;
    end
    if isempty(window.control_effort_mean) || isnan(window.control_effort_mean)
        window.control_effort_mean = 1e6;
    end

    window.Ieq_window = Ieq_window;
    window.battery = battery;
    window.v_req = logged.v_req;
    window.a_req = logged.a_req;
    window.v_exec = v_exec;
    window.a_exec = a_exec;

    soc_end = battery.soc_pct;
    progress_next = decision_idx / cfg.EP_STEPS;

    window.progress_frac = progress_next;
    window.soc_start_pct = soc_start;
    window.soc_end_pct = soc_end;
    window.completed_episode = feasible && (decision_idx >= cfg.EP_STEPS);

    if ~feasible
        window.terminal_reason = 'infeasible';
    elseif battery.margin_norm <= cfg.BATTERY.terminal_margin
        window.terminal_reason = 'battery_terminal';
    elseif decision_idx >= cfg.EP_STEPS
        window.terminal_reason = 'max_steps';
    else
        window.terminal_reason = '';
    end

    [reward, info] = compute_rl_reward(window, cfg);

    nt = window.tracking_error_mean / cfg.TRACK_REF;
    nu = window.control_effort_mean / cfg.EFFORT_REF;

    window_charge = sum(Q_all);
    window_time = sum(T_all);
    dsoc = soc_end - soc_start;

    logged.step_idx = logged.step_idx + 1;
    logged.last_R = R_new;
    logged.v_exec = v_exec;
    logged.a_exec = a_exec;
    logged.battery = battery;
    logged.window = info;
    logged.episode_charge_total = logged.episode_charge_total + window_charge;
    logged.episode_time_total = logged.episode_time_total + window_time;

    last_R = R_new;
    save(lastRPath, 'last_R');

    nextObs = build_rl_observation(nt, nu, battery, progress_next, logged.v_req, logged.a_req, v_exec, a_exec, R_new, lower_abs, upper_abs, cfg);

    isDone = ~feasible || battery.margin_norm <= cfg.BATTERY.terminal_margin || logged.step_idx >= cfg.EP_STEPS;

    if isfield(cfg, 'LOG') && cfg.LOG.enable && cfg.LOG.print_decision
        fprintf('[EP %d | DEC %d/%d END] dR=[%.3f %.3f %.3f], gv=%.3f, ga=%.3f, v=%.3f, a=%.3f, SOC: %.2f%% -> %.2f%% (dSOC=%.2f%%), Q=%.3f A*s, Ieq=%.3f A, reward=%.4f, feasible=%d\n', ...
            logged.episode_idx, decision_idx, cfg.EP_STEPS, ...
            dR_frac(1), dR_frac(2), dR_frac(3), gamma_v, gamma_a, v_exec, a_exec, ...
            soc_start, soc_end, dsoc, window_charge, Ieq_window, reward, feasible);
    end

    if isDone && isfield(cfg, 'LOG') && cfg.LOG.enable && cfg.LOG.print_episode
        if ~feasible
            reason = 'infeasible';
        elseif battery.margin_norm <= cfg.BATTERY.terminal_margin
            reason = 'battery_terminal';
        else
            reason = 'max_steps';
        end

        fprintf('[EP %d END] reason=%s, final_SOC=%.2f%%, total_Q=%.3f A*s, total_time=%.1f s\n\n', ...
            logged.episode_idx, reason, battery.soc_pct, logged.episode_charge_total, logged.episode_time_total);
    end
end