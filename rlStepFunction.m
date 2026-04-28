function [nextObs, reward, isDone, logged] = rlStepFunction(action, logged)
    rootDir = fileparts(mfilename('fullpath'));
    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');
    lastRPath = fullfile(rootDir, 'LastR.mat');

    S = load(cfgPath, 'cfg', 'lower_abs', 'upper_abs');
    cfg = S.cfg;
    lower_abs = S.lower_abs;
    upper_abs = S.upper_abs;

    dR_frac = action(1:3);
    gamma_v_raw = action(4);
    gamma_a = action(5);
    
    if isfield(cfg, 'DGAMMA_V_MAX') && isfield(logged, 'prev_gamma_v')
        gamma_v = min(max(gamma_v_raw, logged.prev_gamma_v - cfg.DGAMMA_V_MAX), ...
            logged.prev_gamma_v + cfg.DGAMMA_V_MAX);
    else
        gamma_v = gamma_v_raw;
    end
    
    gamma_v = min(max(gamma_v, cfg.GAMMA_V_MIN), cfg.GAMMA_V_MAX);
    gamma_a = min(max(gamma_a, cfg.GAMMA_A_MIN), cfg.GAMMA_A_MAX);

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
    DX_all = zeros(cfg.APPLY_EVERY, 1);
    feasible = true;
    fail_reason = '';
    battery = logged.battery;

    for k = 1:cfg.APPLY_EVERY
        battery_before_chunk = battery;
        simOut = run_MPC_simulation(R_all, 0, v_exec, a_exec, cfg);

        te_all(k) = simOut.tracking_error_total;
        ue_all(k) = simOut.control_effort_total;
        Q_all(k) = simOut.charge_total;
        T_all(k) = simOut.current_duration;
        DX_all(k) = simOut.dx_forward;
        battery = simOut.battery;

        chunk_abs = (decision_idx - 1) * cfg.APPLY_EVERY + k;
        mission_time = chunk_abs * cfg.CHUNK_DURATION;

        if T_all(k) > 0
            Ieq_chunk = Q_all(k) / T_all(k);
        else
            Ieq_chunk = cfg.IEQ_REF;
        end
        
        if isfield(cfg, 'RUN') && cfg.RUN.enabled
            chunkRow = struct();
            chunkRow.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
            chunkRow.episode_idx = logged.episode_idx;
            chunkRow.decision_idx = decision_idx;
            chunkRow.chunk_in_decision = k;
            chunkRow.global_chunk_idx = chunk_abs;
        
            chunkRow.dR1 = dR_frac(1);
            chunkRow.dR2 = dR_frac(2);
            chunkRow.dR3 = dR_frac(3);
            chunkRow.gamma_v = gamma_v;
            chunkRow.gamma_a = gamma_a;

            chunkRow.gamma_v_raw = gamma_v_raw;
            chunkRow.v_from_gamma = cfg.V_MIN + gamma_v * (cfg.V_MAX - cfg.V_MIN);
        
            chunkRow.R1 = R_new(1);
            chunkRow.R2 = R_new(2);
            chunkRow.R3 = R_new(3);
        
            chunkRow.v_req = logged.v_req;
            chunkRow.a_req = logged.a_req;
            chunkRow.v_exec = v_exec;
            chunkRow.a_exec = a_exec;
        
            chunkRow.soc_start_pct = battery_before_chunk.soc_pct;
            chunkRow.soc_end_pct = battery.soc_pct;
            chunkRow.Q_chunk_As = Q_all(k);
            chunkRow.T_chunk_s = T_all(k);
            chunkRow.Ieq_chunk_A = Ieq_chunk;
            chunkRow.dx_chunk_m = DX_all(k);
        
            chunkRow.feasible = simOut.feasible;
            chunkRow.fail_reason = string(simOut.fail_reason);
            chunkRow.fail_iter = simOut.fail_iter;
            chunkRow.fail_time_s = simOut.fail_time_s;
            chunkRow.fail_state_norm = simOut.fail_state_norm;
            chunkRow.fail_input_norm = simOut.fail_input_norm;
            chunkRow.fail_com_speed = simOut.fail_com_speed;
            chunkRow.fail_fsm_leg1 = simOut.fail_fsm_leg1;
            chunkRow.fail_h_rcond = simOut.fail_h_rcond;
            chunkRow.fail_Aineq_rows = simOut.fail_Aineq_rows;
            chunkRow.fail_Aineq_cols = simOut.fail_Aineq_cols;
            chunkRow.fail_Aeq_rows = simOut.fail_Aeq_rows;
            chunkRow.fail_Aeq_cols = simOut.fail_Aeq_cols;
            chunkRow.qp_exitflag_last = simOut.qp_exitflag_last;
        
            chunkRow.state_norm_end = simOut.state_norm_end;
            chunkRow.input_norm_end = simOut.input_norm_end;
            chunkRow.com_speed_end = simOut.com_speed_end;
            if isfield(simOut, 'Tst_end')
                chunkRow.Tst_end = simOut.Tst_end;
            else
                chunkRow.Tst_end = NaN;
            end
            
            chunkRow.state_norm_proxy = simOut.state_norm_end;
            
            if isfield(simOut, 'fsm_leg1_end') && isfinite(simOut.fsm_leg1_end)
                chunkRow.fsm_proxy = simOut.fsm_leg1_end - 1;
            else
                chunkRow.fsm_proxy = NaN;
            end
            if isfield(simOut, 'fsm_leg1_end')
                chunkRow.fsm_leg1_end = simOut.fsm_leg1_end;
            else
                chunkRow.fsm_leg1_end = NaN;
            end
        
            chunkRow.tracking_error_total = simOut.tracking_error_total;
            chunkRow.control_effort_total = simOut.control_effort_total;
        
            append_csv_row(cfg.RUN.chunk_csv, chunkRow);
        end

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
    
    window_distance = sum(DX_all);
    distance_next = max(0, logged.distance_m + window_distance);
    
    time_frac_next = min(decision_idx / cfg.EP_STEPS, 1);
    progress_next = min(distance_next / cfg.MISSION.D_TARGET_M, 1);
    lag_frac_next = max(0, time_frac_next - progress_next);
    
    window.progress_frac = progress_next;
    window.time_frac = time_frac_next;
    window.lag_frac = lag_frac_next;
    
    window.distance_start_m = logged.distance_m;
    window.window_distance_m = window_distance;
    window.distance_end_m = distance_next;
    
    window.soc_start_pct = soc_start;
    window.soc_end_pct = soc_end;
    
    window.delta_v_exec = v_exec - logged.v_exec;
    window.delta_a_exec = a_exec - logged.a_exec;
    window.delta_gamma_v = gamma_v - logged.prev_gamma_v;
    window.delta_gamma_a = gamma_a - logged.prev_gamma_a;
    window.prev_Ieq_window = logged.prev_Ieq_window;
    
    window.dR1 = dR_frac(1);
    window.dR2 = dR_frac(2);
    window.dR3 = dR_frac(3);
    
    window.completed_episode = feasible && (distance_next >= cfg.MISSION.D_TARGET_M);
    
    if ~feasible
        window.terminal_reason = 'infeasible';
    elseif window.completed_episode
        window.terminal_reason = 'mission_complete';
    elseif battery.margin_norm <= cfg.BATTERY.terminal_margin
        window.terminal_reason = 'battery_terminal';
    elseif decision_idx >= cfg.EP_STEPS
        window.terminal_reason = 'time_limit';
    else
        window.terminal_reason = '';
    end

    [reward, info] = compute_rl_reward(window, cfg);

    nt = window.tracking_error_mean / cfg.TRACK_REF;
    nu = window.control_effort_mean / cfg.EFFORT_REF;

    window_charge = sum(Q_all);
    window_time = sum(T_all);
    dsoc = soc_end - soc_start;

    if exist('simOut', 'var') && isstruct(simOut)
        com_speed_mag = simOut.com_speed_end;
    
        if isfield(simOut, 'Tst_end') && isfinite(simOut.Tst_end) && cfg.OBS.NOMINAL_TST > 0
            tst_ratio = simOut.Tst_end / cfg.OBS.NOMINAL_TST;
        else
            tst_ratio = 1.0;
        end
    
        if isfield(simOut, 'state_norm_end') && isfinite(simOut.state_norm_end)
            state_norm_proxy = simOut.state_norm_end;
        else
            state_norm_proxy = 0;
        end
    
        if isfield(simOut, 'fsm_leg1_end') && isfinite(simOut.fsm_leg1_end)
            % stance=1 -> 0, swing=2 -> 1
            fsm_proxy = simOut.fsm_leg1_end - 1;
        else
            fsm_proxy = 0;
        end
    else
        com_speed_mag = 0;
        tst_ratio = 1.0;
        state_norm_proxy = 0;
        fsm_proxy = 0;
    end

    logged.step_idx = logged.step_idx + 1;
    logged.last_R = R_new;
    logged.v_exec = v_exec;
    logged.a_exec = a_exec;
    logged.prev_gamma_v = gamma_v;
    logged.prev_gamma_a = gamma_a;
    logged.prev_Ieq_window = Ieq_window;
    logged.distance_m = distance_next;
    logged.battery = battery;
    logged.window = info;
    logged.episode_charge_total = logged.episode_charge_total + window_charge;
    logged.episode_time_total = logged.episode_time_total + window_time;

    if isfield(cfg, 'RUN') && cfg.RUN.enabled
        decisionRow = struct();
        decisionRow.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
        decisionRow.episode_idx = logged.episode_idx;
        decisionRow.decision_idx = decision_idx;
        decisionRow.progress_frac = progress_next;
        decisionRow.time_frac = time_frac_next;
        decisionRow.lag_frac = lag_frac_next;
        decisionRow.distance_start_m = window.distance_start_m;
        decisionRow.window_distance_m = window.window_distance_m;
        decisionRow.distance_end_m = window.distance_end_m;
    
        decisionRow.dR1 = dR_frac(1);
        decisionRow.dR2 = dR_frac(2);
        decisionRow.dR3 = dR_frac(3);
        decisionRow.gamma_v = gamma_v;
        decisionRow.gamma_a = gamma_a;

        decisionRow.gamma_v_raw = gamma_v_raw;
        decisionRow.v_from_gamma = cfg.V_MIN + gamma_v * (cfg.V_MAX - cfg.V_MIN);

        decisionRow.R1 = R_new(1);
        decisionRow.R2 = R_new(2);
        decisionRow.R3 = R_new(3);
    
        decisionRow.v_req = logged.v_req;
        decisionRow.a_req = logged.a_req;
        decisionRow.v_exec = v_exec;
        decisionRow.a_exec = a_exec;
    
        decisionRow.soc_start_pct = soc_start;
        decisionRow.soc_end_pct = soc_end;
        decisionRow.dsoc_pct = dsoc;
    
        decisionRow.window_charge_As = window_charge;
        decisionRow.window_time_s = window_time;
        decisionRow.Ieq_window_A = Ieq_window;

        decisionRow.prev_Ieq_window_A = window.prev_Ieq_window;
        decisionRow.delta_v_exec = window.delta_v_exec;
        decisionRow.delta_a_exec = window.delta_a_exec;
        decisionRow.delta_gamma_v = window.delta_gamma_v;
        decisionRow.delta_gamma_a = window.delta_gamma_a;
    
        decisionRow.tracking_error_mean = window.tracking_error_mean;
        decisionRow.control_effort_mean = window.control_effort_mean;
    
        decisionRow.nt = nt;
        decisionRow.nu = nu;
        decisionRow.com_speed_mag = com_speed_mag;
        decisionRow.tst_ratio = tst_ratio;
        decisionRow.state_norm_proxy = state_norm_proxy;
        decisionRow.fsm_proxy = fsm_proxy;
        decisionRow.reward = reward;
        decisionRow.q_pace = info.q_pace;
        decisionRow.pace_reward = info.pace_reward;
        decisionRow.lag_penalty = info.lag_penalty;
        decisionRow.risk_score = info.risk_score;
        decisionRow.risk_static = info.risk_static;
        decisionRow.risk_transition = info.risk_transition;
        decisionRow.risk_I = info.risk_I;
        decisionRow.risk_track = info.risk_track;
        decisionRow.risk_a = info.risk_a;
        decisionRow.risk_dv = info.risk_dv;
        decisionRow.risk_dgv = info.risk_dgv;
        decisionRow.risk_r2 = info.risk_r2;
        decisionRow.dyn_gate = info.dyn_gate;
        decisionRow.battery_penalty = info.battery_penalty;
        decisionRow.soc_stress = info.soc_stress;
        decisionRow.slow_pen = info.slow_pen;
        decisionRow.nt_raw = info.nt_raw;
        decisionRow.nu_raw = info.nu_raw;
        decisionRow.feasible = feasible;
        decisionRow.terminal_reason = string(window.terminal_reason);
        decisionRow.fail_reason = string(fail_reason);
    
        append_csv_row(cfg.RUN.decision_csv, decisionRow);
    
        if ~feasible
            failureRow = decisionRow;
            append_csv_row(cfg.RUN.failure_csv, failureRow);
        end
    end

    last_R = R_new;
    save(lastRPath, 'last_R');
    
    com_speed_mag = simOut.com_speed_end;
    
    if isfield(simOut, 'Tst_end') && isfinite(simOut.Tst_end) && cfg.OBS.NOMINAL_TST > 0
        tst_ratio = simOut.Tst_end / cfg.OBS.NOMINAL_TST;
    else
        tst_ratio = 1.0;
    end
    
    state_norm_proxy = simOut.state_norm_end;
    
    if isfield(simOut, 'fsm_leg1_end') && isfinite(simOut.fsm_leg1_end)
        % stance = 1 -> 0, swing = 2 -> 1
        fsm_proxy = simOut.fsm_leg1_end - 1;
    else
        fsm_proxy = 0;
    end
    
    nextObs = build_rl_observation( ...
        nt, nu, battery, progress_next, lag_frac_next, ...
        logged.v_req, logged.a_req, v_exec, a_exec, ...
        R_new, lower_abs, upper_abs, cfg, ...
        com_speed_mag, tst_ratio, state_norm_proxy, fsm_proxy, ...
        gamma_v, gamma_a, Ieq_window);

    isDone = ~feasible || window.completed_episode || battery.margin_norm <= cfg.BATTERY.terminal_margin || logged.step_idx >= cfg.EP_STEPS;

    if isfield(cfg, 'RUN') && cfg.RUN.enabled && isfield(cfg, 'CHECKPOINT')
        shouldSaveCheckpoint = mod(decision_idx, cfg.CHECKPOINT.every_decisions) == 0 || isDone;
    
        if shouldSaveCheckpoint
            checkpoint = struct();
            checkpoint.timestamp = datestr(now, 'dd-mm-yyyy HH:MM:SS');
            checkpoint.episode_idx = logged.episode_idx;
            checkpoint.decision_idx = decision_idx;
            checkpoint.progress_frac = progress_next;
    
            checkpoint.action = action(:).';
            checkpoint.dR_frac = dR_frac(:).';
            checkpoint.gamma_v = gamma_v;
            checkpoint.gamma_a = gamma_a;

            checkpoint.gamma_v_raw = gamma_v_raw;
    
            checkpoint.R_new = R_new(:).';
            checkpoint.v_req = logged.v_req;
            checkpoint.a_req = logged.a_req;
            checkpoint.v_exec = v_exec;
            checkpoint.a_exec = a_exec;
    
            checkpoint.soc_start_pct = soc_start;
            checkpoint.soc_end_pct = soc_end;
            checkpoint.dsoc_pct = soc_end - soc_start;
    
            checkpoint.window_charge_As = window_charge;
            checkpoint.window_time_s = window_time;
            checkpoint.Ieq_window_A = Ieq_window;
    
            checkpoint.reward = reward;
            checkpoint.feasible = feasible;
            checkpoint.fail_reason = fail_reason;
            checkpoint.terminal_reason = window.terminal_reason;
            checkpoint.completed_episode = window.completed_episode;
    
            checkpoint.nt = nt;
            checkpoint.nu = nu;
            checkpoint.reward_info = info;
    
            checkpoint.episode_charge_total_As = logged.episode_charge_total;
            checkpoint.episode_time_total_s = logged.episode_time_total;
    
            save_rl_checkpoint(cfg.RUN.checkpoint_file, checkpoint);
        end
    end

    if isfield(cfg, 'LOG') && cfg.LOG.enable && cfg.LOG.print_decision
        fprintf('[EP %d | DEC %d/%d END] dR=[%.3f %.3f %.3f], gv=%.3f, ga=%.3f, v=%.3f, a=%.3f, SOC: %.2f%% -> %.2f%% (dSOC=%.2f%%), Q=%.3f A*s, Ieq=%.3f A, reward=%.4f, feasible=%d\n', ...
            logged.episode_idx, decision_idx, cfg.EP_STEPS, ...
            dR_frac(1), dR_frac(2), dR_frac(3), gamma_v, gamma_a, v_exec, a_exec, ...
            soc_start, soc_end, dsoc, window_charge, Ieq_window, reward, feasible);
    end

    if isDone && isfield(cfg, 'LOG') && cfg.LOG.enable && cfg.LOG.print_episode
        if ~feasible
            reason = 'infeasible';
        elseif window.completed_episode
            reason = 'mission_complete';
        elseif battery.margin_norm <= cfg.BATTERY.terminal_margin
            reason = 'battery_terminal';
        else
            reason = 'time_limit';
        end

        fprintf('[EP %d END] reason=%s, final_SOC=%.2f%%, total_Q=%.3f A*s, total_time=%.1f s\n\n', ...
            logged.episode_idx, reason, battery.soc_pct, logged.episode_charge_total, logged.episode_time_total);
    end
end