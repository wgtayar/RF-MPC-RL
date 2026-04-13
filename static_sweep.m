function Results = static_sweep(sweepCfg)
    rootDir = fileparts(mfilename('fullpath'));

    add_if_exists(fullfile(rootDir, 'fcns'));
    add_if_exists(fullfile(rootDir, 'fcns_MPC'));

    if nargin < 1
        sweepCfg = struct();
    end

    sweepCfg = fill_static_sweep_defaults(sweepCfg, rootDir);

    logsRoot = fullfile(rootDir, 'MPC Boundaries Exploration');
    if ~exist(logsRoot, 'dir')
        mkdir(logsRoot);
    end

    runStamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    runDir = fullfile(logsRoot, ['run_' runStamp]);
    if ~exist(runDir, 'dir')
        mkdir(runDir);
    end

    matFile = fullfile(runDir, ['static_sweep_results_' runStamp '.mat']);
    bufferMatFile = fullfile(runDir, ['static_sweep_buffer_current_' runStamp '.mat']);
    csvFile = fullfile(runDir, ['static_sweep_cases_' runStamp '.csv']);
    rhoCsvFile = fullfile(runDir, ['static_sweep_summary_by_rho_' runStamp '.csv']);
    vaCsvFile = fullfile(runDir, ['static_sweep_summary_by_va_' runStamp '.csv']);
    txtFile = fullfile(runDir, ['static_sweep_log_' runStamp '.txt']);

    fid = fopen(txtFile, 'w');
    if fid < 0
        error('Could not create log file: %s', txtFile);
    end
    cleanupObj = onCleanup(@() fclose(fid));

    log_msg(fid, sweepCfg, 'STATIC SWEEP START');
    log_msg(fid, sweepCfg, ['Run directory: ' runDir]);
    log_msg(fid, sweepCfg, sprintf('chunk_duration = %.3f s', sweepCfg.chunk_duration));
    log_msg(fid, sweepCfg, sprintf('gait = %d', sweepCfg.gait));
    log_msg(fid, sweepCfg, sprintf('compute_current_battery = %d', sweepCfg.compute_current_battery));
    log_msg(fid, sweepCfg, sprintf('save_iteration_logs = %d', sweepCfg.save_iteration_logs));
    log_msg(fid, sweepCfg, sprintf('save_full_qp_on_failure = %d', sweepCfg.save_full_qp_on_failure));
    log_msg(fid, sweepCfg, sprintf('save_full_qp_each_iter = %d', sweepCfg.save_full_qp_each_iter));
    log_msg(fid, sweepCfg, sprintf('autosave_every = %d cases', sweepCfg.autosave_every));
    log_msg(fid, sweepCfg, 'Causality note: static sweep only diagnoses absolute combination feasibility from nominal reset. It cannot, by itself, prove abrupt-change failure.');
    log_msg(fid, sweepCfg, 'rhoR is an absolute multiplier on nominal R, not the RL relative step size.');

    [gridTable, meta] = build_static_grid(sweepCfg);
    nCases = height(gridTable);

    if sweepCfg.use_anisotropic_R
        log_msg(fid, sweepCfg, sprintf('safe isotropic rho count = %d', numel(sweepCfg.safe_iso_rho_vals)));
        log_msg(fid, sweepCfg, sprintf('safe v count = %d', numel(sweepCfg.safe_v_vals)));
        log_msg(fid, sweepCfg, sprintf('safe a count = %d', numel(sweepCfg.safe_a_vals)));
        log_msg(fid, sweepCfg, sprintf('aggressive rho1 count = %d', numel(sweepCfg.aggr_rho1_vals)));
        log_msg(fid, sweepCfg, sprintf('aggressive rho2 count = %d', numel(sweepCfg.aggr_rho2_vals)));
        log_msg(fid, sweepCfg, sprintf('aggressive rho3 count = %d', numel(sweepCfg.aggr_rho3_vals)));
        log_msg(fid, sweepCfg, sprintf('aggressive v count = %d', numel(sweepCfg.aggr_v_vals)));
        log_msg(fid, sweepCfg, sprintf('aggressive a count = %d', numel(sweepCfg.aggr_a_vals)));
    else
        log_msg(fid, sweepCfg, sprintf('safe isotropic rho count = %d', numel(sweepCfg.safe_iso_rho_vals)));
        log_msg(fid, sweepCfg, sprintf('safe v count = %d', numel(sweepCfg.safe_v_vals)));
        log_msg(fid, sweepCfg, sprintf('safe a count = %d', numel(sweepCfg.safe_a_vals)));
    end
    
    log_msg(fid, sweepCfg, sprintf('feasible masked combinations to test = %d', nCases));

    bufferResults = repmat(empty_case_result(), sweepCfg.flush_every_cases, 1);
    bufferCount = 0;
    csvInitialized = false;
    nFlushed = 0;

    tSweep = tic;

    for idx = 1:nCases
        rhoR1 = gridTable.rhoR1(idx);
        rhoR2 = gridTable.rhoR2(idx);
        rhoR3 = gridTable.rhoR3(idx);
        vCmd = gridTable.v_cmd(idx);
        aCmd = gridTable.a_cmd(idx);

        if sweepCfg.print_case_start
            log_msg(fid, sweepCfg, sprintf('START CASE %d/%d | rho=[%.3f %.3f %.3f] | v=%.3f | a=%.3f', ...
                idx, nCases, rhoR1, rhoR2, rhoR3, vCmd, aCmd));
        end

        caseResult = simulate_static_case(rhoR1, rhoR2, rhoR3, vCmd, aCmd, sweepCfg);
        
        caseResult.case_idx = idx;
        caseResult.run_stamp = runStamp;
        caseResult.run_dir = runDir;
        
        bufferCount = bufferCount + 1;
        bufferResults(bufferCount) = caseResult;
        
        log_msg(fid, sweepCfg, format_case_log_line(caseResult, nCases));
        
        if ~caseResult.feasible && sweepCfg.print_failure_snapshot
            failLine = sprintf(['FAIL SNAPSHOT | case=%d | reason=%s | fail_iter=%g | fail_t=%.3f | ' ...
                'fsm=%s | com_speed=%.4f | state_norm=%.4f | input_norm=%.4f | h_rcond=%.3e | ' ...
                'Aineq=%dx%d | Aeq=%dx%d'], ...
                idx, caseResult.fail_reason, caseResult.fail_iter, caseResult.fail_time_s, ...
                num2str(caseResult.fail_fsm_leg1), caseResult.fail_com_speed, caseResult.fail_state_norm, ...
                caseResult.fail_input_norm, caseResult.fail_h_rcond, ...
                caseResult.fail_Aineq_rows, caseResult.fail_Aineq_cols, ...
                caseResult.fail_Aeq_rows, caseResult.fail_Aeq_cols);
            log_msg(fid, sweepCfg, failLine);
        end
        
        shouldFlush = (bufferCount >= sweepCfg.flush_every_cases) || (idx == nCases);
        
        if shouldFlush
            flushBlock = bufferResults(1:bufferCount);
        
            save(bufferMatFile, 'flushBlock', 'meta', 'sweepCfg', 'runDir', 'runStamp');
        
            flushTable = case_results_to_table(flushBlock);
        
            if ~csvInitialized
                writetable(flushTable, csvFile);
                csvInitialized = true;
            else
                writetable(flushTable, csvFile, 'WriteMode', 'append');
            end
        
            nFlushed = nFlushed + bufferCount;
        
            log_msg(fid, sweepCfg, sprintf('FLUSH | wrote %d rows to CSV | total written=%d', ...
                bufferCount, nFlushed));
        
            bufferResults = repmat(empty_case_result(), sweepCfg.flush_every_cases, 1);
            bufferCount = 0;
        
            flushBlock = struct([]);
            save(bufferMatFile, 'flushBlock');
        end
    end

    totalElapsed = toc(tSweep);
    
    caseTable = readtable(csvFile);
    
    rhoTable = summarize_by_rho(caseTable);
    writetable(rhoTable, rhoCsvFile);
    
    vaTable = summarize_by_va(caseTable);
    writetable(vaTable, vaCsvFile);
    
    Results = struct();
    Results.run_stamp = runStamp;
    Results.run_dir = runDir;
    Results.created_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    Results.meta = meta;
    Results.sweepCfg = sweepCfg;
    Results.total_elapsed_sec = totalElapsed;
    Results.case_csv = csvFile;
    Results.summary_by_rho_csv = rhoCsvFile;
    Results.summary_by_va_csv = vaCsvFile;
    Results.case_table = caseTable;
    Results.summary_by_rho = rhoTable;
    Results.summary_by_va = vaTable;
    
    save(matFile, 'Results', '-v7.3');

    write_final_summary(fid, sweepCfg, Results);

    log_msg(fid, sweepCfg, sprintf('MAT file: %s', matFile));
    log_msg(fid, sweepCfg, sprintf('Rolling buffer MAT: %s', bufferMatFile));
    log_msg(fid, sweepCfg, sprintf('Case CSV: %s', csvFile));
    log_msg(fid, sweepCfg, sprintf('Rho summary CSV: %s', rhoCsvFile));
    log_msg(fid, sweepCfg, sprintf('VA summary CSV: %s', vaCsvFile));
    log_msg(fid, sweepCfg, 'STATIC SWEEP END');

    fprintf('[STATIC SWEEP END] run_dir=%s\n', runDir);
    fprintf('[STATIC SWEEP END] cases=%d, feasible=%d, infeasible=%d, elapsed=%.1f s\n', ...
        height(caseTable), sum(caseTable.feasible), sum(~caseTable.feasible), totalElapsed);
end

function sweepCfg = fill_static_sweep_defaults(sweepCfg, rootDir)
    if ~isfield(sweepCfg, 'gait')
        sweepCfg.gait = 0;
    end

    if ~isfield(sweepCfg, 'chunk_duration')
        sweepCfg.chunk_duration = 5;
    end

    if ~isfield(sweepCfg, 'use_anisotropic_R')
        sweepCfg.use_anisotropic_R = true;
    end
    
    if ~isfield(sweepCfg, 'safe_iso_rho_vals')
        sweepCfg.safe_iso_rho_vals = [0.9 1.0 1.1];
    end
    
    if ~isfield(sweepCfg, 'safe_v_vals')
        sweepCfg.safe_v_vals = 0.3:0.2:0.7;
    end
    
    if ~isfield(sweepCfg, 'safe_a_vals')
        sweepCfg.safe_a_vals = [0.5 0.9 1.3 1.7];
    end
    
    if ~isfield(sweepCfg, 'aggr_rho1_vals')
        sweepCfg.aggr_rho1_vals = [0.6 0.8 1.0 1.2 1.4 1.6];
    end
    
    if ~isfield(sweepCfg, 'aggr_rho2_vals')
        sweepCfg.aggr_rho2_vals = [0.8 1.0 1.2 1.4 1.6 1.8];
    end
    
    if ~isfield(sweepCfg, 'aggr_rho3_vals')
        sweepCfg.aggr_rho3_vals = [0.8 1.0 1.2 1.4 1.6 1.8];
    end
    
    if ~isfield(sweepCfg, 'aggr_v_vals')
        sweepCfg.aggr_v_vals = 0.8:0.1:1.5;
    end
    
    if ~isfield(sweepCfg, 'aggr_a_vals')
        sweepCfg.aggr_a_vals = [1.1 1.5 1.9 2.3 2.7 3.1 3.5 3.9];
    end

    if ~isfield(sweepCfg, 'use_feasible_va_mask')
        sweepCfg.use_feasible_va_mask = true;
    end

    if ~isfield(sweepCfg, 'tacc_min')
        sweepCfg.tacc_min = 0.2;
    end

    if ~isfield(sweepCfg, 'tacc_max')
        sweepCfg.tacc_max = 2.0;
    end

    if ~isfield(sweepCfg, 'compute_current_battery')
        sweepCfg.compute_current_battery = true;
    end

    if ~isfield(sweepCfg, 'save_iteration_logs')
        sweepCfg.save_iteration_logs = true;
    end

    if ~isfield(sweepCfg, 'save_full_qp_on_failure')
        sweepCfg.save_full_qp_on_failure = true;
    end

    if ~isfield(sweepCfg, 'save_full_qp_each_iter')
        sweepCfg.save_full_qp_each_iter = false;
    end

    if ~isfield(sweepCfg, 'save_fail_state_vectors')
        sweepCfg.save_fail_state_vectors = true;
    end

    if ~isfield(sweepCfg, 'save_success_snapshots')
        sweepCfg.save_success_snapshots = true;
    end

    if ~isfield(sweepCfg, 'autosave_every')
        sweepCfg.autosave_every = 25;
    end

    if ~isfield(sweepCfg, 'flush_every_cases')
        sweepCfg.flush_every_cases = 400;
    end
    
    if ~isfield(sweepCfg, 'keep_case_structs_in_final_mat')
        sweepCfg.keep_case_structs_in_final_mat = false;
    end

    if ~isfield(sweepCfg, 'print_to_cli')
        sweepCfg.print_to_cli = true;
    end

    if ~isfield(sweepCfg, 'print_case_start')
        sweepCfg.print_case_start = true;
    end

    if ~isfield(sweepCfg, 'print_failure_snapshot')
        sweepCfg.print_failure_snapshot = true;
    end

    if ~isfield(sweepCfg, 'cfgPath')
        sweepCfg.cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');
    end

    if exist(sweepCfg.cfgPath, 'file')
        S = load(sweepCfg.cfgPath, 'cfg');
        if isfield(S, 'cfg')
            sweepCfg.rlCfg = S.cfg;
        else
            sweepCfg.rlCfg = build_fallback_rl_cfg(rootDir);
        end
    else
        sweepCfg.rlCfg = build_fallback_rl_cfg(rootDir);
    end
end

function rlCfg = build_fallback_rl_cfg(rootDir)
    rlCfg = struct();

    rlCfg.BATTERY.metric_type = 'soc';
    rlCfg.BATTERY.SOC_init = 0.95;
    rlCfg.BATTERY.metric_init = 100 * rlCfg.BATTERY.SOC_init;
    rlCfg.BATTERY.metric_min = 20;
    rlCfg.BATTERY.metric_max = 100;
    rlCfg.BATTERY.terminal_margin = 0.20;
    rlCfg.BATTERY.C_nom_Ah = 2.0;
    rlCfg.BATTERY.pack_voltage = 12;
    rlCfg.BATTERY.DoD = 0.8;
    rlCfg.BATTERY.use_pack_sizing = false;
    rlCfg.BATTERY.n_series = 4;
    rlCfg.BATTERY.n_parallel = 4;
    rlCfg.BATTERY.decim = 10;
    rlCfg.BATTERY.make_plots = false;

    rlCfg.PROXY.betaMc = 0.5;
    rlCfg.PROXY.kneeCsv = fullfile(rootDir, 'Symmetric Knee Torques.csv');
    rlCfg.PROXY.alpha = 1;
    rlCfg.PROXY.beta = 2;
    rlCfg.PROXY.Kt = 0.0909;
    rlCfg.PROXY.eta = 0.90;
    rlCfg.PROXY.Nknee = 6 / 1.55;
    rlCfg.PROXY.tauHip4_joint = 6.114;
    rlCfg.PROXY.Nhip = 6;
end


function [gridTable, meta] = build_static_grid(sweepCfg)

    if ~sweepCfg.use_anisotropic_R
        [RHO, VV, AA] = ndgrid(sweepCfg.safe_iso_rho_vals, sweepCfg.safe_v_vals, sweepCfg.safe_a_vals);

        rho1 = RHO(:);
        rho2 = RHO(:);
        rho3 = RHO(:);
        vList = VV(:);
        aList = AA(:);

        if sweepCfg.use_feasible_va_mask
            aLo = max(min(sweepCfg.safe_a_vals), vList ./ sweepCfg.tacc_max);
            aHi = min(max(sweepCfg.safe_a_vals), vList ./ sweepCfg.tacc_min);
            mask = aList >= aLo & aList <= aHi;
        else
            mask = true(size(vList));
        end

        gridTable = table(rho1(mask), rho2(mask), rho3(mask), vList(mask), aList(mask), ...
            'VariableNames', {'rhoR1','rhoR2','rhoR3','v_cmd','a_cmd'});

    else
        % ---- SAFE REGION: isotropic and cheap ----
        [RS, VS, AS] = ndgrid(sweepCfg.safe_iso_rho_vals, sweepCfg.safe_v_vals, sweepCfg.safe_a_vals);

        safe_rho1 = RS(:);
        safe_rho2 = RS(:);
        safe_rho3 = RS(:);
        safe_v = VS(:);
        safe_a = AS(:);

        if sweepCfg.use_feasible_va_mask
            aLo_safe = max(min(sweepCfg.safe_a_vals), safe_v ./ sweepCfg.tacc_max);
            aHi_safe = min(max(sweepCfg.safe_a_vals), safe_v ./ sweepCfg.tacc_min);
            safeMask = safe_a >= aLo_safe & safe_a <= aHi_safe;
        else
            safeMask = true(size(safe_v));
        end

        safeTable = table( ...
            safe_rho1(safeMask), safe_rho2(safeMask), safe_rho3(safeMask), safe_v(safeMask), safe_a(safeMask), ...
            'VariableNames', {'rhoR1','rhoR2','rhoR3','v_cmd','a_cmd'});

        % ---- AGGRESSIVE REGION: anisotropic and focused ----
        [R1, R2, R3, VV, AA] = ndgrid( ...
            sweepCfg.aggr_rho1_vals, ...
            sweepCfg.aggr_rho2_vals, ...
            sweepCfg.aggr_rho3_vals, ...
            sweepCfg.aggr_v_vals, ...
            sweepCfg.aggr_a_vals);

        aggr_rho1 = R1(:);
        aggr_rho2 = R2(:);
        aggr_rho3 = R3(:);
        aggr_v = VV(:);
        aggr_a = AA(:);

        if sweepCfg.use_feasible_va_mask
            aLo_aggr = max(min(sweepCfg.aggr_a_vals), aggr_v ./ sweepCfg.tacc_max);
            aHi_aggr = min(max(sweepCfg.aggr_a_vals), aggr_v ./ sweepCfg.tacc_min);
            aggrMask = aggr_a >= aLo_aggr & aggr_a <= aHi_aggr;
        else
            aggrMask = true(size(aggr_v));
        end

        aggrTable = table( ...
            aggr_rho1(aggrMask), aggr_rho2(aggrMask), aggr_rho3(aggrMask), aggr_v(aggrMask), aggr_a(aggrMask), ...
            'VariableNames', {'rhoR1','rhoR2','rhoR3','v_cmd','a_cmd'});

        gridTable = [safeTable; aggrTable];
        gridTable = unique(gridTable, 'rows');
    end

    gridTable.rhoR_mean = mean([gridTable.rhoR1 gridTable.rhoR2 gridTable.rhoR3], 2);
    gridTable.rhoR_min = min([gridTable.rhoR1 gridTable.rhoR2 gridTable.rhoR3], [], 2);
    gridTable.rhoR_max = max([gridTable.rhoR1 gridTable.rhoR2 gridTable.rhoR3], [], 2);

    gridTable = sortrows(gridTable, {'v_cmd','a_cmd','rhoR_mean','rhoR1','rhoR2','rhoR3'});

    meta = struct();
    meta.use_anisotropic_R = sweepCfg.use_anisotropic_R;
    meta.safe_iso_rho_vals = sweepCfg.safe_iso_rho_vals;
    meta.safe_v_vals = sweepCfg.safe_v_vals;
    meta.safe_a_vals = sweepCfg.safe_a_vals;
    meta.aggr_rho1_vals = sweepCfg.aggr_rho1_vals;
    meta.aggr_rho2_vals = sweepCfg.aggr_rho2_vals;
    meta.aggr_rho3_vals = sweepCfg.aggr_rho3_vals;
    meta.aggr_v_vals = sweepCfg.aggr_v_vals;
    meta.aggr_a_vals = sweepCfg.aggr_a_vals;
    meta.use_feasible_va_mask = sweepCfg.use_feasible_va_mask;
    meta.tacc_min = sweepCfg.tacc_min;
    meta.tacc_max = sweepCfg.tacc_max;
    meta.static_scope = 'absolute_combination_from_nominal_reset_only';
end

function caseResult = simulate_static_case(rhoR1, rhoR2, rhoR3, vCmd, aCmd, sweepCfg)
    reset_mpc_case_state();
    
    gait = sweepCfg.gait;
    rlCfg = sweepCfg.rlCfg;

    p = get_params(gait);
    p_nom = get_params(gait);
    
    baseDiag = diag(p_nom.R);
    baseR1 = baseDiag(1);
    baseR2 = baseDiag(2);
    baseR3 = baseDiag(3);
    
    usedDiag = repmat([rhoR1 * baseR1; rhoR2 * baseR2; rhoR3 * baseR3], 4, 1);
    p.R = diag(usedDiag);
    
    p.vel_d = [vCmd; 0];
    p.acc_d = aCmd;
    p.yaw_d = 0;

    dt_sim = p.simTimeStep;
    maxIter = floor(sweepCfg.chunk_duration / dt_sim);

    qp_options = optimoptions('quadprog', 'Display', 'off');

    p_nom = get_params(gait);
    nominalRDiag = diag(p_nom.R);
    usedRDiag = diag(p.R);

    caseResult = empty_case_result();
    caseResult.rhoR1 = rhoR1;
    caseResult.rhoR2 = rhoR2;
    caseResult.rhoR3 = rhoR3;
    caseResult.rhoR_mean = mean([rhoR1 rhoR2 rhoR3]);
    caseResult.rhoR_min = min([rhoR1 rhoR2 rhoR3]);
    caseResult.rhoR_max = max([rhoR1 rhoR2 rhoR3]);
    caseResult.rhoR = caseResult.rhoR_mean;
    caseResult.v_cmd = vCmd;
    caseResult.a_cmd = aCmd;
    caseResult.chunk_duration = sweepCfg.chunk_duration;
    caseResult.dt_sim = dt_sim;
    caseResult.max_iter = maxIter;
    caseResult.nominal_R1 = nominalRDiag(1);
    caseResult.nominal_R2 = nominalRDiag(2);
    caseResult.nominal_R3 = nominalRDiag(3);
    caseResult.used_R1 = usedRDiag(1);
    caseResult.used_R2 = usedRDiag(2);
    caseResult.used_R3 = usedRDiag(3);
    caseResult.feasible = true;
    caseResult.fail_reason = '';
    caseResult.fail_reason_detail = '';
    caseResult.fail_iter = NaN;
    caseResult.fail_time_s = NaN;
    caseResult.quadprog_exitflag = NaN;
    caseResult.exception_id = '';
    caseResult.exception_message = '';
    caseResult.elapsed_sec = NaN;
    caseResult.causality_scope = 'static_combination_only';
    caseResult.root_cause_inference = 'not_applicable_until_case_finishes';

    if gait == 1
        [p, Xt, Ut] = fcn_bound_ref_traj(p);
    else
        [Xt, Ut] = fcn_gen_XdUd(0, [], [1;1;1;1], p);
    end

    caseResult.com_speed_start = norm(Xt(4:5));
    caseResult.state_norm_start = norm(Xt);

    if sweepCfg.compute_current_battery
        kneeCsv = rlCfg.PROXY.kneeCsv;
        betaMc = rlCfg.PROXY.betaMc;
        kneeTpl = load_knee_template(kneeCsv, betaMc);

        Dmc = readmatrix(kneeCsv);
        Tmc = Dmc(end,1) - Dmc(1,1);

        kneeParams = struct();
        kneeParams.Tmc = Tmc;
        kneeParams.alpha = rlCfg.PROXY.alpha;
        kneeParams.beta = rlCfg.PROXY.beta;
        kneeParams.Kt = rlCfg.PROXY.Kt;
        kneeParams.eta = rlCfg.PROXY.eta;
        kneeParams.Nknee = rlCfg.PROXY.Nknee;

        Ihip4 = joint_torque_to_current(rlCfg.PROXY.tauHip4_joint, rlCfg.PROXY.Nhip, rlCfg.PROXY.eta, rlCfg.PROXY.Kt);
        kneeState = init_knee_proxy_state();
    else
        kneeTpl = [];
        kneeParams = struct();
        Ihip4 = NaN;
        kneeState = [];
    end

    tracking_error = nan(maxIter, 1);
    control_effort = nan(maxIter, 1);
    state_norm = nan(maxIter, 1);
    input_norm = nan(maxIter, 1);
    com_speed = nan(maxIter, 1);
    qp_exitflag = nan(maxIter, 1);

    if sweepCfg.compute_current_battery
        knee_t = nan(maxIter, 1);
        knee_tau4 = nan(maxIter, 1);
        knee_I4 = nan(maxIter, 1);
        Tst_log = nan(maxIter, 1);
        Tsw_log = nan(maxIter, 1);
    else
        knee_t = [];
        knee_tau4 = [];
        knee_I4 = [];
        Tst_log = [];
        Tsw_log = [];
    end

    if sweepCfg.save_iteration_logs
        iterLog(maxIter,1) = empty_iter_log();
    else
        iterLog = [];
    end

    failSnapshot = empty_fail_snapshot();
    lastSuccess = empty_success_snapshot();

    t0 = tic;
    tstart = 0;
    tend = dt_sim;

    try
        for ii = 1:maxIter
            Xt_before = Xt;
            Ut_before = Ut;

            t_hor = tstart + p.Tmpc * (0:p.predHorizon-1);

            if gait == 1
                [FSM, Xd, Ud, Xt] = fcn_FSM_bound(t_hor, Xt, p);
            else
                [FSM, Xd, Ud, Xt] = fcn_FSM(t_hor, Xt, p);
            end

            if sweepCfg.compute_current_battery
                [kneeState, kneeOut] = knee_proxy_step(tstart, FSM(1), kneeState, kneeTpl, kneeParams);
                knee_t(ii) = kneeOut.t;
                knee_tau4(ii) = kneeOut.tau4;
                knee_I4(ii) = kneeOut.I4;

                if isfinite(kneeState.Tst)
                    Tst_log(ii) = kneeState.Tst;
                end
                if isfinite(kneeState.Tsw)
                    Tsw_log(ii) = kneeState.Tsw;
                end
            end

            [H, g, Aineq, bineq, Aeq, beq] = fcn_get_QP_form_eta(Xt, Ut, Xd, Ud, p);
            H = (H + H') / 2;

            qpDiag = compute_qp_diag(H, g, Aineq, bineq, Aeq, beq);

            [zval, ~, exitflag] = quadprog(H, g, Aineq, bineq, Aeq, beq, [], [], [], qp_options);
            qp_exitflag(ii) = exitflag;

            if exitflag <= 0 || isempty(zval)
                caseResult.feasible = false;
                caseResult.fail_reason = 'quadprog';
                caseResult.fail_iter = ii;
                caseResult.fail_time_s = tstart;
                caseResult.quadprog_exitflag = exitflag;
                caseResult.root_cause_inference = 'absolute_combination_infeasible_from_nominal_reset';

                failSnapshot = build_fail_snapshot( ...
                    ii, tstart, Xt_before, Ut_before, Xd, Ud, FSM, ...
                    H, g, Aineq, bineq, Aeq, beq, qpDiag, ...
                    Tst_log, Tsw_log, ii, sweepCfg, 'quadprog', '');
                break
            end

            ineq_margin_min = NaN;
            eq_residual_norm = NaN;
            if ~isempty(Aineq)
                ineq_margin_min = min(bineq - Aineq * zval);
            end
            if ~isempty(Aeq)
                eq_residual_norm = norm(Aeq * zval - beq);
            end

            Ut = Ut + zval(1:12);

            [u_ext, p_ext] = fcn_get_disturbance(tstart, p);
            p.p_ext = p_ext;
            u_ext = 0 * u_ext;

            Xdot_before = dynamics_SRB(tstart, Xt, Ut, Xd, u_ext, p);

            [~, X_chunk] = ode45(@(t, X) dynamics_SRB(t, X, Ut, Xd, u_ext, p), [tstart, tend], Xt);
            Xt_after = X_chunk(end,:).';

            if any(isnan(Xt_after)) || any(isinf(Xt_after))
                caseResult.feasible = false;
                caseResult.fail_reason = 'state_invalid';
                caseResult.fail_iter = ii;
                caseResult.fail_time_s = tend;
                caseResult.root_cause_inference = 'absolute_combination_causes_state_divergence_from_nominal_reset';

                failSnapshot = build_fail_snapshot( ...
                    ii, tend, Xt, Ut, Xd, Ud, FSM, ...
                    H, g, Aineq, bineq, Aeq, beq, qpDiag, ...
                    Tst_log, Tsw_log, ii, sweepCfg, 'state_invalid', '');
                failSnapshot.Xt_after_invalid = Xt_after;
                failSnapshot.Xdot_before = Xdot_before;
                break
            end

            Xt = Xt_after;

            tracking_error(ii) = sum((Xt - Xd(:,1)).^2);
            control_effort(ii) = sum(Ut.^2);
            state_norm(ii) = norm(Xt);
            input_norm(ii) = norm(Ut);
            com_speed(ii) = norm(Xt(4:5));

            lastSuccess = build_success_snapshot(ii, tstart, Xt, Ut, Xd, Ud, FSM, qpDiag, zval, ...
                ineq_margin_min, eq_residual_norm, sweepCfg, Tst_log, Tsw_log);

            if sweepCfg.save_iteration_logs
                iterLog(ii).iter = ii;
                iterLog(ii).t_start = tstart;
                iterLog(ii).t_end = tend;
                iterLog(ii).tracking_error = tracking_error(ii);
                iterLog(ii).control_effort = control_effort(ii);
                iterLog(ii).state_norm = state_norm(ii);
                iterLog(ii).input_norm = input_norm(ii);
                iterLog(ii).com_speed = com_speed(ii);
                iterLog(ii).qp_exitflag = qp_exitflag(ii);
                iterLog(ii).fsm_leg1 = FSM(1);
                iterLog(ii).ineq_margin_min = ineq_margin_min;
                iterLog(ii).eq_residual_norm = eq_residual_norm;
                iterLog(ii).h_rcond = qpDiag.h_rcond;
                iterLog(ii).h_fro = qpDiag.h_fro;
                iterLog(ii).h_min_diag = qpDiag.h_min_diag;
                iterLog(ii).h_max_diag = qpDiag.h_max_diag;
                iterLog(ii).Aineq_rows = qpDiag.Aineq_rows;
                iterLog(ii).Aeq_rows = qpDiag.Aeq_rows;
                iterLog(ii).g_norm = qpDiag.g_norm;
                iterLog(ii).bineq_norm = qpDiag.bineq_norm;
                iterLog(ii).beq_norm = qpDiag.beq_norm;
                iterLog(ii).z_step_norm = norm(zval(1:12));
                if sweepCfg.compute_current_battery
                    iterLog(ii).Tst = Tst_log(ii);
                    iterLog(ii).Tsw = Tsw_log(ii);
                    iterLog(ii).tau4 = knee_tau4(ii);
                    iterLog(ii).I4 = knee_I4(ii);
                end

                if sweepCfg.save_full_qp_each_iter
                    iterLog(ii).Xt = Xt;
                    iterLog(ii).Ut = Ut;
                    iterLog(ii).Xd1 = Xd(:,1);
                    iterLog(ii).Ud1 = Ud(:,1);
                    iterLog(ii).H = H;
                    iterLog(ii).g = g;
                    iterLog(ii).Aineq = Aineq;
                    iterLog(ii).bineq = bineq;
                    iterLog(ii).Aeq = Aeq;
                    iterLog(ii).beq = beq;
                    iterLog(ii).FSM = FSM;
                end
            end

            tstart = tend;
            tend = tstart + dt_sim;
        end
    catch ME
        caseResult.feasible = false;
        caseResult.fail_reason = 'exception';
        caseResult.fail_iter = ii_if_defined();
        caseResult.fail_time_s = tstart;
        caseResult.exception_id = ME.identifier;
        caseResult.exception_message = ME.message;
        caseResult.fail_reason_detail = ME.message;
        caseResult.root_cause_inference = 'exception_during_static_case';

        failSnapshot = build_fail_snapshot( ...
            ii_if_defined(), tstart, Xt, Ut, nan_safe_col(Xt), nan_safe_col(Ut), NaN, ...
            [], [], [], [], [], [], empty_qp_diag(), ...
            Tst_log, Tsw_log, ii_if_defined(), sweepCfg, 'exception', ME.message);
    end

    caseResult.elapsed_sec = toc(t0);
    caseResult.sim_time_completed_s = tstart;
    caseResult.completed_iterations = sum(isfinite(qp_exitflag) | isfinite(tracking_error) | isfinite(control_effort));

    if isempty(caseResult.fail_reason)
        caseResult.fail_reason = 'none';
        caseResult.root_cause_inference = 'static_combination_feasible_from_nominal_reset';
    end

    validDyn = isfinite(tracking_error);
    if any(validDyn)
        caseResult.tracking_error_total = sum(tracking_error(validDyn));
        caseResult.tracking_error_mean = mean(tracking_error(validDyn));
        caseResult.control_effort_total = sum(control_effort(validDyn));
        caseResult.control_effort_mean = mean(control_effort(validDyn));
        caseResult.state_norm_max = max(state_norm(validDyn));
        caseResult.state_norm_mean = mean(state_norm(validDyn));
        caseResult.input_norm_max = max(input_norm(validDyn));
        caseResult.input_norm_mean = mean(input_norm(validDyn));
        caseResult.com_speed_end = com_speed(find(validDyn, 1, 'last'));
        caseResult.com_speed_max = max(com_speed(validDyn));
    else
        caseResult.tracking_error_total = NaN;
        caseResult.tracking_error_mean = NaN;
        caseResult.control_effort_total = NaN;
        caseResult.control_effort_mean = NaN;
        caseResult.state_norm_max = norm(Xt);
        caseResult.state_norm_mean = norm(Xt);
        caseResult.input_norm_max = norm(Ut);
        caseResult.input_norm_mean = norm(Ut);
        caseResult.com_speed_end = norm(Xt(4:5));
        caseResult.com_speed_max = norm(Xt(4:5));
    end

    caseResult.state_norm_end = norm(Xt);

    if sweepCfg.compute_current_battery
        caseResult.Tstance_mean = safe_mean(Tst_log);
        caseResult.Tswing_mean = safe_mean(Tsw_log);
        if isfinite(caseResult.Tstance_mean) && isfinite(caseResult.Tswing_mean)
            caseResult.Tcycle_mean = caseResult.Tstance_mean + caseResult.Tswing_mean;
        else
            caseResult.Tcycle_mean = NaN;
        end

        validCurrent = isfinite(knee_t) & isfinite(knee_I4);
        tCurrent = knee_t(validCurrent);
        I_knee = knee_I4(validCurrent);
        tau_knee = knee_tau4(validCurrent);

        if numel(tCurrent) >= 2
            [tCurrent, ord] = sort(tCurrent);
            I_knee = I_knee(ord);
            tau_knee = tau_knee(ord);

            [tCurrent, uniqIdx] = unique(tCurrent);
            I_knee = I_knee(uniqIdx);
            tau_knee = tau_knee(uniqIdx);

            I_total = I_knee + Ihip4;

            caseResult.knee_tau_mean = safe_mean(abs(tau_knee));
            caseResult.knee_I_mean = safe_mean(abs(I_knee));
            caseResult.total_I_peak = max(abs(I_total));
            caseResult.charge_total_As = trapz(tCurrent, abs(I_total));
            caseResult.current_duration_s = tCurrent(end) - tCurrent(1);

            if caseResult.current_duration_s > 0
                caseResult.Ieq_A = caseResult.charge_total_As / caseResult.current_duration_s;
            else
                caseResult.Ieq_A = NaN;
            end

            batteryInit = struct();
            batteryInit.metric_type = rlCfg.BATTERY.metric_type;
            batteryInit.metric_value = rlCfg.BATTERY.metric_init;
            batteryInit.margin_norm = rlCfg.BATTERY.SOC_init;
            batteryInit.soc_pct = 100 * rlCfg.BATTERY.SOC_init;
            batteryInit.n_series = rlCfg.BATTERY.n_series;
            batteryInit.n_parallel = rlCfg.BATTERY.n_parallel;

            batteryOut = evaluate_battery_feedback(tCurrent, I_total, rlCfg.BATTERY, batteryInit);

            if isfield(batteryOut, 'soc_pct')
                caseResult.soc_end_pct = batteryOut.soc_pct;
            else
                caseResult.soc_end_pct = NaN;
            end
            if isfield(batteryOut, 'margin_norm')
                caseResult.battery_margin_norm = batteryOut.margin_norm;
            else
                caseResult.battery_margin_norm = NaN;
            end

            if sweepCfg.save_iteration_logs
                caseResult.current_trace_time = tCurrent;
                caseResult.current_trace_total = I_total;
                caseResult.current_trace_knee = I_knee;
            end
        else
            caseResult.Tstance_mean = safe_mean(Tst_log);
            caseResult.Tswing_mean = safe_mean(Tsw_log);
            caseResult.Tcycle_mean = NaN;
            caseResult.knee_tau_mean = NaN;
            caseResult.knee_I_mean = NaN;
            caseResult.total_I_peak = NaN;
            caseResult.charge_total_As = NaN;
            caseResult.current_duration_s = NaN;
            caseResult.Ieq_A = NaN;
            caseResult.soc_end_pct = NaN;
            caseResult.battery_margin_norm = NaN;
        end
    else
        caseResult.Tstance_mean = NaN;
        caseResult.Tswing_mean = NaN;
        caseResult.Tcycle_mean = NaN;
        caseResult.knee_tau_mean = NaN;
        caseResult.knee_I_mean = NaN;
        caseResult.total_I_peak = NaN;
        caseResult.charge_total_As = NaN;
        caseResult.current_duration_s = NaN;
        caseResult.Ieq_A = NaN;
        caseResult.soc_end_pct = NaN;
        caseResult.battery_margin_norm = NaN;
    end

    if sweepCfg.save_iteration_logs
        validIter = arrayfun(@(s) ~isempty(s.iter) && s.iter > 0, iterLog);
        caseResult.iteration_logs = iterLog(validIter);
    else
        caseResult.iteration_logs = struct([]);
    end

    if caseResult.feasible
        caseResult.fail_reason_detail = 'none';
    elseif isempty(caseResult.fail_reason_detail)
        caseResult.fail_reason_detail = caseResult.fail_reason;
    end

    if sweepCfg.save_success_snapshots
        caseResult.last_success_snapshot = lastSuccess;
    else
        caseResult.last_success_snapshot = struct([]);
    end

    if sweepCfg.save_full_qp_on_failure || sweepCfg.save_fail_state_vectors
        caseResult.fail_snapshot = failSnapshot;
    else
        caseResult.fail_snapshot = struct([]);
    end

    caseResult.fail_fsm_leg1 = failSnapshot.fsm_leg1;
    caseResult.fail_com_speed = failSnapshot.com_speed;
    caseResult.fail_state_norm = failSnapshot.state_norm;
    caseResult.fail_input_norm = failSnapshot.input_norm;
    caseResult.fail_h_rcond = failSnapshot.h_rcond;
    caseResult.fail_Aineq_rows = failSnapshot.Aineq_rows;
    caseResult.fail_Aineq_cols = failSnapshot.Aineq_cols;
    caseResult.fail_Aeq_rows = failSnapshot.Aeq_rows;
    caseResult.fail_Aeq_cols = failSnapshot.Aeq_cols;

    function out = ii_if_defined()
        try
            out = ii;
        catch
            out = NaN;
        end
    end
end

function qpDiag = compute_qp_diag(H, g, Aineq, bineq, Aeq, beq)
    qpDiag = empty_qp_diag();

    qpDiag.h_rcond = safe_rcond(H);
    qpDiag.h_fro = norm(H, 'fro');
    if isempty(H)
        qpDiag.h_min_diag = NaN;
        qpDiag.h_max_diag = NaN;
    else
        d = diag(H);
        qpDiag.h_min_diag = min(d);
        qpDiag.h_max_diag = max(d);
    end

    qpDiag.g_norm = norm(g);
    qpDiag.Aineq_rows = size(Aineq, 1);
    qpDiag.Aineq_cols = size(Aineq, 2);
    qpDiag.Aeq_rows = size(Aeq, 1);
    qpDiag.Aeq_cols = size(Aeq, 2);

    if isempty(Aineq)
        qpDiag.Aineq_fro = 0;
        qpDiag.bineq_norm = 0;
    else
        qpDiag.Aineq_fro = norm(Aineq, 'fro');
        qpDiag.bineq_norm = norm(bineq);
    end

    if isempty(Aeq)
        qpDiag.Aeq_fro = 0;
        qpDiag.beq_norm = 0;
    else
        qpDiag.Aeq_fro = norm(Aeq, 'fro');
        qpDiag.beq_norm = norm(beq);
    end
end

function s = build_fail_snapshot(iter, tFail, Xt, Ut, Xd, Ud, FSM, H, g, Aineq, bineq, Aeq, beq, qpDiag, Tst_log, Tsw_log, idx, sweepCfg, reason, detail)
    s = empty_fail_snapshot();

    s.iter = iter;
    s.t_fail = tFail;
    s.reason = reason;
    s.detail = detail;
    s.fsm_leg1 = scalar_or_nan(FSM, 1);
    s.com_speed = norm_safe_rows(Xt, 4, 5);
    s.state_norm = norm(Xt);
    s.input_norm = norm(Ut);
    s.Tst = safe_idx(Tst_log, idx);
    s.Tsw = safe_idx(Tsw_log, idx);

    s.h_rcond = qpDiag.h_rcond;
    s.h_fro = qpDiag.h_fro;
    s.h_min_diag = qpDiag.h_min_diag;
    s.h_max_diag = qpDiag.h_max_diag;
    s.g_norm = qpDiag.g_norm;
    s.Aineq_rows = qpDiag.Aineq_rows;
    s.Aineq_cols = qpDiag.Aineq_cols;
    s.Aeq_rows = qpDiag.Aeq_rows;
    s.Aeq_cols = qpDiag.Aeq_cols;
    s.Aineq_fro = qpDiag.Aineq_fro;
    s.Aeq_fro = qpDiag.Aeq_fro;
    s.bineq_norm = qpDiag.bineq_norm;
    s.beq_norm = qpDiag.beq_norm;

    if sweepCfg.save_fail_state_vectors
        s.Xt = Xt;
        s.Ut = Ut;
        s.Xd1 = first_col_or_vector(Xd);
        s.Ud1 = first_col_or_vector(Ud);
        s.FSM = FSM;
    end

    if sweepCfg.save_full_qp_on_failure
        s.H = H;
        s.g = g;
        s.Aineq = Aineq;
        s.bineq = bineq;
        s.Aeq = Aeq;
        s.beq = beq;
    end
end

function s = build_success_snapshot(iter, tNow, Xt, Ut, Xd, Ud, FSM, qpDiag, zval, ineq_margin_min, eq_residual_norm, sweepCfg, Tst_log, Tsw_log)
    s = empty_success_snapshot();

    s.iter = iter;
    s.t = tNow;
    s.fsm_leg1 = scalar_or_nan(FSM, 1);
    s.com_speed = norm_safe_rows(Xt, 4, 5);
    s.state_norm = norm(Xt);
    s.input_norm = norm(Ut);
    s.z_step_norm = norm(zval(1:12));
    s.ineq_margin_min = ineq_margin_min;
    s.eq_residual_norm = eq_residual_norm;
    s.Tst = safe_idx(Tst_log, iter);
    s.Tsw = safe_idx(Tsw_log, iter);

    s.h_rcond = qpDiag.h_rcond;
    s.h_fro = qpDiag.h_fro;
    s.h_min_diag = qpDiag.h_min_diag;
    s.h_max_diag = qpDiag.h_max_diag;
    s.g_norm = qpDiag.g_norm;
    s.Aineq_rows = qpDiag.Aineq_rows;
    s.Aeq_rows = qpDiag.Aeq_rows;

    if sweepCfg.save_success_snapshots
        s.Xt = Xt;
        s.Ut = Ut;
        s.Xd1 = first_col_or_vector(Xd);
        s.Ud1 = first_col_or_vector(Ud);
        s.FSM = FSM;
    end
end

function T = case_results_to_table(caseResults)
    n = numel(caseResults);

    case_idx = zeros(n,1);
    rhoR = zeros(n,1);
    rhoR1 = zeros(n,1);
    rhoR2 = zeros(n,1);
    rhoR3 = zeros(n,1);
    rhoR_mean = zeros(n,1);
    rhoR_min = zeros(n,1);
    rhoR_max = zeros(n,1);
    v_cmd = zeros(n,1);
    a_cmd = zeros(n,1);
    feasible = false(n,1);
    fail_reason = strings(n,1);
    fail_iter = nan(n,1);
    fail_time_s = nan(n,1);
    quadprog_exitflag = nan(n,1);
    sim_time_completed_s = nan(n,1);
    tracking_error_total = nan(n,1);
    tracking_error_mean = nan(n,1);
    control_effort_total = nan(n,1);
    control_effort_mean = nan(n,1);
    state_norm_start = nan(n,1);
    state_norm_end = nan(n,1);
    state_norm_max = nan(n,1);
    input_norm_max = nan(n,1);
    com_speed_start = nan(n,1);
    com_speed_end = nan(n,1);
    com_speed_max = nan(n,1);
    Tstance_mean = nan(n,1);
    Tswing_mean = nan(n,1);
    Tcycle_mean = nan(n,1);
    charge_total_As = nan(n,1);
    Ieq_A = nan(n,1);
    total_I_peak = nan(n,1);
    soc_end_pct = nan(n,1);
    battery_margin_norm = nan(n,1);
    elapsed_sec = nan(n,1);
    fail_fsm_leg1 = nan(n,1);
    fail_com_speed = nan(n,1);
    fail_state_norm = nan(n,1);
    fail_input_norm = nan(n,1);
    fail_h_rcond = nan(n,1);
    fail_Aineq_rows = nan(n,1);
    fail_Aeq_rows = nan(n,1);
    root_cause_inference = strings(n,1);

    for i = 1:n
        c = caseResults(i);
        case_idx(i) = c.case_idx;
        rhoR(i) = c.rhoR;
        rhoR1(i) = c.rhoR1;
        rhoR2(i) = c.rhoR2;
        rhoR3(i) = c.rhoR3;
        rhoR_mean(i) = c.rhoR_mean;
        rhoR_min(i) = c.rhoR_min;
        rhoR_max(i) = c.rhoR_max;
        v_cmd(i) = c.v_cmd;
        a_cmd(i) = c.a_cmd;
        feasible(i) = c.feasible;
        fail_reason(i) = string(c.fail_reason);
        fail_iter(i) = c.fail_iter;
        fail_time_s(i) = c.fail_time_s;
        quadprog_exitflag(i) = c.quadprog_exitflag;
        sim_time_completed_s(i) = c.sim_time_completed_s;
        tracking_error_total(i) = c.tracking_error_total;
        tracking_error_mean(i) = c.tracking_error_mean;
        control_effort_total(i) = c.control_effort_total;
        control_effort_mean(i) = c.control_effort_mean;
        state_norm_start(i) = c.state_norm_start;
        state_norm_end(i) = c.state_norm_end;
        state_norm_max(i) = c.state_norm_max;
        input_norm_max(i) = c.input_norm_max;
        com_speed_start(i) = c.com_speed_start;
        com_speed_end(i) = c.com_speed_end;
        com_speed_max(i) = c.com_speed_max;
        Tstance_mean(i) = c.Tstance_mean;
        Tswing_mean(i) = c.Tswing_mean;
        Tcycle_mean(i) = c.Tcycle_mean;
        charge_total_As(i) = c.charge_total_As;
        Ieq_A(i) = c.Ieq_A;
        total_I_peak(i) = c.total_I_peak;
        soc_end_pct(i) = c.soc_end_pct;
        battery_margin_norm(i) = c.battery_margin_norm;
        elapsed_sec(i) = c.elapsed_sec;
        fail_fsm_leg1(i) = c.fail_fsm_leg1;
        fail_com_speed(i) = c.fail_com_speed;
        fail_state_norm(i) = c.fail_state_norm;
        fail_input_norm(i) = c.fail_input_norm;
        fail_h_rcond(i) = c.fail_h_rcond;
        fail_Aineq_rows(i) = c.fail_Aineq_rows;
        fail_Aeq_rows(i) = c.fail_Aeq_rows;
        root_cause_inference(i) = string(c.root_cause_inference);
    end

    T = table( ...
        case_idx, rhoR, rhoR1, rhoR2, rhoR3, rhoR_mean, rhoR_min, rhoR_max, ...
        v_cmd, a_cmd, feasible, fail_reason, root_cause_inference, ...
        fail_iter, fail_time_s, quadprog_exitflag, ...
        sim_time_completed_s, tracking_error_total, tracking_error_mean, control_effort_total, control_effort_mean, ...
        state_norm_start, state_norm_end, state_norm_max, input_norm_max, ...
        com_speed_start, com_speed_end, com_speed_max, ...
        Tstance_mean, Tswing_mean, Tcycle_mean, ...
        charge_total_As, Ieq_A, total_I_peak, soc_end_pct, battery_margin_norm, elapsed_sec, ...
        fail_fsm_leg1, fail_com_speed, fail_state_norm, fail_input_norm, fail_h_rcond, fail_Aineq_rows, fail_Aeq_rows);
end

function T = summarize_by_rho(caseTable)
    rhoVals = unique(caseTable.rhoR);
    n = numel(rhoVals);

    rhoR = zeros(n,1);
    total_cases = zeros(n,1);
    feasible_cases = zeros(n,1);
    infeasible_cases = zeros(n,1);
    feasible_ratio = zeros(n,1);
    mean_tracking_error = nan(n,1);
    mean_control_effort = nan(n,1);
    mean_Ieq = nan(n,1);
    mean_soc_end = nan(n,1);
    mean_fail_time = nan(n,1);

    for i = 1:n
        rho = rhoVals(i);
        mask = caseTable.rhoR == rho;

        rhoR(i) = rho;
        total_cases(i) = sum(mask);
        feasible_cases(i) = sum(mask & caseTable.feasible);
        infeasible_cases(i) = sum(mask & ~caseTable.feasible);
        feasible_ratio(i) = feasible_cases(i) / max(total_cases(i), 1);

        mean_tracking_error(i) = safe_mean(caseTable.tracking_error_mean(mask & caseTable.feasible));
        mean_control_effort(i) = safe_mean(caseTable.control_effort_mean(mask & caseTable.feasible));
        mean_Ieq(i) = safe_mean(caseTable.Ieq_A(mask & caseTable.feasible));
        mean_soc_end(i) = safe_mean(caseTable.soc_end_pct(mask & caseTable.feasible));
        mean_fail_time(i) = safe_mean(caseTable.fail_time_s(mask & ~caseTable.feasible));
    end

    T = table(rhoR, total_cases, feasible_cases, infeasible_cases, feasible_ratio, ...
        mean_tracking_error, mean_control_effort, mean_Ieq, mean_soc_end, mean_fail_time);
end

function T = summarize_by_va(caseTable)
    [G, v_vals, a_vals] = findgroups(caseTable.v_cmd, caseTable.a_cmd);
    n = max(G);

    v_cmd = zeros(n,1);
    a_cmd = zeros(n,1);
    total_cases = zeros(n,1);
    feasible_cases = zeros(n,1);
    infeasible_cases = zeros(n,1);
    max_feasible_rho = nan(n,1);
    min_infeasible_rho = nan(n,1);
    threshold_detected = false(n,1);
    mean_fail_time = nan(n,1);

    for i = 1:n
        mask = G == i;
        Tsub = sortrows(caseTable(mask,:), 'rhoR');

        v_cmd(i) = v_vals(i);
        a_cmd(i) = a_vals(i);
        total_cases(i) = height(Tsub);
        feasible_cases(i) = sum(Tsub.feasible);
        infeasible_cases(i) = sum(~Tsub.feasible);

        rhoFeas = Tsub.rhoR(Tsub.feasible);
        rhoInfeas = Tsub.rhoR(~Tsub.feasible);

        if ~isempty(rhoFeas)
            max_feasible_rho(i) = max(rhoFeas);
        end
        if ~isempty(rhoInfeas)
            min_infeasible_rho(i) = min(rhoInfeas);
        end

        threshold_detected(i) = ~isempty(rhoFeas) && ~isempty(rhoInfeas) && max(rhoFeas) < min(rhoInfeas);
        mean_fail_time(i) = safe_mean(Tsub.fail_time_s(~Tsub.feasible));
    end

    T = table(v_cmd, a_cmd, total_cases, feasible_cases, infeasible_cases, ...
        max_feasible_rho, min_infeasible_rho, threshold_detected, mean_fail_time);
end

function c = empty_case_result()
    c = struct( ...
        'case_idx', NaN, ...
        'run_stamp', '', ...
        'run_dir', '', ...
        'rhoR', NaN, ...
        'v_cmd', NaN, ...
        'a_cmd', NaN, ...
        'chunk_duration', NaN, ...
        'dt_sim', NaN, ...
        'max_iter', NaN, ...
        'nominal_R1', NaN, ...
        'nominal_R2', NaN, ...
        'nominal_R3', NaN, ...
        'used_R1', NaN, ...
        'used_R2', NaN, ...
        'used_R3', NaN, ...
        'feasible', false, ...
        'fail_reason', '', ...
        'fail_reason_detail', '', ...
        'fail_iter', NaN, ...
        'fail_time_s', NaN, ...
        'quadprog_exitflag', NaN, ...
        'exception_id', '', ...
        'exception_message', '', ...
        'elapsed_sec', NaN, ...
        'sim_time_completed_s', NaN, ...
        'completed_iterations', NaN, ...
        'tracking_error_total', NaN, ...
        'tracking_error_mean', NaN, ...
        'control_effort_total', NaN, ...
        'control_effort_mean', NaN, ...
        'state_norm_start', NaN, ...
        'state_norm_end', NaN, ...
        'state_norm_max', NaN, ...
        'state_norm_mean', NaN, ...
        'input_norm_max', NaN, ...
        'input_norm_mean', NaN, ...
        'com_speed_start', NaN, ...
        'com_speed_end', NaN, ...
        'com_speed_max', NaN, ...
        'Tstance_mean', NaN, ...
        'Tswing_mean', NaN, ...
        'Tcycle_mean', NaN, ...
        'knee_tau_mean', NaN, ...
        'knee_I_mean', NaN, ...
        'total_I_peak', NaN, ...
        'charge_total_As', NaN, ...
        'current_duration_s', NaN, ...
        'Ieq_A', NaN, ...
        'soc_end_pct', NaN, ...
        'battery_margin_norm', NaN, ...
        'current_trace_time', [], ...
        'current_trace_total', [], ...
        'current_trace_knee', [], ...
        'iteration_logs', struct([]), ...
        'fail_snapshot', struct([]), ...
        'last_success_snapshot', struct([]), ...
        'causality_scope', '', ...
        'root_cause_inference', '', ...
        'fail_fsm_leg1', NaN, ...
        'fail_com_speed', NaN, ...
        'fail_state_norm', NaN, ...
        'fail_input_norm', NaN, ...
        'fail_h_rcond', NaN, ...
        'fail_Aineq_rows', NaN, ...
        'fail_Aineq_cols', NaN, ...
        'fail_Aeq_rows', NaN, ...
        'fail_Aeq_cols', NaN, ...
        'rhoR1', NaN, ...
        'rhoR2', NaN, ...
        'rhoR3', NaN, ...
        'rhoR_mean', NaN, ...
        'rhoR_min', NaN, ...
        'rhoR_max', NaN);
end

function s = empty_iter_log()
    s = struct( ...
        'iter', [], ...
        't_start', [], ...
        't_end', [], ...
        'tracking_error', [], ...
        'control_effort', [], ...
        'state_norm', [], ...
        'input_norm', [], ...
        'com_speed', [], ...
        'qp_exitflag', [], ...
        'fsm_leg1', [], ...
        'Tst', [], ...
        'Tsw', [], ...
        'tau4', [], ...
        'I4', [], ...
        'ineq_margin_min', [], ...
        'eq_residual_norm', [], ...
        'h_rcond', [], ...
        'h_fro', [], ...
        'h_min_diag', [], ...
        'h_max_diag', [], ...
        'Aineq_rows', [], ...
        'Aeq_rows', [], ...
        'g_norm', [], ...
        'bineq_norm', [], ...
        'beq_norm', [], ...
        'z_step_norm', []);
end

function s = empty_fail_snapshot()
    s = struct( ...
        'iter', NaN, ...
        't_fail', NaN, ...
        'reason', '', ...
        'detail', '', ...
        'fsm_leg1', NaN, ...
        'com_speed', NaN, ...
        'state_norm', NaN, ...
        'input_norm', NaN, ...
        'Tst', NaN, ...
        'Tsw', NaN, ...
        'h_rcond', NaN, ...
        'h_fro', NaN, ...
        'h_min_diag', NaN, ...
        'h_max_diag', NaN, ...
        'g_norm', NaN, ...
        'Aineq_rows', NaN, ...
        'Aineq_cols', NaN, ...
        'Aeq_rows', NaN, ...
        'Aeq_cols', NaN, ...
        'Aineq_fro', NaN, ...
        'Aeq_fro', NaN, ...
        'bineq_norm', NaN, ...
        'beq_norm', NaN, ...
        'Xt', [], ...
        'Ut', [], ...
        'Xd1', [], ...
        'Ud1', [], ...
        'FSM', [], ...
        'H', [], ...
        'g', [], ...
        'Aineq', [], ...
        'bineq', [], ...
        'Aeq', [], ...
        'beq', [], ...
        'Xt_after_invalid', [], ...
        'Xdot_before', []);
end

function s = empty_success_snapshot()
    s = struct( ...
        'iter', NaN, ...
        't', NaN, ...
        'fsm_leg1', NaN, ...
        'com_speed', NaN, ...
        'state_norm', NaN, ...
        'input_norm', NaN, ...
        'z_step_norm', NaN, ...
        'ineq_margin_min', NaN, ...
        'eq_residual_norm', NaN, ...
        'Tst', NaN, ...
        'Tsw', NaN, ...
        'h_rcond', NaN, ...
        'h_fro', NaN, ...
        'h_min_diag', NaN, ...
        'h_max_diag', NaN, ...
        'g_norm', NaN, ...
        'Aineq_rows', NaN, ...
        'Aeq_rows', NaN, ...
        'Xt', [], ...
        'Ut', [], ...
        'Xd1', [], ...
        'Ud1', [], ...
        'FSM', []);
end

function q = empty_qp_diag()
    q = struct( ...
        'h_rcond', NaN, ...
        'h_fro', NaN, ...
        'h_min_diag', NaN, ...
        'h_max_diag', NaN, ...
        'g_norm', NaN, ...
        'Aineq_rows', NaN, ...
        'Aineq_cols', NaN, ...
        'Aeq_rows', NaN, ...
        'Aeq_cols', NaN, ...
        'Aineq_fro', NaN, ...
        'Aeq_fro', NaN, ...
        'bineq_norm', NaN, ...
        'beq_norm', NaN);
end

function m = safe_mean(x)
    x = x(isfinite(x));
    if isempty(x)
        m = NaN;
    else
        m = mean(x);
    end
end

function add_if_exists(folderPath)
    if exist(folderPath, 'dir')
        addpath(folderPath);
    end
end

function log_msg(fid, sweepCfg, msg)
    line = sprintf('[%s] %s', datestr(now, 'yyyy-mm-dd HH:MM:SS'), msg);
    fprintf(fid, '%s\n', line);
    if sweepCfg.print_to_cli
        fprintf('%s\n', line);
    end
end

function s = format_case_log_line(caseResult, nCases)
    if caseResult.feasible
        s = sprintf(['CASE %d/%d | rho=[%.3f %.3f %.3f] | v=%.3f | a=%.3f | feasible=1 | ' ...
            'track=%.3f | effort=%.3f | Ieq=%.3f | SOC_end=%.2f | elapsed=%.2f s'], ...
            caseResult.case_idx, nCases, caseResult.rhoR1, caseResult.rhoR2, caseResult.rhoR3, ...
            caseResult.v_cmd, caseResult.a_cmd, ...
            caseResult.tracking_error_mean, caseResult.control_effort_mean, ...
            caseResult.Ieq_A, caseResult.soc_end_pct, caseResult.elapsed_sec);
    else
        s = sprintf(['CASE %d/%d | rho=[%.3f %.3f %.3f] | v=%.3f | a=%.3f | feasible=0 | ' ...
            'reason=%s | root=%s | fail_iter=%g | fail_t=%.3f | exitflag=%g | elapsed=%.2f s'], ...
            caseResult.case_idx, nCases, caseResult.rhoR1, caseResult.rhoR2, caseResult.rhoR3, ...
            caseResult.v_cmd, caseResult.a_cmd, ...
            caseResult.fail_reason, caseResult.root_cause_inference, caseResult.fail_iter, caseResult.fail_time_s, ...
            caseResult.quadprog_exitflag, caseResult.elapsed_sec);
    end
end

function write_final_summary(fid, sweepCfg, Results)
    T = Results.case_table;

    feasibleCount = sum(T.feasible);
    infeasibleCount = sum(~T.feasible);

    log_msg(fid, sweepCfg, sprintf('Total cases = %d', height(T)));
    log_msg(fid, sweepCfg, sprintf('Feasible cases = %d', feasibleCount));
    log_msg(fid, sweepCfg, sprintf('Infeasible cases = %d', infeasibleCount));
    log_msg(fid, sweepCfg, sprintf('Feasible ratio = %.4f', feasibleCount / max(height(T), 1)));

    failReasons = categories(categorical(T.fail_reason(~T.feasible)));
    for i = 1:numel(failReasons)
        r = failReasons{i};
        count = sum(strcmp(T.fail_reason, r));
        log_msg(fid, sweepCfg, sprintf('Fail reason [%s] count = %d', r, count));
    end

    if feasibleCount > 0
        log_msg(fid, sweepCfg, sprintf('Feasible mean tracking error = %.4f', safe_mean(T.tracking_error_mean(T.feasible))));
        log_msg(fid, sweepCfg, sprintf('Feasible mean control effort = %.4f', safe_mean(T.control_effort_mean(T.feasible))));
        log_msg(fid, sweepCfg, sprintf('Feasible mean Ieq = %.4f A', safe_mean(T.Ieq_A(T.feasible))));
        log_msg(fid, sweepCfg, sprintf('Feasible mean SOC_end = %.4f %%', safe_mean(T.soc_end_pct(T.feasible))));
    end
end

function y = safe_rcond(H)
    try
        y = rcond(H);
    catch
        y = NaN;
    end
end

function y = scalar_or_nan(x, idx)
    try
        y = x(idx);
    catch
        y = NaN;
    end
end

function y = norm_safe_rows(x, i1, i2)
    try
        y = norm(x(i1:i2));
    catch
        y = NaN;
    end
end

function y = safe_idx(x, idx)
    try
        y = x(idx);
    catch
        y = NaN;
    end
end

function y = first_col_or_vector(x)
    if isempty(x)
        y = [];
    elseif isvector(x)
        y = x(:);
    else
        y = x(:,1);
    end
end

function y = nan_safe_col(x)
    if isempty(x)
        y = [];
    else
        y = nan(size(x(:)));
    end
end