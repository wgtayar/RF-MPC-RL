function result = run_solver_continuation_ab(runDir, outputFolder, caseIds, continuationSeconds)
%run_solver_continuation_ab Replay and continue archived solver failures.

    if nargin < 1 || strlength(string(runDir)) == 0
        error('run_solver_continuation_ab:RunDirectoryRequired', ...
            'Provide the archived Phase-1 run directory.');
    end
    if nargin < 2 || strlength(string(outputFolder)) == 0
        stamp = char(datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss'));
        outputFolder = fullfile(fileparts(mfilename('fullpath')), ...
            'Phase 2 Outputs', ['solver_ab_' stamp]);
    end
    if nargin < 3 || isempty(caseIds)
        caseIds = [ ...
            "ep0010_dec04_chunk01_iter0046"
            "ep0014_dec05_chunk01_iter0246"
            "ep0040_dec08_chunk01_iter0279"];
    end
    if nargin < 4 || isempty(continuationSeconds)
        continuationSeconds = 10;
    end
    bootstrap_RF_MPC_RL();
    runDir = string(runDir);
    outputFolder = string(outputFolder);
    if ~isfolder(outputFolder)
        mkdir(outputFolder);
    end

    cfgFile = localSingleFile(runDir, 'cfg_snapshot_*.mat');
    cfgData = load(cfgFile, 'cfg');
    cfg = cfgData.cfg;
    chunkFile = localSingleFile(runDir, 'rl_chunks_*.csv');
    chunks = readtable(chunkFile, TextType='string');
    chunks = sortrows(chunks, {'episode_idx','global_chunk_idx'});
    rows = repmat(localEmptySummary(), numel(caseIds), 1);
    caseResults = cell(numel(caseIds), 1);

    for caseIndex = 1:numel(caseIds)
        caseId = string(caseIds(caseIndex));
        qpFile = fullfile(runDir, 'qp_failures', caseId + ".mat");
        archived = load(qpFile, 'failureQP');
        qp = archived.failureQP;
        fprintf('[solver A/B] %s: replaying prefix to %.3f s\n', ...
            caseId, qp.fail_time_s);

        [branchState, prefixTrace, prefixFallbackCount] = ...
            localReplayPrefix(chunks, qp, cfg);
        branchState.current_time = [];
        branchState.current_total = [];
        branchState.battery.soc_pct = localChunkValue( ...
            chunks, qp, 'soc_start_pct', branchState.battery.soc_pct);
        branchState.battery.margin_norm = branchState.battery.soc_pct/100;
        branchState.battery.metric_value = branchState.battery.soc_pct;

        p = get_params(branchState.gait);
        oneStep = p.simTimeStep;
        probeOptions = struct('duration_s', oneStep, ...
            'solver_strategy', "default", 'capture_trace', true, ...
            'capture_first_problem', true, 'update_proxy', true, ...
            'update_battery', false, 'classify_failure', true);
        [~, probeOut] = simulate_mpc_horizon(branchState, ...
            localControl(qp), cfg, probeOptions);
        prefixComparison = localCompareProblem(probeOut.first_problem, qp);

        branchState.Xt = qp.Xt;
        branchState.Ut = qp.Ut;
        branchState.t = qp.fail_time_s;
        branchState.fsm_internal_state = ...
            probeOut.first_problem.fsm_internal_state;
        common = struct('duration_s', continuationSeconds, ...
            'capture_trace', true, 'capture_first_problem', true, ...
            'initial_problem_override', qp, 'update_proxy', true, ...
            'update_battery', true, 'classify_failure', true);
        aOptions = common;
        aOptions.solver_strategy = "default";
        [stateA, outA] = simulate_mpc_horizon(branchState, ...
            localControl(qp), cfg, aOptions);

        bOptions = common;
        bOptions.solver_strategy = "active_set_feasible_point";
        [stateB, outB] = simulate_mpc_horizon(branchState, ...
            localControl(qp), cfg, bOptions);

        comparisonA = localCompareProblem(outA.first_problem, qp);
        comparisonB = localCompareProblem(outB.first_problem, qp);
        exactBranchProblem = comparisonA.max_relative_error <= 1e-12 && ...
            comparisonB.max_relative_error <= 1e-12;
        hiddenStateValidated = ...
            prefixComparison.FSM_relative_error == 0 && ...
            prefixComparison.Xt_relative_error <= 1e-3 && ...
            prefixComparison.Ut_relative_error <= 1e-2 && ...
            prefixComparison.Xd_relative_error <= 1e-4;
        outcome = localClassify(outA, outB, continuationSeconds);
        if ~exactBranchProblem || ~hiddenStateValidated
            outcome = "ambiguous";
        end

        caseFolder = fullfile(outputFolder, caseId);
        if ~isfolder(caseFolder)
            mkdir(caseFolder);
        end
        writetable(prefixTrace, fullfile(caseFolder, 'prefix_trace.csv'));
        writetable(outA.trace, fullfile(caseFolder, 'branch_A_default.csv'));
        writetable(outB.trace, fullfile(caseFolder, 'branch_B_active_set.csv'));
        save(fullfile(caseFolder, 'continuation_evidence.mat'), ...
            'branchState', 'stateA', 'stateB', 'outA', 'outB', ...
            'probeOut', 'prefixComparison', 'comparisonA', ...
            'comparisonB', 'qp', '-v7.3');

        rows(caseIndex) = localSummary(caseId, qp, exactBranchProblem, ...
            hiddenStateValidated, prefixFallbackCount, prefixComparison, ...
            outA, outB, outcome);
        caseResults{caseIndex} = struct('case_id', caseId, ...
            'exact_branch_problem', exactBranchProblem, ...
            'hidden_state_validated', hiddenStateValidated, ...
            'prefix_fallback_count', prefixFallbackCount, ...
            'prefix_comparison', prefixComparison, ...
            'comparison_A', comparisonA, 'comparison_B', comparisonB, ...
            'branch_A', outA, 'branch_B', outB, 'outcome', outcome);
        fprintf('[solver A/B] %s: branch_exact=%d, A=%.3f s, B=%.3f s, outcome=%s\n', ...
            caseId, exactBranchProblem, outA.survived_duration_s, ...
            outB.survived_duration_s, outcome);
    end

    summary = struct2table(rows);
    writetable(summary, fullfile(outputFolder, 'solver_ab_summary.csv'));
    provenance = struct('source_git_sha', localGitSha(), ...
        'phase1_run_dir', runDir, 'case_ids', caseIds, ...
        'continuation_seconds', continuationSeconds, ...
        'matlab_version', version, 'created_at', ...
        string(datetime('now', 'TimeZone', 'local')));
    save(fullfile(outputFolder, 'solver_ab_results.mat'), ...
        'summary', 'caseResults', 'provenance', '-v7.3');
    result = struct('summary', summary, 'case_results', {caseResults}, ...
        'provenance', provenance, 'output_folder', outputFolder);
end

function [state, trace, fallbackCount] = localReplayPrefix(chunks, qp, cfg)
    state = initialize_mpc_replay_state(cfg, 0);
    episodeRows = chunks(chunks.episode_idx == qp.context.episode_idx, :);
    targetMask = episodeRows.decision_idx == qp.context.decision_idx & ...
        episodeRows.chunk_in_decision == qp.context.chunk_in_decision;
    targetIndex = find(targetMask, 1);
    if isempty(targetIndex)
        error('run_solver_continuation_ab:MissingTargetChunk', ...
            'No chunk row found for %s.', qp.failure_qp_id);
    end
    traceParts = cell(targetIndex, 1);
    fallbackCount = 0;
    p = get_params(0);
    for rowIndex = 1:targetIndex
        if rowIndex < targetIndex
            duration = cfg.CHUNK_DURATION;
        else
            duration = (qp.fail_iter - 1)*p.simTimeStep;
        end
        row = episodeRows(rowIndex, :);
        options = struct('duration_s', duration, ...
            'solver_strategy', "default", ...
            'capture_trace', state.t + duration >= qp.fail_time_s - 2, ...
            'trace_start_time_s', qp.fail_time_s - 2, ...
            'classify_failure', true, 'update_proxy', true, ...
            'update_battery', false);
        [state, traceParts{rowIndex}, usedFallbacks] = ...
            localAdvancePrefix(state, localControlFromRow(row), ...
            cfg, options);
        fallbackCount = fallbackCount + usedFallbacks;
    end
    trace = vertcat(traceParts{:});
end

function [state, trace, fallbackCount] = localAdvancePrefix( ...
        state, control, cfg, options)
    segmentStart = state.t;
    requestedDuration = options.duration_s;
    p = get_params(state.gait);
    traceParts = cell(0, 1);
    fallbackCount = 0;
    while state.t < segmentStart + requestedDuration - p.simTimeStep/2
        options.duration_s = segmentStart + requestedDuration - state.t;
        [state, out] = simulate_mpc_horizon(state, control, cfg, options);
        traceParts{end+1, 1} = out.trace;
        if out.completed_horizon
            break
        end
        if out.terminal_reason ~= "numerical_solver_failure" || ...
                isempty(fieldnames(out.failure_problem))
            error('run_solver_continuation_ab:PrefixFailure', ...
                'Prefix replay failed at %.3f s with %s.', ...
                state.t, out.terminal_reason);
        end

        rescueOptions = options;
        rescueOptions.duration_s = p.simTimeStep;
        rescueOptions.solver_strategy = "active_set_feasible_point";
        rescueOptions.initial_problem_override = out.failure_problem;
        rescueOptions.capture_first_problem = true;
        rescueOptions.update_proxy = false;
        [state, rescue] = simulate_mpc_horizon( ...
            state, control, cfg, rescueOptions);
        traceParts{end+1, 1} = rescue.trace;
        if ~rescue.completed_horizon
            error('run_solver_continuation_ab:PrefixRescueFailure', ...
                'Prefix active-set rescue failed at %.3f s with %s.', ...
                state.t, rescue.terminal_reason);
        end
        fallbackCount = fallbackCount + 1;
        options.initial_problem_override = struct();
    end
    trace = vertcat(traceParts{:});
end

function control = localControl(qp)
    rWeights = diag(qp.R);
    control = struct('R', rWeights(1:3), 'v_cmd', qp.v_cmd, ...
        'a_cmd', qp.a_cmd);
    if isfield(qp, 'context')
        context = qp.context;
        control.action = [context.dR_frac_applied(:); ...
            context.gamma_v_applied; context.gamma_a_applied];
    end
end

function control = localControlFromRow(row)
    control = struct('R', [row.R1; row.R2; row.R3], ...
        'v_cmd', row.v_exec, 'a_cmd', row.a_exec, ...
        'action', [row.dR1; row.dR2; row.dR3; row.gamma_v; row.gamma_a]);
end

function comparison = localCompareProblem(actual, expected)
    fields = {'Xt','Ut','Xd','Ud','H','g','Aineq','bineq','Aeq','beq','FSM'};
    comparison = struct();
    maximum = 0;
    for i = 1:numel(fields)
        field = fields{i};
        if ~isfield(actual, field) || ~isfield(expected, field) || ...
                ~isequal(size(actual.(field)), size(expected.(field)))
            relativeError = inf;
        else
            relativeError = norm(actual.(field)(:) - expected.(field)(:)) / ...
                max(norm(expected.(field)(:)), 1);
        end
        comparison.([field '_relative_error']) = relativeError;
        maximum = max(maximum, relativeError);
    end
    comparison.max_relative_error = maximum;
end

function outcome = localClassify(outA, outB, requestedDuration)
    if isempty(outB.trace) || outB.trace.solver_exitflag(1) <= 0
        outcome = "no_solver_rescue";
        return
    end
    if outA.survived_duration_s >= requestedDuration - 1e-9
        outcome = "ambiguous";
        return
    end
    if outB.survived_duration_s < requestedDuration - 1e-9
        if outB.survived_duration_s >= 0.5
            outcome = "solver_rescue_delays_failure";
        else
            outcome = "solver_rescue_but_state_still_collapses";
        end
        return
    end
    finalIndex = find(isfinite(outB.trace.orientation_error_after_rad), 1, 'last');
    if isempty(finalIndex)
        outcome = "solver_rescue_but_state_still_collapses";
        return
    end
    final = outB.trace(finalIndex, :);
    healthy = final.orientation_error_after_rad <= 1.0 && ...
        final.angular_velocity_after <= 10 && ...
        final.position_invariant_norm_after <= 10 && ...
        final.Ut_norm_after <= 100;
    if healthy
        outcome = "solver_rescue_and_recovery";
    elseif outA.qp_failed_count > 0
        outcome = "solver_rescue_but_state_still_collapses";
    else
        outcome = "ambiguous";
    end
end

function row = localEmptySummary()
    row = struct('case_id', "", 'episode', NaN, 'decision', NaN, ...
        'chunk', NaN, 'failure_time_s', NaN, ...
        'exact_branch_problem', false, ...
        'hidden_state_reconstruction_validated', false, ...
        'prefix_active_set_fallback_count', NaN, ...
        'prefix_max_qp_relative_error', NaN, ...
        'prefix_Xt_relative_error', NaN, 'prefix_Ut_relative_error', NaN, ...
        'default_exitflag', NaN, 'default_continuation_survival_s', NaN, ...
        'default_subsequent_solver_failures', NaN, ...
        'active_set_immediate_exitflag', NaN, ...
        'active_set_immediate_iterations', NaN, ...
        'active_set_immediate_objective', NaN, ...
        'active_set_immediate_kkt_inf', NaN, ...
        'active_set_immediate_margin_min', NaN, ...
        'continuation_survival_s', NaN, 'subsequent_solver_failures', NaN, ...
        'final_orientation_error_rad', NaN, ...
        'final_angular_velocity', NaN, 'final_com_velocity', NaN, ...
        'final_position_invariant_norm', NaN, 'final_Ut_norm', NaN, ...
        'continuation_Ieq_A', NaN, 'outcome', "");
end

function row = localSummary(caseId, qp, exactBranch, hiddenStateValidated, ...
        prefixFallbackCount, prefixComparison, outA, outB, outcome)
    row = localEmptySummary();
    row.case_id = caseId;
    row.episode = qp.context.episode_idx;
    row.decision = qp.context.decision_idx;
    row.chunk = qp.context.chunk_in_decision;
    row.failure_time_s = qp.fail_time_s;
    row.exact_branch_problem = exactBranch;
    row.hidden_state_reconstruction_validated = hiddenStateValidated;
    row.prefix_active_set_fallback_count = prefixFallbackCount;
    row.prefix_max_qp_relative_error = prefixComparison.max_relative_error;
    row.prefix_Xt_relative_error = prefixComparison.Xt_relative_error;
    row.prefix_Ut_relative_error = prefixComparison.Ut_relative_error;
    row.default_continuation_survival_s = outA.survived_duration_s;
    row.default_subsequent_solver_failures = outA.qp_failed_count;
    if ~isempty(outA.trace)
        row.default_exitflag = outA.trace.solver_exitflag(1);
    end
    if ~isempty(outB.trace)
        first = outB.trace(1, :);
        lastIndex = find(isfinite( ...
            outB.trace.orientation_error_after_rad), 1, 'last');
        if isempty(lastIndex)
            lastIndex = height(outB.trace);
        end
        last = outB.trace(lastIndex, :);
        row.active_set_immediate_exitflag = first.solver_exitflag;
        row.active_set_immediate_iterations = first.solver_iterations;
        row.active_set_immediate_objective = first.objective;
        row.active_set_immediate_kkt_inf = first.kkt_stationarity_inf;
        row.active_set_immediate_margin_min = first.inequality_margin_min;
        row.final_orientation_error_rad = last.orientation_error_after_rad;
        row.final_angular_velocity = last.angular_velocity_after;
        row.final_com_velocity = last.com_velocity_after;
        row.final_position_invariant_norm = last.position_invariant_norm_after;
        row.final_Ut_norm = last.Ut_norm_after;
    end
    row.continuation_survival_s = outB.survived_duration_s;
    row.subsequent_solver_failures = outB.qp_failed_count;
    row.continuation_Ieq_A = outB.Ieq_A;
    row.outcome = outcome;
end

function value = localChunkValue(chunks, qp, field, defaultValue)
    mask = chunks.episode_idx == qp.context.episode_idx & ...
        chunks.decision_idx == qp.context.decision_idx & ...
        chunks.chunk_in_decision == qp.context.chunk_in_decision;
    if any(mask) && ismember(field, chunks.Properties.VariableNames)
        value = chunks.(field)(find(mask, 1));
    else
        value = defaultValue;
    end
end

function file = localSingleFile(folder, pattern)
    files = dir(fullfile(folder, '**', pattern));
    if numel(files) ~= 1
        error('run_solver_continuation_ab:FileCount', ...
            'Expected one %s under %s; found %d.', ...
            pattern, folder, numel(files));
    end
    file = string(fullfile(files.folder, files.name));
end

function sha = localGitSha()
    [status, text] = system('git rev-parse HEAD');
    if status == 0
        sha = string(strtrim(text));
    else
        sha = "unknown";
    end
end
