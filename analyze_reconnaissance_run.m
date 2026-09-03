function summary = analyze_reconnaissance_run(runDir)
%analyze_reconnaissance_run Analyze one instrumented reconnaissance run.

    arguments
        runDir (1,1) string {mustBeFolder}
    end
    bootstrap_RF_MPC_RL();

    analysisDir = fullfile(runDir, 'analysis');
    if ~isfolder(analysisDir)
        mkdir(analysisDir);
    end

    decisionFile = localSingleFile(runDir, 'rl_decisions_*.csv');
    chunkFile = localSingleFile(runDir, 'rl_chunks_*.csv');
    decisions = readtable(decisionFile, TextType='string');
    chunks = readtable(chunkFile, TextType='string');
    decisions = sortrows(decisions, {'episode_idx','decision_idx'});
    chunks = sortrows(chunks, ...
        {'episode_idx','decision_idx','chunk_in_decision'});

    terminalRows = decisions([diff(decisions.episode_idx) ~= 0; true], :);
    completionMask = terminalRows.terminal_reason == "mission_complete";
    infeasibleMask = terminalRows.terminal_reason == "infeasible";
    batteryMask = terminalRows.terminal_reason == "battery_terminal";
    timeLimitMask = terminalRows.terminal_reason == "time_limit";

    summary = struct();
    summary.run_id = char(extractAfter(runDir, fileparts(runDir) + filesep));
    summary.episode_count = height(terminalRows);
    summary.mission_complete_count = nnz(completionMask);
    summary.mission_complete_rate = mean(completionMask);
    summary.infeasible_count = nnz(infeasibleMask);
    summary.infeasible_rate = mean(infeasibleMask);
    summary.battery_terminal_count = nnz(batteryMask);
    summary.time_limit_count = nnz(timeLimitMask);
    summary.first_mission_completion_episode = localFirstOrNaN( ...
        terminalRows.episode_idx(completionMask));
    summary.best_distance_m = max(terminalRows.distance_end_m, [], 'omitmissing');
    summary.mean_final_distance_m = mean(terminalRows.distance_end_m, 'omitmissing');
    summary.median_final_distance_m = median(terminalRows.distance_end_m, 'omitmissing');
    summary.mean_final_soc_pct = mean(terminalRows.soc_end_pct, 'omitmissing');
    summary.minimum_final_soc_pct = min(terminalRows.soc_end_pct, [], 'omitmissing');
    crossingRows = decisions(decisions.target_crossed_this_window == 1, :);
    summary.target_crossing_count = height(crossingRows);
    summary.best_target_cross_time_s = localMinOrNaN(crossingRows.target_cross_time_s);
    summary.mean_soc_at_target_pct = localMeanOrNaN(crossingRows.target_cross_soc_pct);
    summary.total_decisions = height(decisions);
    summary.total_logged_chunks = height(chunks);
    summary.total_qp_solves = sum(chunks.qp_solve_count, 'omitmissing');
    summary.failed_qp_count = sum(chunks.qp_failed_count, 'omitmissing');
    summary.qp_failure_rate = summary.failed_qp_count / summary.total_qp_solves;
    summary.first_chunk_failure_count = nnz( ...
        chunks.feasible == 0 & chunks.chunk_in_decision == 1);
    summary.first_chunk_failure_rate = ...
        summary.first_chunk_failure_count / max(summary.infeasible_count, 1);
    summary.historic_decision_4_to_6_chunk_1_to_2_count = nnz( ...
        chunks.feasible == 0 & isbetween(chunks.decision_idx, 4, 6) & ...
        isbetween(chunks.chunk_in_decision, 1, 2));

    manifestFile = localSingleFile(runDir, 'run_manifest_*.txt');
    manifest = fileread(manifestFile);
    summary.wall_clock_seconds = localManifestNumber(manifest, 'wall_clock_seconds');
    summary.seconds_per_episode = summary.wall_clock_seconds / summary.episode_count;
    summary.seconds_per_decision = summary.wall_clock_seconds / summary.total_decisions;
    summary.seconds_per_logged_chunk = ...
        summary.wall_clock_seconds / summary.total_logged_chunks;
    summary.seconds_per_qp_solve = ...
        summary.wall_clock_seconds / summary.total_qp_solves;
    summary.artifact_size_bytes = localFolderBytes(runDir);

    episodeOutcomes = terminalRows(:, intersect( ...
        {'episode_idx','decision_idx','terminal_reason','fail_reason', ...
        'distance_end_m','soc_end_pct','target_cross_time_s', ...
        'target_cross_soc_pct','reward','failure_qp_id'}, ...
        terminalRows.Properties.VariableNames, 'stable'));
    writetable(episodeOutcomes, fullfile(analysisDir, 'episode_outcomes.csv'));

    failedChunks = chunks(chunks.feasible == 0, :);
    byDecision = groupcounts(failedChunks, 'decision_idx');
    byChunk = groupcounts(failedChunks, 'chunk_in_decision');
    failedChunks.contact_configuration = localContactKey(failedChunks);
    byContact = groupcounts(failedChunks, 'contact_configuration');
    writetable(byDecision, fullfile(analysisDir, 'failure_by_decision.csv'));
    writetable(byChunk, fullfile(analysisDir, 'failure_by_chunk.csv'));
    writetable(byContact, fullfile(analysisDir, 'failure_by_contact.csv'));
    writetable(failedChunks(:, intersect( ...
        {'episode_idx','decision_idx','chunk_in_decision','fail_time_s', ...
        'failure_qp_id','fail_reason','v_exec','a_exec','dR1','dR2','dR3', ...
        'R1','R2','R3','soc_start_pct','state_linear_velocity_norm', ...
        'state_orientation_error_rad','state_angular_velocity_norm', ...
        'state_position_invariant_state_norm','input_norm_end', ...
        'inequality_margin_min','equality_residual_max_abs', ...
        'contact_configuration'}, failedChunks.Properties.VariableNames, 'stable')), ...
        fullfile(analysisDir, 'failure_events.csv'));

    matchedTransitions = localMatchSuccessfulChunks(chunks);
    writetable(matchedTransitions, ...
        fullfile(analysisDir, 'matched_successful_transitions.csv'));

    [qpResults, phaseResults] = localAnalyzeFailureQps(runDir);
    writetable(qpResults, fullfile(analysisDir, 'qp_phase1_results.csv'));
    save(fullfile(analysisDir, 'qp_phase1_results.mat'), ...
        'qpResults', 'phaseResults', '-v7.3');

    summary.phase1_feasible_to_tolerance_count = ...
        nnz(qpResults.phase_relax <= 1e-7 & qpResults.phase_max_eq <= 1e-7);
    summary.phase1_mathematically_infeasible_count = nnz( ...
        qpResults.classification == "mathematically_infeasible_linear_constraints");
    summary.active_set_resolve_success_count = nnz(qpResults.active_exit > 0);
    summary.positive_definite_hessian_count = nnz(qpResults.min_eig_H > 0);
    summary.full_row_rank_Aeq_count = nnz(qpResults.rank_Aeq == 72);
    summary.original_exitflag_zero_count = nnz(qpResults.original_exit == 0);
    summary.original_negative_exitflag_count = nnz(qpResults.original_exit < 0);
    summary.maximum_phase1_equality_residual = ...
        max(qpResults.phase_max_eq, [], 'omitmissing');
    summary.maximum_phase1_inequality_relaxation = ...
        max(qpResults.phase_relax, [], 'omitmissing');
    summary.minimum_hessian_rcond = min(qpResults.rcond_H, [], 'omitmissing');
    summary.maximum_hessian_condition_estimate = ...
        max(qpResults.condest_H, [], 'omitmissing');

    save(fullfile(analysisDir, 'run_analysis.mat'), ...
        'summary', 'episodeOutcomes', 'failedChunks', 'matchedTransitions');
    localWriteJson(fullfile(analysisDir, 'run_summary.json'), summary);
end

function matched = localMatchSuccessfulChunks(chunks)
    chunks.distance_end_m = zeros(height(chunks), 1);
    episodes = unique(chunks.episode_idx, 'stable');
    for i = 1:numel(episodes)
        mask = chunks.episode_idx == episodes(i);
        chunks.distance_end_m(mask) = cumsum(chunks.dx_chunk_m(mask));
    end
    chunks.distance_start_m = chunks.distance_end_m - chunks.dx_chunk_m;
    successful = chunks(chunks.feasible == 1, :);
    failed = chunks(chunks.feasible == 0, :);
    n = height(failed);
    matchIndex = zeros(n, 1);
    matchScore = nan(n, 1);
    for i = 1:n
        candidates = find(successful.episode_idx ~= failed.episode_idx(i));
        exactDecision = candidates( ...
            successful.decision_idx(candidates) == failed.decision_idx(i));
        if ~isempty(exactDecision)
            candidates = exactDecision;
        end
        score = localMatchScore(successful(candidates, :), failed(i, :));
        [matchScore(i), localIndex] = min(score);
        matchIndex(i) = candidates(localIndex);
    end
    good = successful(matchIndex, :);
    matched = table( ...
        failed.episode_idx, failed.decision_idx, failed.chunk_in_decision, ...
        failed.failure_qp_id, good.episode_idx, good.decision_idx, ...
        good.chunk_in_decision, matchScore, ...
        failed.soc_start_pct, good.soc_start_pct, ...
        failed.distance_start_m, good.distance_start_m, ...
        failed.v_exec, good.v_exec, failed.a_exec, good.a_exec, ...
        failed.R1, good.R1, failed.R2, good.R2, failed.R3, good.R3, ...
        failed.state_linear_velocity_norm, good.state_linear_velocity_norm, ...
        failed.state_orientation_error_rad, good.state_orientation_error_rad, ...
        failed.state_angular_velocity_norm, good.state_angular_velocity_norm, ...
        failed.state_position_invariant_state_norm, ...
        good.state_position_invariant_state_norm, ...
        failed.input_norm_end, good.input_norm_end, ...
        failed.inequality_margin_min, good.inequality_margin_min, ...
        failed.equality_residual_max_abs, good.equality_residual_max_abs, ...
        'VariableNames', { ...
        'failure_episode','failure_decision','failure_chunk','failure_qp_id', ...
        'match_episode','match_decision','match_chunk','match_score', ...
        'failure_soc','match_soc','failure_distance','match_distance', ...
        'failure_v','match_v','failure_a','match_a', ...
        'failure_R1','match_R1','failure_R2','match_R2','failure_R3','match_R3', ...
        'failure_velocity_norm','match_velocity_norm', ...
        'failure_orientation_error','match_orientation_error', ...
        'failure_angular_velocity_norm','match_angular_velocity_norm', ...
        'failure_position_invariant_norm','match_position_invariant_norm', ...
        'failure_input_norm','match_input_norm', ...
        'failure_min_margin','match_min_margin', ...
        'failure_max_eq_residual','match_max_eq_residual'});
end

function score = localMatchScore(candidates, failure)
    score = 4*((candidates.decision_idx - failure.decision_idx) / 1).^2 + ...
        ((candidates.soc_start_pct - failure.soc_start_pct) / 10).^2 + ...
        ((candidates.distance_start_m - failure.distance_start_m) / 40).^2 + ...
        ((candidates.v_exec - failure.v_exec) / 0.08).^2 + ...
        ((candidates.a_exec - failure.a_exec) / 0.08).^2 + ...
        ((candidates.R1 - failure.R1) / 0.002).^2 + ...
        ((candidates.R2 - failure.R2) / 0.004).^2 + ...
        ((candidates.R3 - failure.R3) / 0.002).^2 + ...
        ((candidates.dR1 - failure.dR1) / 5e-4).^2 + ...
        ((candidates.dR2 - failure.dR2) / 5e-4).^2 + ...
        ((candidates.dR3 - failure.dR3) / 5e-4).^2;
    for leg = 1:4
        field = sprintf('fsm_leg%d_end', leg);
        score = score + 2*(candidates.(field) ~= failure.(field)).^2;
    end
end

function [results, phaseResults] = localAnalyzeFailureQps(runDir)
    files = dir(fullfile(runDir, 'qp_failures', '*.mat'));
    n = numel(files);
    episode = zeros(n,1); decision = zeros(n,1); chunk = zeros(n,1);
    iteration = zeros(n,1); originalExit = zeros(n,1);
    originalIterations = zeros(n,1); phaseExit = zeros(n,1);
    phaseRelax = nan(n,1); phaseEqualityLsq = nan(n,1);
    phaseMaxInequality = nan(n,1); phaseMaxEquality = nan(n,1);
    rcondH = nan(n,1); condestH = nan(n,1); rankAeq = zeros(n,1);
    minEigenvalueH = nan(n,1); maxEigenvalueH = nan(n,1);
    activeExit = zeros(n,1); activeIterations = zeros(n,1);
    activeConstraintViolation = nan(n,1); activeFirstOrder = nan(n,1);
    failureStateNorm = nan(n,1); lastStateNorm = nan(n,1);
    failureInvariantNorm = nan(n,1); lastInvariantNorm = nan(n,1);
    stateDeltaNorm = nan(n,1); carriedInputDeltaNorm = nan(n,1);
    changedFsmLegs = nan(n,1); gNorm = nan(n,1); previousGNorm = nan(n,1);
    beqNorm = nan(n,1); previousBeqNorm = nan(n,1);
    hDeltaNorm = nan(n,1);
    aDeltaNorm = nan(n,1); bDeltaNorm = nan(n,1); dDeltaNorm = nan(n,1);
    phaseForceCorrectionNorm = nan(n,1); phaseSolutionNorm = nan(n,1);
    zeroCorrectionMarginMin = nan(n,1); phaseMarginMin = nan(n,1);
    classification = strings(n,1); originalMessage = strings(n,1);
    file = strings(n,1); phaseResults = cell(n,1);
    activeOptions = optimoptions('quadprog', 'Display', 'off', ...
        'Algorithm', 'active-set', 'MaxIterations', 5000);
    for i = 1:n
        file(i) = string(fullfile(files(i).folder, files(i).name));
        saved = load(file(i), 'failureQP');
        qp = saved.failureQP;
        phase = analyze_qp_feasibility(qp);
        phaseResults{i} = phase;
        episode(i) = qp.context.episode_idx;
        decision(i) = qp.context.decision_idx;
        chunk(i) = qp.context.chunk_in_decision;
        iteration(i) = qp.fail_iter;
        originalExit(i) = qp.exitflag;
        originalIterations(i) = qp.quadprog_output.iterations;
        originalMessage(i) = string(qp.quadprog_output.message);
        phaseExit(i) = phase.phase1_exitflag;
        phaseRelax(i) = phase.minimum_scaled_inequality_relaxation_l1;
        phaseEqualityLsq(i) = phase.minimum_scaled_equality_residual_l2;
        phaseMaxInequality(i) = phase.maximum_inequality_violation;
        phaseMaxEquality(i) = phase.maximum_equality_residual;
        rcondH(i) = qp.rcond_H;
        condestH(i) = qp.condest_H;
        rankAeq(i) = qp.rank_Aeq;
        minEigenvalueH(i) = qp.H_min_eigenvalue;
        maxEigenvalueH(i) = qp.H_max_eigenvalue;
        classification(i) = string(phase.classification);
        failureComponents = decompose_srb_state(qp.Xt, qp.Xd(:,1));
        lastComponents = decompose_srb_state( ...
            qp.last_successful_Xt, qp.last_successful_Xd(:,1));
        failureStateNorm(i) = norm(qp.Xt);
        lastStateNorm(i) = norm(qp.last_successful_Xt);
        failureInvariantNorm(i) = failureComponents.position_invariant_state_norm;
        lastInvariantNorm(i) = lastComponents.position_invariant_state_norm;
        stateDeltaNorm(i) = norm(qp.Xt - qp.last_successful_Xt);
        carriedInputDeltaNorm(i) = norm(qp.Ut - qp.last_successful_Ut);
        changedFsmLegs(i) = nnz(qp.FSM ~= qp.last_successful_FSM);
        gNorm(i) = norm(qp.g);
        beqNorm(i) = norm(qp.beq);
        p = get_params(0);
        p.R = qp.R;
        p.Q = qp.Q;
        p.Qf = qp.Qf;
        [previousH, previousG, ~, ~, ~, previousBeq] = ...
            fcn_get_QP_form_eta(qp.last_successful_Xt, ...
            qp.last_successful_Ut, qp.last_successful_Xd, ...
            qp.last_successful_Ud, p);
        [previousA, previousB, previousD] = fcn_get_ABD_eta( ...
            qp.last_successful_Xt, qp.last_successful_Ut, p);
        previousGNorm(i) = norm(previousG);
        previousBeqNorm(i) = norm(previousBeq);
        hDeltaNorm(i) = norm(qp.H - previousH, 'fro');
        aDeltaNorm(i) = norm(qp.A - previousA, 'fro');
        bDeltaNorm(i) = norm(qp.B - previousB, 'fro');
        dDeltaNorm(i) = norm(qp.d - previousD);
        phaseForceCorrectionNorm(i) = norm(phase.phase1_z(1:12));
        phaseSolutionNorm(i) = norm(phase.phase1_z);
        zeroCorrectionMarginMin(i) = min(qp.bineq);
        phaseMarginMin(i) = min(qp.bineq - qp.Aineq*phase.phase1_z);
        [~, ~, activeExit(i), activeOutput] = quadprog( ...
            qp.H, qp.g, qp.Aineq, qp.bineq, qp.Aeq, qp.beq, [], [], ...
            phase.phase1_z, activeOptions);
        activeIterations(i) = activeOutput.iterations;
        activeConstraintViolation(i) = activeOutput.constrviolation;
        activeFirstOrder(i) = activeOutput.firstorderopt;
    end
    results = table(episode, decision, chunk, iteration, originalExit, ...
        originalIterations, phaseExit, phaseRelax, phaseEqualityLsq, ...
        phaseMaxInequality, phaseMaxEquality, rcondH, condestH, rankAeq, ...
        minEigenvalueH, maxEigenvalueH, activeExit, activeIterations, ...
        activeConstraintViolation, activeFirstOrder, failureStateNorm, ...
        lastStateNorm, failureInvariantNorm, lastInvariantNorm, ...
        stateDeltaNorm, carriedInputDeltaNorm, changedFsmLegs, gNorm, ...
        previousGNorm, beqNorm, previousBeqNorm, hDeltaNorm, ...
        aDeltaNorm, bDeltaNorm, ...
        dDeltaNorm, phaseForceCorrectionNorm, phaseSolutionNorm, ...
        zeroCorrectionMarginMin, phaseMarginMin, classification, ...
        originalMessage, file, 'VariableNames', { ...
        'episode','decision','chunk','iteration','original_exit', ...
        'original_iterations','phase_exit','phase_relax','phase_eq_lsq', ...
        'phase_max_ineq','phase_max_eq','rcond_H','condest_H','rank_Aeq', ...
        'min_eig_H','max_eig_H','active_exit','active_iterations', ...
        'active_constrviolation','active_firstorderopt', ...
        'failure_state_norm','last_state_norm','failure_invariant_norm', ...
        'last_invariant_norm','state_delta_norm','carried_input_delta_norm', ...
        'changed_fsm_legs','g_norm','previous_g_norm','beq_norm', ...
        'previous_beq_norm','H_delta_norm','A_delta_norm','B_delta_norm','d_delta_norm', ...
        'phase_force_correction_norm','phase_solution_norm', ...
        'zero_correction_margin_min','phase_margin_min','classification', ...
        'original_message','file'});
end

function key = localContactKey(rows)
    key = strings(height(rows), 1);
    for leg = 1:4
        field = sprintf('fsm_leg%d_end', leg);
        key = key + string(rows.(field));
    end
end

function file = localSingleFile(folder, pattern)
    files = dir(fullfile(folder, pattern));
    if numel(files) ~= 1
        error('analyze_reconnaissance_run:FileCount', ...
            'Expected one %s file under %s; found %d.', pattern, folder, numel(files));
    end
    file = string(fullfile(files.folder, files.name));
end

function value = localManifestNumber(manifest, key)
    token = regexp(manifest, ['(?m)^' key ':\s*([^\r\n]+)$'], ...
        'tokens', 'once');
    if isempty(token)
        value = NaN;
    else
        value = str2double(token{1});
    end
end

function bytes = localFolderBytes(folder)
    files = dir(fullfile(folder, '**', '*'));
    bytes = sum([files(~[files.isdir]).bytes]);
end

function value = localFirstOrNaN(values)
    if isempty(values)
        value = NaN;
    else
        value = values(1);
    end
end

function value = localMinOrNaN(values)
    if isempty(values)
        value = NaN;
    else
        value = min(values, [], 'omitmissing');
    end
end

function value = localMeanOrNaN(values)
    if isempty(values)
        value = NaN;
    else
        value = mean(values, 'omitmissing');
    end
end

function localWriteJson(file, value)
    fid = fopen(file, 'w');
    if fid < 0
        error('analyze_reconnaissance_run:OpenFailed', ...
            'Could not write %s.', file);
    end
    cleanupObj = onCleanup(@() fclose(fid));
    fprintf(fid, '%s\n', jsonencode(value, PrettyPrint=true));
end
