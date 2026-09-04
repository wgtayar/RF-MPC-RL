function summary = analyze_dynamic_divergence(runDir, evidenceFolders, outputFolder)
%analyze_dynamic_divergence Locate sustained pre-failure dynamic separation.

    if nargin < 3 || strlength(string(outputFolder)) == 0
        outputFolder = fullfile(fileparts(mfilename('fullpath')), ...
            'Phase 2 Outputs', 'dynamic_divergence');
    end
    if ~isfolder(outputFolder)
        mkdir(outputFolder);
    end
    runDir = string(runDir);
    evidenceFolders = string(evidenceFolders(:));
    matches = readtable(localSingleFile(runDir, ...
        'matched_successful_transitions.csv'), TextType='string');
    rows = repmat(localEmptyRow(), numel(evidenceFolders), 1);

    variableNames = ["orientation_error", "angular_velocity", ...
        "translational_velocity_norm", "position_invariant_norm", "Ut_norm"];
    absoluteThresholds = [0.25, 2.0, 0.5, 2.0, 20.0];
    persistenceSamples = 50;

    for i = 1:numel(evidenceFolders)
        folder = evidenceFolders(i);
        data = load(fullfile(folder, 'continuation_evidence.mat'), ...
            'qp', 'prefixComparison');
        qp = data.qp;
        trace = readtable(fullfile(folder, 'prefix_trace.csv'), ...
            TextType='string');
        [~, uniqueIndex] = unique(trace.time_s, 'stable');
        trace = trace(uniqueIndex, :);
        match = matches(matches.failure_qp_id == string(qp.failure_qp_id), :);
        if height(match) ~= 1
            error('analyze_dynamic_divergence:MatchCount', ...
                'Expected one matched row for %s.', qp.failure_qp_id);
        end

        baseline = [match.match_orientation_error, ...
            match.match_angular_velocity_norm, match.match_velocity_norm, ...
            match.match_position_invariant_norm, match.match_input_norm];
        values = [trace.orientation_error_before_rad, ...
            trace.angular_velocity_before, trace.com_velocity_before, ...
            trace.position_invariant_norm_before, trace.Ut_norm_before];
        detection = detect_persistent_divergence(trace.time_s, values, ...
            baseline, absoluteThresholds, persistenceSamples);

        row = localEmptyRow();
        row.case_id = string(qp.failure_qp_id);
        row.failure_time_s = qp.fail_time_s;
        row.window_start_s = trace.time_s(1);
        row.window_duration_s = qp.fail_time_s - trace.time_s(1);
        row.divergence_found = detection.found;
        row.left_censored = detection.left_censored;
        row.t_first_divergence_s = detection.time;
        row.lead_before_solver_failure_s = qp.fail_time_s - detection.time;
        if detection.found
            row.first_variable = variableNames(detection.variable_index);
            row.first_standardized_excess = detection.standardized_excess( ...
                detection.index, detection.variable_index);
        end
        row.nearest_mechanism = localNearestMechanism(trace, detection);
        row.prefix_Xt_relative_error = data.prefixComparison.Xt_relative_error;
        row.prefix_Ut_relative_error = data.prefixComparison.Ut_relative_error;
        row.match_episode = match.match_episode;
        row.match_decision = match.match_decision;
        row.match_chunk = match.match_chunk;
        rows(i) = row;
        localPlotTrace(trace, baseline, absoluteThresholds, ...
            variableNames, detection, outputFolder, row.case_id);
    end
    summary = struct2table(rows);
    writetable(summary, fullfile(outputFolder, 'dynamic_divergence_summary.csv'));
    save(fullfile(outputFolder, 'dynamic_divergence_summary.mat'), ...
        'summary', 'variableNames', 'absoluteThresholds', ...
        'persistenceSamples');
end

function mechanism = localNearestMechanism(trace, detection)
    mechanism = "window_already_diverged";
    if ~detection.found || detection.left_censored
        return
    end
    index = detection.index;
    candidates = strings(0, 1);
    if index > 1 && any(trace{index, {'fsm_leg1','fsm_leg2','fsm_leg3','fsm_leg4'}} ~= ...
            trace{index-1, {'fsm_leg1','fsm_leg2','fsm_leg3','fsm_leg4'}})
        candidates(end+1) = "FSM_contact_event";
    end
    if index > 1 && abs(trace.desired_force_norm(index) - ...
            trace.desired_force_norm(index-1)) > 1e-6
        candidates(end+1) = "desired_force_update";
    end
    if index > 1 && trace.solver_iterations(index) > ...
            max(2*trace.solver_iterations(index-1), 50)
        candidates(end+1) = "solver_iteration_growth";
    end
    if isempty(candidates)
        mechanism = "accumulated_dynamic_error";
    else
        mechanism = strjoin(candidates, "+");
    end
end

function localPlotTrace(trace, baseline, thresholds, names, detection, folder, caseId)
    figureHandle = figure('Visible', 'off', 'Color', 'w', ...
        'Position', [100, 100, 1000, 700]);
    cleanupObj = onCleanup(@() close(figureHandle));
    values = [trace.orientation_error_before_rad, ...
        trace.angular_velocity_before, trace.com_velocity_before, ...
        trace.position_invariant_norm_before, trace.Ut_norm_before];
    relativeTime = trace.time_s - trace.time_s(end);
    tiledlayout(3, 2, 'TileSpacing', 'compact');
    for variable = 1:numel(names)
        nexttile;
        plot(relativeTime, values(:, variable), 'LineWidth', 1.2);
        hold on;
        yline(baseline(variable) + thresholds(variable), '--');
        if detection.found
            xline(detection.time - trace.time_s(end), ':');
        end
        grid on;
        title(strrep(names(variable), '_', ' '));
        xlabel('time before captured solver failure (s)');
    end
    exportgraphics(figureHandle, fullfile(folder, caseId + "_divergence.png"), ...
        'Resolution', 160);
end

function row = localEmptyRow()
    row = struct('case_id', "", 'failure_time_s', NaN, ...
        'window_start_s', NaN, 'window_duration_s', NaN, ...
        'divergence_found', false, 'left_censored', false, ...
        't_first_divergence_s', NaN, ...
        'lead_before_solver_failure_s', NaN, 'first_variable', "", ...
        'first_standardized_excess', NaN, 'nearest_mechanism', "", ...
        'prefix_Xt_relative_error', NaN, 'prefix_Ut_relative_error', NaN, ...
        'match_episode', NaN, 'match_decision', NaN, 'match_chunk', NaN);
end

function file = localSingleFile(folder, pattern)
    files = dir(fullfile(folder, '**', pattern));
    if numel(files) ~= 1
        error('analyze_dynamic_divergence:FileCount', ...
            'Expected one %s under %s; found %d.', ...
            pattern, folder, numel(files));
    end
    file = string(fullfile(files.folder, files.name));
end
