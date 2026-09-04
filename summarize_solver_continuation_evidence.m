function summary = summarize_solver_continuation_evidence(evidenceFolders, outputFolder)
%summarize_solver_continuation_evidence Consolidate completed A/B case artifacts.

    evidenceFolders = string(evidenceFolders(:));
    if nargin < 2 || strlength(string(outputFolder)) == 0
        outputFolder = fileparts(evidenceFolders(1));
    end
    if ~isfolder(outputFolder)
        mkdir(outputFolder);
    end
    rows = repmat(localEmptyRow(), numel(evidenceFolders), 1);
    for i = 1:numel(evidenceFolders)
        data = load(fullfile(evidenceFolders(i), 'continuation_evidence.mat'));
        prefix = data.prefixComparison;
        branchExact = data.comparisonA.max_relative_error <= 1e-12 && ...
            data.comparisonB.max_relative_error <= 1e-12;
        hiddenValidated = prefix.FSM_relative_error == 0 && ...
            prefix.Xt_relative_error <= 1e-3 && ...
            prefix.Ut_relative_error <= 1e-2 && ...
            prefix.Xd_relative_error <= 1e-4;
        row = localEmptyRow();
        row.case_id = string(data.qp.failure_qp_id);
        row.failure_time_s = data.qp.fail_time_s;
        row.exact_branch_problem = branchExact;
        row.hidden_state_reconstruction_validated = hiddenValidated;
        row.prefix_max_relative_error = prefix.max_relative_error;
        row.prefix_Xt_relative_error = prefix.Xt_relative_error;
        row.prefix_Ut_relative_error = prefix.Ut_relative_error;
        row.default_exitflag = data.outA.trace.solver_exitflag(1);
        row.default_survival_s = data.outA.survived_duration_s;
        row.active_exitflag = data.outB.trace.solver_exitflag(1);
        row.active_iterations = data.outB.trace.solver_iterations(1);
        row.active_objective = data.outB.trace.objective(1);
        row.active_kkt_stationarity_inf = ...
            data.outB.trace.kkt_stationarity_inf(1);
        row.active_margin_min = data.outB.trace.inequality_margin_min(1);
        row.active_survival_s = data.outB.survived_duration_s;
        row.active_terminal_class = data.outB.terminal_reason;
        row.subsequent_solver_failures = data.outB.qp_failed_count;
        valid = find(isfinite( ...
            data.outB.trace.orientation_error_after_rad), 1, 'last');
        if ~isempty(valid)
            last = data.outB.trace(valid, :);
            row.final_orientation_error_rad = ...
                last.orientation_error_after_rad;
            row.final_angular_velocity = last.angular_velocity_after;
            row.final_com_velocity = last.com_velocity_after;
            row.final_position_invariant_norm = ...
                last.position_invariant_norm_after;
            row.final_Ut_norm = last.Ut_norm_after;
        end
        row.Ieq_A = data.outB.Ieq_A;
        row.outcome = localOutcome(data.outA, data.outB, ...
            branchExact, hiddenValidated);
        rows(i) = row;
    end
    summary = struct2table(rows);
    writetable(summary, fullfile(outputFolder, 'solver_ab_summary.csv'));
    save(fullfile(outputFolder, 'solver_ab_summary.mat'), ...
        'summary', 'evidenceFolders');
end

function outcome = localOutcome(outA, outB, branchExact, hiddenValidated)
    if ~branchExact || ~hiddenValidated
        outcome = "ambiguous";
    elseif outB.trace.solver_exitflag(1) <= 0
        outcome = "no_solver_rescue";
    elseif outA.survived_duration_s >= outA.requested_duration_s - 1e-9
        outcome = "ambiguous";
    elseif outB.survived_duration_s < 0.5
        outcome = "solver_rescue_but_state_still_collapses";
    elseif outB.survived_duration_s < outB.requested_duration_s - 1e-9
        outcome = "solver_rescue_delays_failure";
    else
        valid = find(isfinite( ...
            outB.trace.orientation_error_after_rad), 1, 'last');
        last = outB.trace(valid, :);
        healthy = last.orientation_error_after_rad <= 1 && ...
            last.angular_velocity_after <= 10 && ...
            last.position_invariant_norm_after <= 10 && ...
            last.Ut_norm_after <= 100;
        if healthy
            outcome = "solver_rescue_and_recovery";
        else
            outcome = "solver_rescue_but_state_still_collapses";
        end
    end
end

function row = localEmptyRow()
    row = struct('case_id', "", 'failure_time_s', NaN, ...
        'exact_branch_problem', false, ...
        'hidden_state_reconstruction_validated', false, ...
        'prefix_max_relative_error', NaN, ...
        'prefix_Xt_relative_error', NaN, 'prefix_Ut_relative_error', NaN, ...
        'default_exitflag', NaN, 'default_survival_s', NaN, ...
        'active_exitflag', NaN, 'active_iterations', NaN, ...
        'active_objective', NaN, 'active_kkt_stationarity_inf', NaN, ...
        'active_margin_min', NaN, 'active_survival_s', NaN, ...
        'active_terminal_class', "", 'subsequent_solver_failures', NaN, ...
        'final_orientation_error_rad', NaN, ...
        'final_angular_velocity', NaN, 'final_com_velocity', NaN, ...
        'final_position_invariant_norm', NaN, 'final_Ut_norm', NaN, ...
        'Ieq_A', NaN, 'outcome', "");
end
