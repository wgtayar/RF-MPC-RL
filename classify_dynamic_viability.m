function classification = classify_dynamic_viability(initialMetrics, out, thresholds)
%classify_dynamic_viability Assign one explicit continuation outcome class.

    if nargin < 3 || isempty(thresholds)
        thresholds = struct('orientation', 1, 'angular_velocity', 10, ...
            'position_invariant', 10, 'Ut', 100, ...
            'solver_iterations', 100, 'equality_residual', 1e-6, ...
            'inequality_margin', -1e-6, 'kkt_stationarity', 1e-4);
    end
    terminal = string(out.terminal_reason);
    if terminal == "mathematical_constraint_infeasible"
        classification = "mathematical_infeasibility";
        return
    elseif terminal == "numerical_solver_failure"
        classification = "solver_failure";
        return
    elseif terminal == "invalid_state"
        classification = "invalid_state";
        return
    end

    valid = find(isfinite(out.trace.orientation_error_after_rad), 1, 'last');
    if isempty(valid)
        classification = "solver_failure";
        return
    end
    final = out.trace(valid, :);
    healthy = final.orientation_error_after_rad <= thresholds.orientation && ...
        final.angular_velocity_after <= thresholds.angular_velocity && ...
        final.position_invariant_norm_after <= thresholds.position_invariant && ...
        final.Ut_norm_after <= thresholds.Ut;
    robust = max(out.trace.solver_iterations, [], 'omitnan') <= ...
        thresholds.solver_iterations && ...
        max(out.trace.equality_residual_max_abs, [], 'omitnan') <= ...
        thresholds.equality_residual && ...
        min(out.trace.inequality_margin_min, [], 'omitnan') >= ...
        thresholds.inequality_margin && ...
        max(out.trace.kkt_stationarity_inf, [], 'omitnan') <= ...
        thresholds.kkt_stationarity;
    if healthy && robust
        classification = "healthy_robust_solver";
    elseif healthy
        classification = "healthy_solver_stressed";
    else
        initialScore = localHealthScore(initialMetrics, thresholds);
        finalMetrics = struct('orientation', final.orientation_error_after_rad, ...
            'angular_velocity', final.angular_velocity_after, ...
            'position_invariant', final.position_invariant_norm_after, ...
            'Ut', final.Ut_norm_after);
        finalScore = localHealthScore(finalMetrics, thresholds);
        if finalScore <= 0.8*initialScore
            classification = "degraded_recoverable";
        else
            classification = "degraded_high_failure_risk";
        end
    end
end

function score = localHealthScore(metrics, thresholds)
    score = max([metrics.orientation/thresholds.orientation, ...
        metrics.angular_velocity/thresholds.angular_velocity, ...
        metrics.position_invariant/thresholds.position_invariant, ...
        metrics.Ut/thresholds.Ut]);
end
