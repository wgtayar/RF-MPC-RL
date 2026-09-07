function [classification, assessment] = classify_dynamic_viability_v2(initial, out, thresholds)
%classify_dynamic_viability_v2 Separate horizon-wide health from recovery.
% Thresholds are provisional unless explicitly validated. Descriptive classes
% alone must not authorize a policy action.
    requiredThresholds = {'orientation','angular_velocity','position_invariant','Ut', ...
        'solver_iterations','equality_residual','inequality_margin','kkt_stationarity'};
    assert(all(isfield(thresholds,requiredThresholds)), ...
        'classify_dynamic_viability_v2:Thresholds','Supply explicit health and solver thresholds.');
    limits = [thresholds.orientation,thresholds.angular_velocity, ...
        thresholds.position_invariant,thresholds.Ut];
    validateattributes(limits,{'numeric'},{'finite','positive','numel',4});
    validateattributes(out.requested_duration_s,{'numeric'},{'scalar','finite','positive'});
    assessment = struct('schema_version','viability_v2_whole_horizon', ...
        'horizon_s',out.requested_duration_s,'within_envelope_at_horizon',false, ...
        'recovery_candidate',false,'safe_for_policy',false,'minimum_health_margin',NaN, ...
        'thresholds_validated',isfield(thresholds,'validated') && isequal(thresholds.validated,true));
    terminal = string(out.terminal_reason);
    switch terminal
        case "mathematical_constraint_infeasible"
            classification = "mathematical_infeasibility";
            return
        case {"numerical_solver_failure","numerical_solver_failure_unrecovered"}
            classification = "numerical_solver_failure";
            return
        case "invalid_state"
            classification = "invalid_state";
            return
        case "unclassified_solver_failure"
            classification = "unclassified_solver_failure";
            return
    end
    names = {'orientation_error_after_rad','angular_velocity_after', ...
        'position_invariant_norm_after','Ut_norm_after'};
    beforeNames = {'orientation_error_before_rad','angular_velocity_before', ...
        'position_invariant_norm_before','Ut_norm_before'};
    solverNames = {'solver_iterations','equality_residual_max_abs', ...
        'inequality_margin_min','kkt_stationarity_inf'};
    required = [names,beforeNames,solverNames,{'time_s','solver_wall_time_s'}];
    if isempty(out.trace) || ~all(ismember(required,out.trace.Properties.VariableNames)) || ...
            ~isfield(out,'mpc_timestep_s') || ~isfield(out,'initial_state') || ...
            ~isfield(out.initial_state,'time_s') || ~isfinite(out.survived_duration_s) || ...
            out.survived_duration_s+1e-9 < out.requested_duration_s || ...
            terminal ~= "horizon_complete"
        classification = "insufficient_data";
        return
    end
    dt = out.mpc_timestep_s;
    validateattributes(dt,{'numeric'},{'scalar','finite','positive'});
    expectedSteps = round(out.requested_duration_s/dt);
    expectedTimes = out.initial_state.time_s+dt*(0:expectedSteps-1).';
    if abs(expectedSteps*dt-out.requested_duration_s) > 1e-9 || ...
            height(out.trace) ~= expectedSteps || ...
            any(~isfinite(out.trace.time_s)) || ...
            any(abs(out.trace.time_s-expectedTimes) > 1e-9)
        classification = "insufficient_data";
        return
    end
    initialValues = [initial.orientation,initial.angular_velocity,initial.position_invariant,initial.Ut];
    after = out.trace{:,names};
    before = out.trace{:,beforeNames};
    diagnostics = out.trace{:,solverNames};
    if any(~isfinite([initialValues(:);after(:);before(:)])) || ...
            any([initialValues(:);after(:);before(:)] < 0)
        classification = "invalid_state";
        return
    end
    if any(~isfinite(diagnostics(:))) || any(~isfinite(out.trace.solver_wall_time_s)) || ...
            any(out.trace.solver_wall_time_s < 0)
        classification = "insufficient_data";
        return
    end
    scores = max([initialValues;before;after]./limits,[],2);
    peakScore = max(scores);
    finalScore = max(after(end,:)./limits);
    assessment.minimum_health_margin = 1-peakScore;
    robust = all(diagnostics(:,1) <= thresholds.solver_iterations) && ...
        all(diagnostics(:,2) <= thresholds.equality_residual) && ...
        all(diagnostics(:,3) >= thresholds.inequality_margin) && ...
        all(diagnostics(:,4) <= thresholds.kkt_stationarity) && ...
        all(out.trace.solver_wall_time_s <= dt);
    if ismember('solver_classification',out.trace.Properties.VariableNames)
        robust = robust && ~any(out.trace.solver_classification == "numerical_solver_failure_recovered");
    end
    if peakScore > 1
        if finalScore <= 0.8*peakScore
            classification = "degraded_recovering";
            assessment.recovery_candidate = true;
        else
            classification = "degraded_not_recovering";
        end
    elseif peakScore > 0.8
        classification = "marginal";
    elseif robust
        classification = "healthy_robust";
    else
        classification = "healthy_solver_stressed";
    end
    assessment.within_envelope_at_horizon = peakScore <= 1;
    assessment.safe_for_policy = assessment.thresholds_validated && ...
        any(classification == ["healthy_robust","healthy_solver_stressed"]);
end
