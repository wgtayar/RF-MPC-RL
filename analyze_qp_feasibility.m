function result = analyze_qp_feasibility(snapshotOrQp)
%analyze_qp_feasibility Distinguish linear infeasibility from solver difficulty.

    failureQP = localLoadQp(snapshotOrQp);
    required = {'H','Aineq','bineq','Aeq','beq'};
    for i = 1:numel(required)
        if ~isfield(failureQP, required{i})
            error('analyze_qp_feasibility:MissingField', ...
                'The failure QP is missing field "%s".', required{i});
        end
    end

    H = failureQP.H;
    Aineq = failureQP.Aineq;
    bineq = failureQP.bineq(:);
    Aeq = failureQP.Aeq;
    beq = failureQP.beq(:);
    if any(~isfinite([H(:); Aineq(:); bineq; Aeq(:); beq]))
        result = struct('classification', 'invalid_problem_data', ...
            'phase1_exitflag', NaN);
        return
    end

    numberVariables = size(H, 1);
    numberInequalities = size(Aineq, 1);
    inequalityScale = max([vecnorm(Aineq, 2, 2), abs(bineq), ...
        ones(numberInequalities, 1)], [], 2);
    equalityScale = max([vecnorm(Aeq, 2, 2), abs(beq), ...
        ones(size(Aeq, 1), 1)], [], 2);
    scaledAineq = Aineq ./ inequalityScale;
    scaledBineq = bineq ./ inequalityScale;
    scaledAeq = Aeq ./ equalityScale;
    scaledBeq = beq ./ equalityScale;

    equalityParticular = lsqminnorm(scaledAeq, scaledBeq);
    scaledEqualityResidual = scaledAeq*equalityParticular - scaledBeq;
    nullBasis = null(scaledAeq);
    numberFreeVariables = size(nullBasis, 2);
    objective = [zeros(numberFreeVariables, 1); ones(numberInequalities, 1)];
    phaseAineq = [scaledAineq*nullBasis, -eye(numberInequalities)];
    phaseBineq = scaledBineq - scaledAineq*equalityParticular;
    lowerBound = [-inf(numberFreeVariables, 1); zeros(numberInequalities, 1)];
    options = optimoptions('linprog', 'Display', 'none', ...
        'Algorithm', 'dual-simplex-highs', ...
        'ConstraintTolerance', 1e-10, 'OptimalityTolerance', 1e-10);

    primaryPhaseError = [];
    phaseAlgorithm = "dual-simplex-highs";
    try
        [phaseSolution, phaseObjective, phaseExitflag, phaseOutput] = linprog( ...
            objective, phaseAineq, phaseBineq, [], [], lowerBound, [], options);
    catch ME
        primaryPhaseError = ME;
        phaseAlgorithm = "dual-simplex";
        fallbackOptions = optimoptions(options, 'Algorithm', 'dual-simplex');
        [phaseSolution, phaseObjective, phaseExitflag, phaseOutput] = linprog( ...
            objective, phaseAineq, phaseBineq, [], [], lowerBound, [], ...
            fallbackOptions);
        options = fallbackOptions;
    end

    result = struct();
    result.phase1_exitflag = phaseExitflag;
    result.phase1_output = phaseOutput;
    result.phase1_solver_algorithm = phaseAlgorithm;
    result.phase1_solver_fallback_used = ~isempty(primaryPhaseError);
    if isempty(primaryPhaseError)
        result.phase1_primary_error_identifier = "";
        result.phase1_primary_error_message = "";
    else
        result.phase1_primary_error_identifier = string(primaryPhaseError.identifier);
        result.phase1_primary_error_message = string(primaryPhaseError.message);
    end
    result.phase1_solver_objective = phaseObjective;
    result.minimum_scaled_equality_residual_l2 = norm(scaledEqualityResidual);
    result.original_quadprog_exitflag = localField(failureQP, 'exitflag', NaN);

    equalityTolerance = 1e-8;
    feasibilityTolerance = 1e-7;
    if phaseExitflag > 0 && ...
            result.minimum_scaled_equality_residual_l2 <= equalityTolerance
        if numberFreeVariables == 0
            freeSolution = zeros(0, 1);
        else
            freeSolution = phaseSolution(1:numberFreeVariables);
        end
        z = equalityParticular + nullBasis*freeSolution;
        slack = phaseSolution(numberFreeVariables+1:end);
        result.phase1_z = z;
        result.inequality_slack_scaled = slack;
        scaledViolation = max(scaledAineq*z - scaledBineq, 0);
        result.minimum_scaled_inequality_relaxation_l1 = sum(scaledViolation);
        result.inequality_margin = bineq - Aineq*z;
        result.equality_residual = Aeq*z - beq;
        result.maximum_inequality_violation = ...
            max(max(-result.inequality_margin, 0), [], 'omitnan');
        result.maximum_equality_residual = max(abs(result.equality_residual));
        if max(scaledViolation, [], 'omitnan') <= feasibilityTolerance && ...
                result.maximum_equality_residual <= feasibilityTolerance
            result.offending_inequality_rows = map_qp_inequality_rows([]);
            if result.original_quadprog_exitflag <= 0
                result.classification = 'solver_difficulty_or_objective_numerics';
            else
                result.classification = 'linearly_feasible';
            end
        else
            offendingRows = find(scaledViolation > feasibilityTolerance | ...
                slack > feasibilityTolerance);
            result.offending_inequality_rows = map_qp_inequality_rows(offendingRows);
            result.classification = 'mathematically_infeasible_linear_constraints';
        end
    else
        result = localSolveJointRelaxation(result, scaledAineq, scaledBineq, ...
            scaledAeq, scaledBeq, numberVariables, options);
    end
end

function result = localSolveJointRelaxation(result, Aineq, bineq, Aeq, beq, numberVariables, options)
    numberInequalities = size(Aineq, 1);
    numberEqualities = size(Aeq, 1);
    objective = [zeros(numberVariables, 1); ones(numberInequalities, 1); ...
        ones(2*numberEqualities, 1)];
    zeroIneqEq = zeros(numberInequalities, 2*numberEqualities);
    zeroEqIneq = zeros(numberEqualities, numberInequalities);
    phaseA = [
        Aineq, -eye(numberInequalities), zeroIneqEq
        Aeq, zeroEqIneq, -eye(numberEqualities), zeros(numberEqualities)
        -Aeq, zeroEqIneq, zeros(numberEqualities), -eye(numberEqualities)
    ];
    phaseB = [bineq; beq; -beq];
    lowerBound = [-inf(numberVariables, 1); ...
        zeros(numberInequalities + 2*numberEqualities, 1)];
    [solution, objectiveValue, exitflag, output] = linprog( ...
        objective, phaseA, phaseB, [], [], lowerBound, [], options);
    result.joint_relaxation_exitflag = exitflag;
    result.joint_relaxation_output = output;
    result.minimum_joint_scaled_relaxation_l1 = objectiveValue;
    result.joint_relaxation_solution = solution;
    if exitflag > 0
        result.classification = 'equality_and_or_inequality_relaxation_required';
    else
        result.classification = 'phase1_solver_failure';
    end
end

function failureQP = localLoadQp(snapshotOrQp)
    if ischar(snapshotOrQp) || isstring(snapshotOrQp)
        loaded = load(snapshotOrQp, 'failureQP');
        if ~isfield(loaded, 'failureQP')
            error('analyze_qp_feasibility:MissingVariable', ...
                'Snapshot does not contain a failureQP variable: %s', snapshotOrQp);
        end
        failureQP = loaded.failureQP;
    else
        failureQP = snapshotOrQp;
    end
end

function value = localField(s, field, defaultValue)
    if isfield(s, field)
        value = s.(field);
    else
        value = defaultValue;
    end
end
