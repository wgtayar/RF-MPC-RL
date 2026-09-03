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

    objective = [zeros(numberVariables, 1); ones(numberInequalities, 1)];
    phaseAineq = [scaledAineq, -eye(numberInequalities)];
    phaseBineq = scaledBineq;
    phaseAeq = [scaledAeq, zeros(size(Aeq, 1), numberInequalities)];
    lowerBound = [-inf(numberVariables, 1); zeros(numberInequalities, 1)];
    options = optimoptions('linprog', 'Display', 'none', ...
        'Algorithm', 'dual-simplex-highs');

    [phaseSolution, phaseObjective, phaseExitflag, phaseOutput] = linprog( ...
        objective, phaseAineq, phaseBineq, phaseAeq, scaledBeq, ...
        lowerBound, [], options);

    result = struct();
    result.phase1_exitflag = phaseExitflag;
    result.phase1_output = phaseOutput;
    result.minimum_scaled_inequality_relaxation_l1 = phaseObjective;
    result.minimum_scaled_equality_residual_l2 = ...
        norm(scaledAeq*lsqminnorm(scaledAeq, scaledBeq) - scaledBeq);
    result.original_quadprog_exitflag = localField(failureQP, 'exitflag', NaN);

    tolerance = 1e-8;
    if phaseExitflag > 0
        z = phaseSolution(1:numberVariables);
        slack = phaseSolution(numberVariables+1:end);
        result.phase1_z = z;
        result.inequality_slack_scaled = slack;
        result.inequality_margin = bineq - Aineq*z;
        result.equality_residual = Aeq*z - beq;
        result.maximum_inequality_violation = ...
            max(max(-result.inequality_margin, 0), [], 'omitnan');
        result.maximum_equality_residual = max(abs(result.equality_residual));
        offendingRows = find(slack > tolerance);
        result.offending_inequality_rows = map_qp_inequality_rows(offendingRows);
        if phaseObjective <= tolerance
            if result.original_quadprog_exitflag <= 0
                result.classification = 'solver_difficulty_or_objective_numerics';
            else
                result.classification = 'linearly_feasible';
            end
        else
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
