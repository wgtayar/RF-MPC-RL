function result = solve_mpc_qp(problem, strategy, options)
%solve_mpc_qp Solve an MPC QP with explicit, auditable solver semantics.

    if nargin < 2 || strlength(string(strategy)) == 0
        strategy = "default";
    end
    if nargin < 3
        options = struct();
    end
    strategy = string(strategy);
    classifyFailure = localOption(options, 'classify_failure', true);

    required = {'H','g','Aineq','bineq','Aeq','beq'};
    for i = 1:numel(required)
        if ~isfield(problem, required{i})
            error('solve_mpc_qp:MissingField', ...
                'QP problem is missing field "%s".', required{i});
        end
    end

    H = problem.H;
    g = problem.g;
    Aineq = problem.Aineq;
    bineq = problem.bineq;
    Aeq = problem.Aeq;
    beq = problem.beq;
    result = localEmptyResult(strategy);

    if any(~isfinite([H(:); g(:); Aineq(:); bineq(:); Aeq(:); beq(:)]))
        result.classification = "invalid_state";
        return
    end

    switch strategy
        case "default"
            qpOptions = optimoptions('quadprog', 'Display', 'off');
            x0 = [];
        case "active_set_feasible_point"
            phase = analyze_qp_feasibility(problem);
            result.phase1 = phase;
            acceptedPhaseClasses = ...
                ["solver_difficulty_or_objective_numerics", "linearly_feasible"];
            if ~isfield(phase, 'phase1_z') || ...
                    ~any(string(phase.classification) == acceptedPhaseClasses)
                result.classification = localPhaseClassification(phase);
                return
            end
            x0 = phase.phase1_z;
            qpOptions = optimoptions('quadprog', 'Display', 'off', ...
                'Algorithm', 'active-set', 'MaxIterations', 5000);
        otherwise
            error('solve_mpc_qp:UnknownStrategy', ...
                'Unknown QP strategy "%s".', strategy);
    end

    solveTimer = tic;
    [z, objective, exitflag, output, lambda] = quadprog( ...
        H, g, Aineq, bineq, Aeq, beq, [], [], x0, qpOptions);
    result.wall_time_s = toc(solveTimer);
    result.z = z;
    result.objective = objective;
    result.exitflag = exitflag;
    result.output = output;
    result.lambda = lambda;
    result.iterations = localOutputValue(output, 'iterations');
    result.first_order_opt = localOutputValue(output, 'firstorderopt');
    result.constraint_violation = localOutputValue(output, 'constrviolation');

    if exitflag > 0 && ~isempty(z)
        result.success = true;
        result.classification = "solver_success";
        result.diagnostics = compute_qp_diagnostics( ...
            z, Aineq, bineq, Aeq, beq);
        result.kkt_stationarity_inf = localStationarityResidual( ...
            H, g, Aineq, Aeq, z, lambda);
        result.kkt_complementarity_inf = localComplementarityResidual( ...
            Aineq, bineq, z, lambda);
    elseif classifyFailure
        phase = analyze_qp_feasibility(problem);
        result.phase1 = phase;
        result.classification = localPhaseClassification(phase);
    else
        result.classification = "numerical_solver_failure";
    end
end

function result = localEmptyResult(strategy)
    result = struct( ...
        'strategy', strategy, ...
        'success', false, ...
        'classification', "numerical_solver_failure", ...
        'z', [], ...
        'objective', NaN, ...
        'exitflag', NaN, ...
        'output', struct(), ...
        'lambda', struct(), ...
        'iterations', NaN, ...
        'wall_time_s', NaN, ...
        'first_order_opt', NaN, ...
        'constraint_violation', NaN, ...
        'kkt_stationarity_inf', NaN, ...
        'kkt_complementarity_inf', NaN, ...
        'diagnostics', struct(), ...
        'phase1', struct());
end

function value = localOption(options, name, defaultValue)
    if isfield(options, name)
        value = options.(name);
    else
        value = defaultValue;
    end
end

function value = localOutputValue(output, name)
    if isstruct(output) && isfield(output, name)
        value = output.(name);
    else
        value = NaN;
    end
end

function classification = localPhaseClassification(phase)
    phaseClass = string(phase.classification);
    if phaseClass == "mathematically_infeasible_linear_constraints" || ...
            phaseClass == "equality_and_or_inequality_relaxation_required"
        classification = "mathematical_constraint_infeasible";
    elseif phaseClass == "invalid_problem_data"
        classification = "invalid_state";
    else
        classification = "numerical_solver_failure";
    end
end

function residual = localStationarityResidual(H, g, Aineq, Aeq, z, lambda)
    gradient = H*z + g;
    if isstruct(lambda) && isfield(lambda, 'ineqlin') && ~isempty(lambda.ineqlin)
        gradient = gradient + Aineq.'*lambda.ineqlin;
    end
    if isstruct(lambda) && isfield(lambda, 'eqlin') && ~isempty(lambda.eqlin)
        gradient = gradient + Aeq.'*lambda.eqlin;
    end
    residual = norm(gradient, inf);
end

function residual = localComplementarityResidual(Aineq, bineq, z, lambda)
    residual = NaN;
    if isstruct(lambda) && isfield(lambda, 'ineqlin') && ...
            ~isempty(lambda.ineqlin)
        residual = norm(lambda.ineqlin .* (bineq - Aineq*z), inf);
    end
end
