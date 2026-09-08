classdef testPhase3SolverPolicy < matlab.unittest.TestCase
    properties (TestParameter)
        Strategy = {"default", "active_set_feasible_point", "default_one_shot_fallback", ...
            "active_set_tight_primal_v1", "default_one_shot_fallback_tight_primal_v1"}
        NonRescuable = {"unclassified_solver_failure", "invalid_state", ...
            "mathematical_constraint_infeasible"}
    end
    methods (Test)
        function solvesHealthyQP(testCase, Strategy)
            p = testCase.problem();
            r = solve_mpc_qp(p, Strategy);
            testCase.verifyTrue(r.success);
            testCase.verifyEqual(r.z, [0;1], 'AbsTol', 2e-4);
            testCase.verifyFalse(r.fallback_attempted);
            testCase.verifyGreaterThanOrEqual(r.wall_time_s, ...
                r.quadprog_wall_time_s+r.phase1_wall_time_s);
            testCase.verifyNotEmpty(r.solver_options);
        end
        function infeasibilityNeverTriggersRescue(testCase)
            p = testCase.problem();
            p.bineq = [-1;-1];
            r = solve_mpc_qp(p, "default_one_shot_fallback");
            testCase.verifyFalse(r.success);
            testCase.verifyFalse(r.fallback_attempted);
            testCase.verifyEqual(r.classification, "mathematical_constraint_infeasible");
        end
        function nonfiniteDataNeverTriggersRescue(testCase)
            p = testCase.problem();
            p.g(1) = NaN;
            r = solve_mpc_qp(p, "default_one_shot_fallback");
            testCase.verifyEqual(r.classification, "invalid_state");
            testCase.verifyFalse(r.fallback_attempted);
            testCase.verifyGreaterThanOrEqual(r.wall_time_s, 0);
        end
        function unknownAndPhysicalFailuresAreNotNumeric(testCase, NonRescuable)
            r = struct('success', false, 'classification', NonRescuable, ...
                'phase1', struct('phase1_z', [0;1], 'classification', "linearly_feasible"));
            testCase.verifyFalse(should_rescue_qp(r));
        end
        function numericalFailureRequiresFiniteFeasibilityWitness(testCase)
            r = struct('success', false, 'classification', "numerical_solver_failure", ...
                'phase1', struct('phase1_z', [0;1], 'classification', "linearly_feasible"));
            testCase.verifyTrue(should_rescue_qp(r));
            r.phase1.phase1_z = [NaN;1];
            testCase.verifyFalse(should_rescue_qp(r));
            r.phase1 = struct();
            testCase.verifyFalse(should_rescue_qp(r));
        end
        function tightStrategyRetainsStrictPrimalGate(testCase)
            result = solve_mpc_qp(testCase.problem(),"active_set_tight_primal_v1");
            testCase.verifyEqual(result.solver_options.ConstraintTolerance,1e-10);
            testCase.verifyTrue(result.success);
            testCase.verifyLessThanOrEqual(result.constraint_violation,1e-6);
        end
        function tightFallbackStartsWithUnchangedDefault(testCase)
            result = solve_mpc_qp(testCase.problem(),"default_one_shot_fallback_tight_primal_v1");
            standard = solve_mpc_qp(testCase.problem(),"default");
            testCase.verifyFalse(result.fallback_attempted);
            testCase.verifyEqual(result.z,standard.z);
            testCase.verifyEqual(result.solver_options.ConstraintTolerance, ...
                standard.solver_options.ConstraintTolerance);
        end
    end
    methods (Static, Access = private)
        function p = problem()
            p = struct('H', eye(2), 'g', [-1;-2], ...
                'Aineq', -eye(2), 'bineq', [0;0], 'Aeq', [1,1], 'beq', 1);
        end
    end
end
