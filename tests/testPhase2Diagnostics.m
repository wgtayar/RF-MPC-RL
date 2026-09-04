classdef testPhase2Diagnostics < matlab.unittest.TestCase
    %testPhase2Diagnostics Controlled tests for Phase-2 replay helpers.

    properties (TestParameter)
        Strategy = {"default", "active_set_feasible_point"}
    end

    properties
        Config
    end

    methods (TestClassSetup)
        function loadConfiguration(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            rootFolder = fileparts(testFolder);
            addpath(rootFolder);
            bootstrap_RF_MPC_RL();
            data = load(fullfile(rootFolder, 'rlEnv_MPC_R.mat'), 'cfg');
            testCase.Config = data.cfg;
        end
    end

    methods (TestMethodSetup)
        function resetPersistentState(~)
            reset_mpc_case_state();
        end
    end

    methods (Test)
        function feasibleProblemSolvesWithBothStrategies(testCase, Strategy)
            problem = struct('H', eye(2), 'g', [-1; -2], ...
                'Aineq', [-1, 0; 0, -1], 'bineq', [0; 0], ...
                'Aeq', [1, 1], 'beq', 1);
            result = solve_mpc_qp(problem, Strategy);
            testCase.verifyTrue(result.success);
            testCase.verifyGreaterThan(result.exitflag, 0);
            testCase.verifyLessThanOrEqual( ...
                result.diagnostics.equality_residual_max_abs, 1e-8);
            testCase.verifyGreaterThanOrEqual( ...
                result.diagnostics.inequality_margin_min, -1e-8);
        end

        function activeSetRejectsInconsistentConstraints(testCase)
            problem = struct('H', 1, 'g', 0, 'Aineq', 1, ...
                'bineq', 0, 'Aeq', 1, 'beq', 1);
            result = solve_mpc_qp(problem, "active_set_feasible_point");
            testCase.verifyFalse(result.success);
            testCase.verifyEqual(result.classification, ...
                "mathematical_constraint_infeasible");
        end

        function explicitReplayStateBranchesEquivalently(testCase)
            state0 = initialize_mpc_replay_state(testCase.Config, 0);
            control = struct('R', state0.R, 'v_cmd', 0.5, 'a_cmd', 0.25);
            prefixOptions = struct('duration_s', 0.003, ...
                'solver_strategy', "default", 'capture_trace', true, ...
                'update_proxy', true, 'update_battery', false);
            [branchState, prefix] = simulate_mpc_horizon( ...
                state0, control, testCase.Config, prefixOptions);
            testCase.verifyTrue(prefix.completed_horizon);

            branchOptions = prefixOptions;
            branchOptions.duration_s = 0.002;
            [stateA, outA] = simulate_mpc_horizon( ...
                branchState, control, testCase.Config, branchOptions);
            [stateB, outB] = simulate_mpc_horizon( ...
                branchState, control, testCase.Config, branchOptions);
            testCase.verifyEqual(stateB.Xt, stateA.Xt, 'AbsTol', 1e-12);
            testCase.verifyEqual(stateB.Ut, stateA.Ut, 'AbsTol', 1e-12);
            testCase.verifyEqual(stateB.fsm_internal_state, ...
                stateA.fsm_internal_state);
            traceA = removevars(outA.trace, 'solver_wall_time_s');
            traceB = removevars(outB.trace, 'solver_wall_time_s');
            testCase.verifyEqual(traceB, traceA);
        end

        function archivedFailureHasActiveSetRescue(testCase)
            monitorRoot = fullfile(fileparts(fileparts(fileparts( ...
                mfilename('fullpath')))), 'RL-MPC-Monitor');
            snapshot = fullfile(monitorRoot, 'runs', '2026', ...
                'run_2026-09-03_12-14-20', 'qp_failures', ...
                'ep0010_dec04_chunk01_iter0046.mat');
            testCase.assumeTrue(isfile(snapshot), ...
                'Archived Phase-1 evidence is not available.');
            saved = load(snapshot, 'failureQP');
            qp = saved.failureQP;
            problem = struct('H', qp.H, 'g', qp.g, ...
                'Aineq', qp.Aineq, 'bineq', qp.bineq, ...
                'Aeq', qp.Aeq, 'beq', qp.beq);
            result = solve_mpc_qp(problem, "active_set_feasible_point");
            testCase.verifyTrue(result.success);
            testCase.verifyGreaterThan(result.exitflag, 0);
            testCase.verifyLessThanOrEqual( ...
                result.diagnostics.equality_residual_max_abs, 1e-6);
        end
    end
end
