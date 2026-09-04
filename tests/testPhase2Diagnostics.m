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

        function persistentDivergenceRequiresSpecifiedRun(testCase)
            time = (0:0.01:0.19).';
            values = zeros(numel(time), 2);
            values(8:end, 2) = 3;
            result = detect_persistent_divergence( ...
                time, values, [0, 0], [1, 1], 5);
            testCase.verifyTrue(result.found);
            testCase.verifyEqual(result.index, 8);
            testCase.verifyEqual(result.time, 0.07, 'AbsTol', 1e-12);
            testCase.verifyEqual(result.variable_index, 2);
            testCase.verifyFalse(result.left_censored);
        end

        function viabilityClassifierSeparatesSolverAndHealth(testCase)
            initial = struct('orientation', 0.5, 'angular_velocity', 2, ...
                'position_invariant', 3, 'Ut', 50);
            trace = table(20, 1e-8, 0, 1e-8, 0.4, 2, 3, 50, ...
                'VariableNames', {'solver_iterations', ...
                'equality_residual_max_abs','inequality_margin_min', ...
                'kkt_stationarity_inf','orientation_error_after_rad', ...
                'angular_velocity_after','position_invariant_norm_after', ...
                'Ut_norm_after'});
            out = struct('terminal_reason', "horizon_complete", ...
                'trace', trace);
            testCase.verifyEqual(classify_dynamic_viability(initial, out), ...
                "healthy_robust_solver");
            out.terminal_reason = "mathematical_constraint_infeasible";
            testCase.verifyEqual(classify_dynamic_viability(initial, out), ...
                "mathematical_infeasibility");
        end

        function actionSamplesCoverRequiredSlices(testCase)
            samples = generate_phase2_action_samples(testCase.Config);
            testCase.verifyTrue(any(samples.profile == ...
                "Agent74_75_initial_dR"));
            testCase.verifyTrue(any(samples.sample_type == "dR_slice"));
            testCase.verifyTrue(any(samples.sample_type == ...
                "local_perturbation"));
            testCase.verifyGreaterThan(height(samples), 100);
        end

        function transactionRestorePreservesPayloadAndContinuation(testCase)
            simulation = initialize_mpc_replay_state(testCase.Config, 0);
            decision = struct('episode', 2, 'decision', 4, ...
                'progress_m', 82.5, 'charge_As', 1234);
            controller = struct('previous_gamma', [0.3; 0.2], ...
                'candidate_action', zeros(5, 1), ...
                'applied_action', nan(5, 1), 'fallback_level', 0);
            snapshot = capture_rl_transaction_state( ...
                simulation, decision, controller);
            [restoredSimulation, restoredDecision, restoredController] = ...
                restore_rl_transaction_state(snapshot);
            testCase.verifyEqual(restoredSimulation, simulation);
            testCase.verifyEqual(restoredDecision, decision);
            testCase.verifyEqual(restoredController, controller);

            control = struct('R', simulation.R, ...
                'v_cmd', 0.5, 'a_cmd', 0.25);
            options = struct('duration_s', 0.002, ...
                'solver_strategy', "default", 'capture_trace', true, ...
                'update_proxy', true, 'update_battery', false);
            [expected, expectedOut] = simulate_mpc_horizon( ...
                simulation, control, testCase.Config, options);
            [actual, actualOut] = simulate_mpc_horizon( ...
                restoredSimulation, control, testCase.Config, options);
            testCase.verifyEqual(actual.Xt, expected.Xt, 'AbsTol', 1e-12);
            testCase.verifyEqual(actual.Ut, expected.Ut, 'AbsTol', 1e-12);
            expectedTrace = removevars(expectedOut.trace, 'solver_wall_time_s');
            actualTrace = removevars(actualOut.trace, 'solver_wall_time_s');
            testCase.verifyEqual(actualTrace, expectedTrace);
        end

        function activeRewardStillPrefersAboveTargetPace(testCase)
            cfg = testCase.Config;
            window = struct('tracking_error_mean', cfg.TRACK_REF, ...
                'control_effort_mean', cfg.EFFORT_REF, 'Ieq_window', 55, ...
                'soc_start_pct', 75.5, 'soc_end_pct', 75, ...
                'lag_frac', 0, 'time_frac', 0.5, 'progress_frac', 0.5, ...
                'distance_start_m', 160, ...
                'window_distance_m', cfg.MISSION.WINDOW_TARGET_M, ...
                'v_exec', cfg.V_REQ_FIXED, 'a_exec', 0.5, ...
                'delta_v_exec', 0, 'delta_gamma_v', 0, ...
                'dR2', -cfg.DR_MAX, 'state_norm_proxy', 0, ...
                'com_speed_mag', cfg.V_REQ_FIXED, 'terminal_reason', '', ...
                'battery', struct('margin_norm', 0.75));
            [rewardAtTarget, targetInfo] = compute_rl_reward(window, cfg);
            window.window_distance_m = 2*cfg.MISSION.WINDOW_TARGET_M;
            rewardAboveTarget = compute_rl_reward(window, cfg);
            testCase.verifyGreaterThan(rewardAboveTarget, rewardAtTarget);
            testCase.verifyEqual(targetInfo.risk_r2, 0);
            window.terminal_reason = 'infeasible';
            terminalReward = compute_rl_reward(window, cfg);
            testCase.verifyGreaterThan(rewardAboveTarget-terminalReward, 70);
        end
    end
end
