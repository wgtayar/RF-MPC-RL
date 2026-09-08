classdef testPhase3EnvironmentContract < matlab.unittest.TestCase
    properties
        Bundle
        Metadata
        Folder
    end
    methods (TestMethodSetup)
        function configure(testCase)
            source = bootstrap_RF_MPC_RL();
            testCase.Bundle = load(fullfile(source,'rlEnv_MPC_R.mat'), ...
                'cfg','initial_R','lower_abs','upper_abs');
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            testCase.Folder = fullfile(fixture.Folder,'dataset');
            testCase.Metadata = struct('run_id','environment_contract_unit', ...
                'run_type','unit_test','monitor_root',source, ...
                'seed',testCase.Bundle.cfg.RNG_SEED,'source_policy','no_mpc_rollout');
        end
    end
    methods (Test)
        function constructorDoesNotRunThePlant(testCase)
            env = Phase3MpcEnvironment(testCase.Bundle,testCase.Folder,testCase.Metadata);
            testCase.verifyEqual(env.Writer.RowCount,0);
            testCase.verifyEqual(env.Episode,0);
            testCase.verifyError(@() step(env,zeros(5,1)),'Phase3MpcEnvironment:ResetRequired');
            close(env,'unit_test_complete');
        end
        function resetLifecycleHasUniqueEpisodes(testCase)
            env = Phase3MpcEnvironment(testCase.Bundle,testCase.Folder,testCase.Metadata);
            first = reset(env);
            second = reset(env);
            close(env,'unit_test_complete');
            report = validate_exact_state_dataset(testCase.Folder);
            testCase.verifyEqual(first,second);
            testCase.verifyEqual(env.Episode,2);
            testCase.verifyEqual(report.rows,0);
            testCase.verifyEqual(report.supervisory_records,4);
            testCase.verifyError(@() reset(env),'Phase3MpcEnvironment:Closed');
        end
        function randomRequestsUseAnIsolatedStream(testCase)
            bundle = testCase.Bundle;
            bundle.cfg.RANDOMIZE_REQUEST = true;
            state = rng;
            env = Phase3MpcEnvironment(bundle,testCase.Folder,testCase.Metadata);
            reset(env);
            firstRequest = env.State.supervisory_state.v_req;
            reset(env);
            secondRequest = env.State.supervisory_state.v_req;
            close(env,'unit_test_complete');
            testCase.verifyEqual(rng,state);
            testCase.verifyNotEqual(firstRequest,secondRequest);
        end
        function rejectsSubstepChunk(testCase)
            bundle = testCase.Bundle;
            bundle.cfg.CHUNK_DURATION = 0.003;
            testCase.verifyError(@() Phase3MpcEnvironment(bundle,testCase.Folder,testCase.Metadata), ...
                'Phase3MpcEnvironment:DurationGrid');
            testCase.verifyFalse(isfolder(testCase.Folder));
        end
        function requiresFreshRunForResume(testCase)
            env = Phase3MpcEnvironment(testCase.Bundle,testCase.Folder,testCase.Metadata);
            reset(env);
            testCase.verifyError(@() resumeFromDecision(env,'unused',1,1), ...
                'Phase3MpcEnvironment:ResumeRequiresNewRun');
            close(env,'unit_test_complete');
        end
        function recoveredEventsDoNotReceiveFailurePenalty(testCase)
            cfg = testCase.Bundle.cfg;
            window = testCase.windowFixture();
            [baseline,~] = compute_rl_reward(window,cfg);
            window.recovered_solver_events = 1;
            [actual,info] = compute_phase3_reward_bridge(window,cfg);
            testCase.verifyEqual(actual,baseline);
            testCase.verifyEqual(info.recovered_solver_events,1);
            testCase.verifyFalse(info.training_promoted);
        end
        function rewardBridgeRetainsActualTerminalClass(testCase)
            window = testCase.windowFixture();
            window.terminal_reason = 'numerical_solver_failure_unrecovered';
            [reward,info] = compute_phase3_reward_bridge(window,testCase.Bundle.cfg);
            testCase.verifyTrue(isfinite(reward));
            testCase.verifyEqual(info.actual_terminal_reason,'numerical_solver_failure_unrecovered');
            testCase.verifyEqual(info.legacy_formula_terminal_category,'infeasible');
            testCase.verifyEqual(info.version,'reward_phase3_legacy_bridge_v1');
        end
    end
    methods (Static,Access = private)
        function window = windowFixture()
            window = struct('tracking_error_mean',1,'control_effort_mean',1, ...
                'Ieq_window',30,'soc_start_pct',95,'soc_end_pct',94,'lag_frac',0, ...
                'time_frac',1/12,'progress_frac',1/12,'distance_start_m',0, ...
                'window_distance_m',320/12,'v_exec',0.5333,'a_exec',0.2667, ...
                'delta_v_exec',0,'delta_gamma_v',0,'dR2',0,'state_norm_proxy',0, ...
                'com_speed_mag',0.5333,'terminal_reason','', ...
                'battery',struct('margin_norm',0.94),'recovered_solver_events',0);
        end
    end
end
