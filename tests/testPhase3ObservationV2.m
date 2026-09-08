classdef testPhase3ObservationV2 < matlab.unittest.TestCase
    properties
        Bundle
        State
        Row
        Legacy
        Folder
    end
    methods (TestMethodSetup)
        function setup(testCase)
            source = bootstrap_RF_MPC_RL();
            testCase.Bundle = load(fullfile(source,'rlEnv_MPC_R.mat'),'cfg','initial_R','lower_abs','upper_abs');
            state = initialize_mpc_replay_state(testCase.Bundle.cfg,0);
            state.t = 0.2;
            state.decision_bookkeeping.initial_position_x = 0;
            state.fsm_internal_state = struct('FSM',[1;2;2;1],'Ta',zeros(4,1),'Tb',0.3*ones(4,1));
            testCase.State = state;
            testCase.Row = struct('Xd',state.Xt,'time_after',state.t,'state_after',state,'action_execution', ...
                struct('candidate_action',[0;0;0;0.45;0.2],'applied_action',[0;0;0;0.32;0.2]), ...
                'solver',struct('iterations',14,'wall_time_s',0.015,'classification','solver_success', ...
                'diagnostics',struct('inequality_margin_min',0)));
            testCase.Legacy = zeros(19,1);
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            testCase.Folder = fixture.Folder;
        end
    end
    methods (Test)
        function versionedFiniteDimension(testCase)
            [observation,audit] = build_phase3_observation_v2(testCase.State,testCase.Legacy,testCase.Row,testCase.Bundle.cfg);
            testCase.verifySize(observation,[77 1]);
            testCase.verifyTrue(all(isfinite(observation)));
            testCase.verifyFalse(audit.schema.training_promoted);
            testCase.verifyFalse(any(contains(audit.schema.names,{'state_norm_proxy','fsm_proxy'})));
        end
        function invariantToGlobalTranslation(testCase)
            shifted = testCase.State;
            shifted.Xt(1) = shifted.Xt(1)+1000;
            shifted.Xt([19 22 25 28]) = shifted.Xt([19 22 25 28])+1000;
            shifted.decision_bookkeeping.initial_position_x = 1000;
            shiftedRow = testCase.Row;
            shiftedRow.state_after = shifted;
            original = build_phase3_observation_v2(testCase.State,testCase.Legacy,testCase.Row,testCase.Bundle.cfg);
            actual = build_phase3_observation_v2(shifted,testCase.Legacy,shiftedRow,testCase.Bundle.cfg);
            testCase.verifyEqual(actual,original,AbsTol=1e-12);
        end
        function legacyBadProxiesIgnored(testCase)
            legacy = testCase.Legacy;
            legacy([5 13 14 15 16 17 18]) = 100;
            original = build_phase3_observation_v2(testCase.State,testCase.Legacy,testCase.Row,testCase.Bundle.cfg);
            actual = build_phase3_observation_v2(testCase.State,legacy,testCase.Row,testCase.Bundle.cfg);
            testCase.verifyEqual(actual,original);
        end
        function distinguishesLegsAndRawCandidate(testCase)
            row = testCase.Row;
            row.action_execution.candidate_action(4) = 0.4;
            state = testCase.State;
            state.fsm_internal_state.FSM(2) = 1;
            original = build_phase3_observation_v2(testCase.State,testCase.Legacy,testCase.Row,testCase.Bundle.cfg);
            actual = build_phase3_observation_v2(state,testCase.Legacy,row,testCase.Bundle.cfg);
            testCase.verifyNotEqual(actual,original);
        end
        function healthDoesNotClipAtOne(testCase)
            state = testCase.State;
            state.Xt(16) = 100;
            row = testCase.Row;
            row.state_after = state;
            [actual,audit] = build_phase3_observation_v2(state,testCase.Legacy,row,testCase.Bundle.cfg);
            testCase.verifyGreaterThan(actual(audit.schema.names=="omega_x"),1);
            testCase.verifyFalse(any(audit.clipped));
        end
        function missingHistoryIsNotInvented(testCase)
            testCase.verifyError(@() build_phase3_observation_v2(testCase.State,testCase.Legacy,struct(),testCase.Bundle.cfg), ...
                'observationV2:MissingHistory');
        end
        function invalidDynamicsFlaggedAndFinite(testCase)
            state = testCase.State;
            state.Xt(16) = NaN;
            row = testCase.Row;
            row.state_after = state;
            [actual,audit] = build_phase3_observation_v2(state,testCase.Legacy,row,testCase.Bundle.cfg);
            testCase.verifyTrue(all(isfinite(actual)));
            testCase.verifyEqual(actual(audit.schema.names=="dynamic_state_valid"),0);
            testCase.verifyFalse(audit.available(audit.schema.names=="omega_x"));
        end
        function resetMissingContextsAreExplicit(testCase)
            state = testCase.State;
            state.t = 0;
            state.fsm_internal_state = struct();
            [actual,audit] = build_phase3_observation_v2(state,testCase.Legacy,struct(),testCase.Bundle.cfg);
            testCase.verifyEqual(actual(startsWith(audit.schema.names,"has_")),zeros(4,1));
            testCase.verifyFalse(audit.available(audit.schema.names=="velocity_error_x"));
        end
        function environmentSelectsV2WithoutChangingLegacy(testCase)
            metadata = struct('run_id','v2_unit','run_type','unit_no_rollout','monitor_root',pwd, ...
                'seed',testCase.Bundle.cfg.RNG_SEED,'source_policy','none');
            legacy = Phase3MpcEnvironment(testCase.Bundle,fullfile(testCase.Folder,'legacy'),metadata);
            candidate = Phase3MpcEnvironment(testCase.Bundle,fullfile(testCase.Folder,'v2'),metadata, ...
                struct('observation_schema','observation_v2_dynamic_health_candidate_v1'));
            testCase.verifySize(reset(legacy),[19 1]);
            testCase.verifySize(reset(candidate),[77 1]);
            testCase.verifyEqual(candidate.Config.PHASE3.observation_schema,'observation_v2_dynamic_health_candidate_v1');
            close(legacy,'unit_test');
            close(candidate,'unit_test');
        end
        function futureRowRejected(testCase)
            row = testCase.Row;
            row.time_after = testCase.State.t+0.01;
            testCase.verifyError(@() build_phase3_observation_v2(testCase.State,testCase.Legacy,row,testCase.Bundle.cfg), ...
                'observationV2:Endpoint');
        end
    end
end
