classdef testPhase3InitialCondition < matlab.unittest.TestCase
    properties
        Bundle
        State
        Condition
        Folder
        Metadata
    end
    methods (TestMethodSetup)
        function configure(testCase)
            source = bootstrap_RF_MPC_RL();
            testCase.Bundle = load(fullfile(source,'rlEnv_MPC_R.mat'),'cfg','initial_R','lower_abs','upper_abs');
            testCase.State = initialize_mpc_replay_state(testCase.Bundle.cfg,0);
            testCase.Condition = struct('schema','reset_body_perturbation_v1','id','unit_pitch', ...
                'rotation_vector_body_rad',[0;0.1;0], ...
                'angular_velocity_body_delta_rad_s',[0;0.2;0], ...
                'linear_velocity_world_delta_m_s',[0;0;0]);
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            testCase.Folder = fixture.Folder;
            testCase.Metadata = struct('run_id','initial_condition_unit','run_type','unit_no_mpc', ...
                'monitor_root',source,'seed',testCase.Bundle.cfg.RNG_SEED,'source_policy','none');
        end
    end
    methods (Test)
        function zeroPerturbationPreservesExactState(testCase)
            condition = testCase.Condition;
            condition.rotation_vector_body_rad(:) = 0;
            condition.angular_velocity_body_delta_rad_s(:) = 0;
            [actual,audit] = apply_phase3_initial_condition(testCase.State,condition);
            testCase.verifyEqual(actual,testCase.State);
            testCase.verifyEqual(audit.Xt_before,audit.Xt_after);
        end
        function rotationUsesBodyChartAndKeepsWorldFeet(testCase)
            state = testCase.State;
            yaw = [cos(0.3),-sin(0.3),0;sin(0.3),cos(0.3),0;0,0,1];
            pitch = [cos(0.1),0,sin(0.1);0,1,0;-sin(0.1),0,cos(0.1)];
            state.Xt(7:15) = yaw(:);
            actual = apply_phase3_initial_condition(state,testCase.Condition);
            rotation = reshape(actual.Xt(7:15),3,3);
            testCase.verifyEqual(rotation,yaw*pitch,AbsTol=1e-12);
            testCase.verifyEqual(rotation.'*rotation,eye(3),AbsTol=1e-12);
            testCase.verifyEqual(det(rotation),1,AbsTol=1e-12);
            testCase.verifyEqual(actual.Xt([1:3,19:30]),state.Xt([1:3,19:30]));
            testCase.verifyEqual(actual.Ut,state.Ut);
        end
        function velocityOffsetsAreExplicit(testCase)
            condition = testCase.Condition;
            condition.linear_velocity_world_delta_m_s = [0.1;-0.2;0.05];
            actual = apply_phase3_initial_condition(testCase.State,condition);
            testCase.verifyEqual(actual.Xt(4:6)-testCase.State.Xt(4:6),condition.linear_velocity_world_delta_m_s,AbsTol=1e-12);
            testCase.verifyEqual(actual.Xt(16:18)-testCase.State.Xt(16:18),condition.angular_velocity_body_delta_rad_s,AbsTol=1e-12);
            testCase.verifyEqual(rmfield(actual,'Xt'),rmfield(testCase.State,'Xt'));
        end
        function progressedStateRejected(testCase)
            state = testCase.State;
            state.t = 0.01;
            testCase.verifyError(@() apply_phase3_initial_condition(state,testCase.Condition),'phase3InitialCondition:NotFresh');
        end
        function initializedFsmRejected(testCase)
            state = testCase.State;
            state.fsm_internal_state.FSM = ones(4,1);
            testCase.verifyError(@() apply_phase3_initial_condition(state,testCase.Condition),'phase3InitialCondition:NotFresh');
        end
        function alreadyResetStateCannotBePerturbedAgain(testCase)
            state = testCase.State;
            state.decision_bookkeeping.episode = 1;
            testCase.verifyError(@() apply_phase3_initial_condition(state,testCase.Condition),'phase3InitialCondition:NotFresh');
        end
        function nonscalarSchemaRejected(testCase)
            condition = testCase.Condition;
            condition.schema = ["reset_body_perturbation_v1","reset_body_perturbation_v1"];
            testCase.verifyError(@() validate_phase3_initial_condition(condition),'phase3InitialCondition:Schema');
        end
        function invalidRotationNotSilentlyRepaired(testCase)
            state = testCase.State;
            state.Xt(7) = 2;
            testCase.verifyError(@() apply_phase3_initial_condition(state,testCase.Condition),'phase3InitialCondition:Rotation');
        end
        function unknownFieldsRejectedBeforeCapture(testCase)
            condition = testCase.Condition;
            condition.unrecorded_force = 10;
            root = fullfile(testCase.Folder,'invalid');
            testCase.verifyError(@() Phase3MpcEnvironment(testCase.Bundle,root,testCase.Metadata, ...
                struct('initial_condition',condition)),'phase3InitialCondition:Schema');
            testCase.verifyFalse(isfolder(root));
        end
        function resetRecordsAndRepeatsCondition(testCase)
            root = fullfile(testCase.Folder,'reset');
            env = Phase3MpcEnvironment(testCase.Bundle,root,testCase.Metadata,struct('initial_condition',testCase.Condition));
            reset(env);
            first = env.State.Xt;
            reset(env);
            testCase.verifyEqual(env.State.Xt,first);
            testCase.verifyEqual(env.Config.PHASE3.options.initial_condition,testCase.Condition);
            testCase.verifyEqual(env.Writer.RowCount,0);
            close(env,'unit_test');
            recorded = load(fullfile(root,'episodes','episode_000001_start.mat'),'event');
            testCase.verifyEqual(recorded.event.record.initial_condition_audit.Xt_after,first);
            testCase.verifyEqual(recorded.event.record.initial_condition_audit.condition,testCase.Condition);
            report = validate_exact_state_dataset(root);
            testCase.verifyEqual(report.supervisory_records,4);
        end
        function oldResetAliasesStatesButV2Distinguishes(testCase)
            nominal = Phase3MpcEnvironment(testCase.Bundle,fullfile(testCase.Folder,'nominal'),testCase.Metadata);
            changed = Phase3MpcEnvironment(testCase.Bundle,fullfile(testCase.Folder,'changed'),testCase.Metadata, ...
                struct('initial_condition',testCase.Condition));
            oldA = reset(nominal);
            oldB = reset(changed);
            newA = build_phase3_observation_v2(nominal.State,oldA,struct(),nominal.Config,'observation_v2_dynamic_health_candidate_v2');
            newB = build_phase3_observation_v2(changed.State,oldB,struct(),changed.Config,'observation_v2_dynamic_health_candidate_v2');
            testCase.verifyEqual(oldA,oldB);
            testCase.verifyNotEqual(newA,newB);
            testCase.verifyFalse(isfield(nominal.Config.PHASE3.options,'initial_condition'));
            close(nominal,'unit_test');
            close(changed,'unit_test');
        end
        function historicalActorRejectsV2BeforeCapture(testCase)
            root = fullfile(testCase.Folder,'actor');
            testCase.verifyError(@() run_phase3_policy_evaluation('Agent74',root, ...
                struct('observation_schema','observation_v2_dynamic_health_candidate_v2')), ...
                'run_phase3_policy_evaluation:LegacyObservation');
            testCase.verifyFalse(isfolder(root));
        end
    end
end
