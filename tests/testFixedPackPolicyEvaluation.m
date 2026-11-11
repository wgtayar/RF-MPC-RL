classdef testFixedPackPolicyEvaluation < matlab.unittest.TestCase
    methods (Test)
        function freshCheckoutEvaluatesWithoutCreatingWorkerBundle(tc)
            source=bootstrap_RF_MPC_RL();
            worker=fullfile(source,'rlEnv_MPC_R.mat');
            tc.assumeFalse(isfile(worker),'This regression exercises the fresh-checkout path.');
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            root=fullfile(fixture.Folder,'evaluation');
            before=rng;
            result=run_phase3_policy_evaluation('mission_average',root,struct('max_decisions',1));
            tc.verifyEqual(rng,before);
            tc.verifyFalse(isfile(worker));
            tc.verifyEqual(result.configuration_origin,'fresh_setup');
            tc.verifyEqual(result.options.reference_mode,'velocity_continuous_v2');
            tc.verifyEqual(result.options.solver_strategy,'default_one_shot_fallback_tight_primal_v1');
            tc.verifyEqual(result.options.mission_end_mode,'mpc_step_target_v1');
            tc.verifyTrue(result.checkpoint_unchanged);
            tc.verifyGreaterThan(result.validation.rows,0);
            tc.verifyEqual(result.metrics.target_crossing.schema,'fixed_pack_target_crossing_v2');
            cfg=load(fullfile(root,'manifest','configuration.mat'),'cfg');
            tc.verifyEqual(cfg.cfg.POLICY_EVALUATION.configuration_origin,'fresh_setup');
            tc.verifyFalse(cfg.cfg.PHASE3.training_promoted);
        end
    end
end
