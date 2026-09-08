classdef testPhase3PolicyEvaluation < matlab.unittest.TestCase
    properties (TestParameter)
        Checkpoint = {'Agent74','Agent75'}
    end
    methods (Test)
        function immutableActorIsDeterministic(testCase,Checkpoint)
            policy = load_phase3_policy(Checkpoint);
            parameters = getLearnableParameters(policy.actor);
            randomState = rng;
            first = phase3_policy_candidate(policy,zeros(19,1),0,struct());
            second = phase3_policy_candidate(policy,zeros(19,1),0,struct());
            testCase.verifyEqual(first,second);
            testCase.verifyEqual(size(first),[5,1]);
            testCase.verifyTrue(all(isfinite(first)));
            testCase.verifyEqual(getLearnableParameters(policy.actor),parameters);
            testCase.verifyEqual(rng,randomState);
            testCase.verifyEqual(sha256_file(policy.checkpoint_path),policy.checkpoint_sha256);
        end
        function rejectsUnknownPolicy(testCase)
            testCase.verifyError(@() load_phase3_policy('Agent74_warmstart'), ...
                'load_phase3_policy:UnknownPolicy');
        end
        function baselineUsesSharedProfile(testCase)
            cfg = struct('MISSION',struct('D_TARGET_M',320),'MISSION_DURATION',600, ...
                'V_MIN',0.3,'V_MAX',1.1,'GAMMA_A_MIN',0.1);
            policy = load_phase3_policy('mission_average');
            action = phase3_policy_candidate(policy,zeros(19,1),0,cfg);
            testCase.verifyEqual(action,baseline_supervisory_action('mission_average',0,cfg));
        end
        function targetTimeIsBracketed(testCase)
            steps = struct('integrated',true,'distance_before',0,'distance_after',2, ...
                'time_before',10,'time_after',10.01);
            current = struct('time',zeros(0,1),'total',zeros(0,1));
            result = phase3_target_crossing(steps,1,current,struct());
            testCase.verifyTrue(result.detected);
            testCase.verifyEqual(result.time_linear_s,10.005,'AbsTol',1e-12);
            testCase.verifyEqual(result.time_bracket_s,[10,10.01]);
            testCase.verifyFalse(result.sampled_energy_available);
            testCase.verifyTrue(isnan(result.soc_pct));
        end
        function failedSolveCannotClaimTargetCrossing(testCase)
            steps = struct('integrated',false,'distance_before',0,'distance_after',2, ...
                'time_before',10,'time_after',10);
            result = phase3_target_crossing(steps,1,struct('time',[],'total',[]),struct());
            testCase.verifyFalse(result.detected);
        end
        function noEnergyExtrapolationPastLastSample(testCase)
            steps = struct('integrated',true,'distance_before',0,'distance_after',2, ...
                'time_before',10,'time_after',10.01);
            current = struct('time',[9;10],'total',[1;1]);
            result = phase3_target_crossing(steps,1,current,struct());
            testCase.verifyTrue(result.detected);
            testCase.verifyFalse(result.sampled_energy_available);
            testCase.verifyTrue(isnan(result.charge_As));
        end
        function integratesInterpolatedCurrentAtCrossing(testCase)
            source = bootstrap_RF_MPC_RL();
            saved = load(fullfile(source,'rlEnv_MPC_R.mat'),'cfg');
            saved.cfg.BATTERY.decim = 1;
            steps = struct('integrated',true,'distance_before',0,'distance_after',4, ...
                'time_before',0,'time_after',4);
            times = (0:0.1:4).';
            current = struct('time',times,'total',1+times);
            result = phase3_target_crossing(steps,2.05,current,saved.cfg.BATTERY);
            testCase.verifyTrue(result.sampled_energy_available);
            testCase.verifyEqual(result.charge_As,4.15125,'AbsTol',1e-12);
            testCase.verifyTrue(isfinite(result.soc_pct));
        end
    end
end
