classdef testPhase3Baselines < matlab.unittest.TestCase
    properties (TestParameter)
        DecisionTime = {0,100,600}
    end
    properties
        Config = struct('MISSION',struct('D_TARGET_M',320),'MISSION_DURATION',600, ...
            'V_MIN',0.3,'V_MAX',1.1,'GAMMA_V_MAX',0.45,'GAMMA_A_MIN',0.1)
    end
    methods (Test)
        function highAccelerationUsesExplicitBounds(testCase,DecisionTime)
            cfg = testCase.Config;
            cfg.GAMMA_A_MAX = 0.47;
            policy = load_phase3_policy('fast_start_high_acceleration');
            action = phase3_policy_candidate(policy,zeros(19,1),DecisionTime,cfg);
            testCase.verifyEqual(action,[0;0;0;cfg.GAMMA_V_MAX;cfg.GAMMA_A_MAX]);
            testCase.verifyEqual(policy.kind,'baseline');
            testCase.verifyEmpty(policy.actor);
            testCase.verifyEmpty(policy.checkpoint_path);
        end
        function originalFastStartRemainsMinimumAcceleration(testCase)
            action = baseline_supervisory_action('fast_start',0,testCase.Config);
            testCase.verifyEqual(action,[0;0;0;0.45;0.1]);
        end
        function missionAverageUsesPhysicalTarget(testCase)
            action = baseline_supervisory_action('mission_average',0,testCase.Config);
            testCase.verifyEqual(0.3+0.8*action(4),320/600,'AbsTol',1e-12);
            testCase.verifyEqual(action(1:3),zeros(3,1));
        end
        function slowStartHasExplicitSwitch(testCase)
            early = baseline_supervisory_action('slow_start',99,testCase.Config);
            later = baseline_supervisory_action('slow_start',100,testCase.Config);
            testCase.verifyEqual(early(4),0.2);
            testCase.verifyEqual(later(4),0.4);
        end
        function rampStopsAtConfiguredProfileEnd(testCase)
            early = baseline_supervisory_action('hand_ramp',0,testCase.Config);
            later = baseline_supervisory_action('hand_ramp',600,testCase.Config);
            testCase.verifyEqual(early(4),0.2);
            testCase.verifyEqual(later(4),0.45,'AbsTol',1e-12);
        end
        function rejectsUnknownPolicy(testCase)
            testCase.verifyError(@() baseline_supervisory_action('unknown',0,testCase.Config), ...
                'baseline_supervisory_action:UnknownPolicy');
        end
    end
end
