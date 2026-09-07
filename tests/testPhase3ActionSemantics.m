classdef testPhase3ActionSemantics < matlab.unittest.TestCase
    %testPhase3ActionSemantics Preserve legacy execution and expose map rate limits.

    properties (TestParameter)
        Candidate = struct('high', [-0.0005;-0.0005;0.0005;0.45;0.1], ...
            'low', [0;0;0;0;0.5], 'clipped', [1;-1;1;2;-1], ...
            'interior', [0.0001;0;-0.0002;0.3;0.3])
    end
    properties
        Config
        Previous
        Lower
        Upper
    end
    methods (TestClassSetup)
        function configure(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            testCase.Config = struct('DR_MAX', 0.0005, 'DGAMMA_V_MAX', 0.03, ...
                'GAMMA_V_MIN', 0, 'GAMMA_V_MAX', 0.45, ...
                'GAMMA_A_MIN', 0.1, 'GAMMA_A_MAX', 0.5, ...
                'V_MIN', 0.3, 'V_MAX', 1.1, 'A_MIN', 0.2, 'A_MAX', 1, ...
                'TACC_MIN', 1, 'TACC_MAX', 2);
            testCase.Previous = struct('last_R', [0.1;0.2;0.1], ...
                'prev_gamma_v', 0.29, 'prev_gamma_a', 0.1, ...
                'v_req', 0.8, 'a_req', 1, 'v_exec', 0.532, ...
                'previous_applied_action', [0;0;0;0.29;0.1]);
            testCase.Lower = [0.095;0.19;0.095];
            testCase.Upper = [0.105;0.21;0.105];
        end
    end
    methods (Test)
        function agreesWithFrozenLegacyFormula(testCase, Candidate)
            expected = testPhase3ActionSemantics.legacyFormula( ...
                Candidate, testCase.Previous, testCase.Config, testCase.Lower, testCase.Upper);
            actual = resolve_supervisory_action(Candidate, testCase.Previous, ...
                testCase.Config, testCase.Lower, testCase.Upper);
            testCase.verifyEqual([actual.R_applied; actual.gamma_v_applied; ...
                actual.gamma_a_applied; actual.v_exec; actual.a_exec], expected, AbsTol=1e-14);
        end
        function replayMatchesEnvironmentMapping(testCase, Candidate)
            state = struct('R', testCase.Previous.last_R, ...
                'supervisory_state', testCase.Previous);
            [control, replay] = resolve_replay_action(Candidate, state, ...
                testCase.Config, testCase.Lower, testCase.Upper);
            environment = resolve_supervisory_action(Candidate, testCase.Previous, ...
                testCase.Config, testCase.Lower, testCase.Upper);
            testCase.verifyEqual(replay, environment);
            testCase.verifyEqual(control.v_cmd, environment.v_exec, AbsTol=1e-14);
            testCase.verifyEqual(control.action, environment.applied_action, AbsTol=1e-14);
        end
        function highCandidateIsLimitedByPreviousAppliedGamma(testCase)
            result = resolve_supervisory_action([0;0;0;0.45;0.1], ...
                testCase.Previous, testCase.Config, testCase.Lower, testCase.Upper);
            testCase.verifyEqual(result.gamma_v_raw, 0.45, AbsTol=1e-14);
            testCase.verifyEqual(result.gamma_v_applied, 0.32, AbsTol=1e-14);
            testCase.verifyEqual(result.delta_v_exec, 0.024, AbsTol=1e-14);
        end
        function saturationKeepsCandidateAndEffectiveRDistinct(testCase)
            previous = testCase.Previous;
            previous.last_R = testCase.Upper;
            result = resolve_supervisory_action([0.0005;0.0005;0.0005;0.3;0.1], ...
                previous, testCase.Config, testCase.Lower, testCase.Upper);
            testCase.verifyEqual(result.dR_effective, zeros(3,1), AbsTol=1e-14);
            testCase.verifyEqual(result.dR_clipped, 0.0005*ones(3,1), AbsTol=1e-14);
            testCase.verifyTrue(result.R_saturated);
        end
        function missingHistoryIsRejected(testCase)
            state = struct('R', testCase.Previous.last_R);
            testCase.verifyError(@() resolve_replay_action([0;0;0;0.3;0.1], state, ...
                testCase.Config, testCase.Lower, testCase.Upper), ...
                'resolve_replay_action:MissingActionHistory');
        end
        function tableRetainsRawAndAppliedValues(testCase)
            execution = resolve_supervisory_action([0;0;0;0.45;0.1], ...
                testCase.Previous, testCase.Config, testCase.Lower, testCase.Upper);
            rows = supervisory_action_table({execution});
            testCase.verifySize(rows, [1,30]);
            testCase.verifyEqual(rows.gamma_v_raw, 0.45, AbsTol=1e-14);
            testCase.verifyEqual(rows.gamma_v_applied, 0.32, AbsTol=1e-14);
        end
    end
    methods (Static, Access=private)
        function result = legacyFormula(action, logged, cfg, lowerR, upperR)
            dR = min(max(action(1:3), -cfg.DR_MAX), cfg.DR_MAX);
            gammaV = min(max(action(4), logged.prev_gamma_v-cfg.DGAMMA_V_MAX), ...
                logged.prev_gamma_v+cfg.DGAMMA_V_MAX);
            gammaV = min(max(gammaV, cfg.GAMMA_V_MIN), cfg.GAMMA_V_MAX);
            gammaA = min(max(action(5), cfg.GAMMA_A_MIN), cfg.GAMMA_A_MAX);
            R = min(max(logged.last_R.*(1+dR), lowerR), upperR);
            [v,a] = apply_command_governor(logged.v_req, logged.a_req, gammaV, gammaA,cfg);
            result = [R;gammaV;gammaA;v;a];
        end
    end
end
