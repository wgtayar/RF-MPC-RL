classdef testPhase3ViabilityLabels < matlab.unittest.TestCase
    methods (Test)
        function recoveredEndpointDoesNotEraseEarlierDegradation(testCase)
            [initial,out,limits] = testCase.fixture([0.2;1.3;0.2]);
            [label,a] = classify_dynamic_viability_v2(initial,out,limits);
            testCase.verifyEqual(label,"degraded_recovering");
            testCase.verifyTrue(a.recovery_candidate);
            testCase.verifyFalse(a.within_envelope_at_horizon);
            testCase.verifyFalse(a.safe_for_policy);
        end
        function provisionalHealthyIsNotPolicyApproval(testCase)
            [initial,out,limits] = testCase.fixture([0.2;0.3;0.2]);
            [label,a] = classify_dynamic_viability_v2(initial,out,limits);
            testCase.verifyEqual(label,"healthy_robust");
            testCase.verifyTrue(a.within_envelope_at_horizon);
            testCase.verifyFalse(a.safe_for_policy);
        end
        function nearBoundaryIsMarginal(testCase)
            [initial,out,limits] = testCase.fixture([0.2;0.9;0.2]);
            label = classify_dynamic_viability_v2(initial,out,limits);
            testCase.verifyEqual(label,"marginal");
        end
        function sustainedDegradationIsNotRecovery(testCase)
            [initial,out,limits] = testCase.fixture([1.2;1.4;1.5]);
            [label,a] = classify_dynamic_viability_v2(initial,out,limits);
            testCase.verifyEqual(label,"degraded_not_recovering");
            testCase.verifyFalse(a.recovery_candidate);
        end
        function initialDegradationIsIncluded(testCase)
            [initial,out,limits] = testCase.fixture([0.2;0.2;0.2]);
            initial.orientation = 1.5;
            label = classify_dynamic_viability_v2(initial,out,limits);
            testCase.verifyEqual(label,"degraded_recovering");
        end
        function missingSolverEvidenceIsNotHealthy(testCase)
            [initial,out,limits] = testCase.fixture([0.2;0.2;0.2]);
            out.trace.kkt_stationarity_inf(2) = NaN;
            label = classify_dynamic_viability_v2(initial,out,limits);
            testCase.verifyEqual(label,"insufficient_data");
        end
        function shortSurvivalDoesNotSatisfyRequestedHorizon(testCase)
            [initial,out,limits] = testCase.fixture([0.2;0.2;0.2]);
            out.requested_duration_s = 5;
            label = classify_dynamic_viability_v2(initial,out,limits);
            testCase.verifyEqual(label,"insufficient_data");
        end
        function clippedTraceCannotClaimTheWholeHorizon(testCase)
            [initial,out,limits] = testCase.fixture([0.2;0.2;0.2]);
            out.trace = out.trace(2:end,:);
            testCase.verifyEqual(classify_dynamic_viability_v2(initial,out,limits), ...
                "insufficient_data");
        end
        function missingStepCannotClaimWholeHorizon(testCase)
            [initial,out,limits] = testCase.fixture([0.2;0.2;0.2]);
            out.trace.time_s(2) = 0.011;
            testCase.verifyEqual(classify_dynamic_viability_v2(initial,out,limits), ...
                "insufficient_data");
        end
        function deadlineStressIsNotDynamicDegradation(testCase)
            [initial,out,limits] = testCase.fixture([0.2;0.2;0.2]);
            out.trace.solver_wall_time_s(2) = 0.02;
            testCase.verifyEqual(classify_dynamic_viability_v2(initial,out,limits), ...
                "healthy_solver_stressed");
        end
        function numericalAndUnknownFailuresRemainDistinct(testCase)
            [initial,out,limits] = testCase.fixture([0.2;0.2;0.2]);
            out.terminal_reason = "numerical_solver_failure";
            testCase.verifyEqual(classify_dynamic_viability_v2(initial,out,limits), ...
                "numerical_solver_failure");
            out.terminal_reason = "unclassified_solver_failure";
            testCase.verifyEqual(classify_dynamic_viability_v2(initial,out,limits), ...
                "unclassified_solver_failure");
        end
    end
    methods (Static, Access = private)
        function [initial,out,limits] = fixture(orientation)
            limits = struct('orientation',1,'angular_velocity',10, ...
                'position_invariant',10,'Ut',100,'solver_iterations',100, ...
                'equality_residual',1e-6,'inequality_margin',-1e-6,'kkt_stationarity',1e-4);
            initial = struct('orientation',orientation(1),'angular_velocity',1, ...
                'position_invariant',1,'Ut',10);
            n = numel(orientation);
            values = [orientation,ones(n,2),10*ones(n,1), ...
                orientation,ones(n,2),10*ones(n,1), ...
                20*ones(n,1),1e-8*ones(n,1),zeros(n,1),1e-8*ones(n,1)];
            trace = array2table(values,'VariableNames', ...
                {'orientation_error_after_rad','angular_velocity_after', ...
                'position_invariant_norm_after','Ut_norm_after', ...
                'orientation_error_before_rad','angular_velocity_before', ...
                'position_invariant_norm_before','Ut_norm_before', ...
                'solver_iterations','equality_residual_max_abs', ...
                'inequality_margin_min','kkt_stationarity_inf'});
            out = struct('trace',trace,'terminal_reason',"horizon_complete", ...
                'requested_duration_s',0.03,'survived_duration_s',0.03, ...
                'mpc_timestep_s',0.01,'initial_state',struct('time_s',0));
            out.trace.time_s = (0:n-1).'*0.01;
            out.trace.solver_wall_time_s = 0.001*ones(n,1);
        end
    end
end
