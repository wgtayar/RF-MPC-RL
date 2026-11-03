classdef testFixedPackTargetCrossing < matlab.unittest.TestCase
    methods (Test)
        function validCrossingInterpolatesRecordedEndpoints(tc)
            [steps,current] = localFixture();
            result = phase3_target_crossing(steps,2,current,struct());
            tc.verifyTrue(result.detected);
            tc.verifyTrue(result.geometric_crossing.detected);
            tc.verifyEqual(result.qualification,'actuator_feasible');
            tc.verifyEqual(result.time_linear_s,.015,'AbsTol',1e-14);
            tc.verifyEqual(result.time_bracket_s,[.01,.02]);
            tc.verifyEqual(result.charge_As,4);
            tc.verifyEqual(result.energy_Wh,.02,'AbsTol',1e-14);
            tc.verifyEqual(result.soc_pct,94.98,'AbsTol',1e-12);
            tc.verifyTrue(result.sampled_energy_available);
        end

        function simultaneousVoltageViolationIsOnlyGeometric(tc)
            [steps,current] = localFixture();
            steps.voltage_limit_steps(2)=1;
            result = phase3_target_crossing(steps,2,current,struct());
            localVerifyInvalid(tc,result,'actuator_limit');
            % The metric and environment must agree at simultaneous arrival
            % and actuator failure; mission-target precedence cannot hide it.
            state=struct('Xt',zeros(30,1),'Ut',zeros(12,1),'t',.02, ...
                'battery',struct('margin_norm',.95,'soc_pct',95, ...
                'voltage_limit_steps',1,'torque_limit_steps',0));
            [reason,done]=resolve_phase3_terminal(state, ...
                struct('terminal_reason','mission_target_reached'),2,struct());
            tc.verifyEqual(reason,'actuator_limit');
            tc.verifyTrue(done);
        end

        function simultaneousTorqueViolationIsOnlyGeometric(tc)
            [steps,current] = localFixture();
            steps.torque_limit_steps(2)=1;
            localVerifyInvalid(tc,phase3_target_crossing(steps,2,current,struct()),'actuator_limit');
        end

        function earlierViolationCannotBeForgottenAtCrossing(tc)
            [steps,current] = localFixture();
            steps.voltage_limit_steps(:)=1;
            localVerifyInvalid(tc,phase3_target_crossing(steps,2,current,struct()),'actuator_limit');
        end

        function missingEnergyIsNotQualified(tc)
            [steps,current] = localFixture();
            steps.energy_committed(2)=false;
            localVerifyInvalid(tc,phase3_target_crossing(steps,2,current,struct()),'energy_not_committed');
        end

        function unintegratedAttemptDoesNotCross(tc)
            [steps,current] = localFixture();
            steps.integrated(2)=false;
            steps.energy_committed(2)=false;
            result=phase3_target_crossing(steps,2,current,struct());
            tc.verifyFalse(result.detected);
            tc.verifyFalse(result.geometric_crossing.detected);
            tc.verifyEqual(result.qualification,'no_geometric_crossing');
        end

        function missingFeasibilityEvidenceFailsClosed(tc)
            [steps,current] = localFixture();
            steps=rmfield(steps,'voltage_limit_steps');
            tc.verifyError(@()phase3_target_crossing(steps,2,current,struct()), ...
                'phase3_target_crossing:Feasibility');
        end

        function invalidLimitCountsFailClosed(tc)
            [steps,current] = localFixture();
            steps.voltage_limit_steps(2)=NaN;
            tc.verifyError(@()phase3_target_crossing(steps,2,current,struct()), ...
                'phase3_target_crossing:Feasibility');
        end

        function nonfiniteLedgerCannotClaimEnergy(tc)
            [steps,current] = localFixture();
            current.ledger(2,4)=NaN;
            tc.verifyError(@()phase3_target_crossing(steps,2,current,struct()), ...
                'phase3_target_crossing:Ledger');
        end

        function legacyGeometryRetainsHistoricalMeaning(tc)
            [steps,~] = localFixture();
            steps=rmfield(steps,{'voltage_limit_steps','torque_limit_steps','energy_committed'});
            current=struct('time',zeros(0,1),'total',zeros(0,1));
            result=phase3_target_crossing(steps,2,current,struct());
            tc.verifyTrue(result.detected);
            tc.verifyEqual(result.time_linear_s,.015,'AbsTol',1e-14);
            tc.verifyFalse(result.sampled_energy_available);
            tc.verifyFalse(isfield(result,'qualification'));
            tc.verifyFalse(isfield(result,'geometric_crossing'));
        end
    end
end

function [steps,current]=localFixture()
% Synthetic accounting endpoints; these are not simulated physical evidence.
steps=struct('integrated',true(2,1),'distance_before',[0;1],'distance_after',[1;3], ...
    'time_before',[0;.01],'time_after',[.01;.02], ...
    'voltage_limit_steps',zeros(2,1),'torque_limit_steps',zeros(2,1), ...
    'energy_committed',true(2,1));
current=struct('time',zeros(0,1),'total',zeros(0,1), ...
    'ledger',[0 2 0 .01 95 94.99;2 6 .01 .03 94.99 94.97]);
end

function localVerifyInvalid(tc,result,reason)
tc.verifyFalse(result.detected);
tc.verifyTrue(result.geometric_crossing.detected);
tc.verifyEqual(result.geometric_crossing.time_linear_s,.015,'AbsTol',1e-14);
tc.verifyEqual(result.geometric_crossing.time_bracket_s,[.01,.02]);
tc.verifyEqual(result.qualification,reason);
tc.verifyTrue(all(isnan([result.time_linear_s,result.charge_As,result.energy_Wh,result.soc_pct])));
tc.verifyFalse(result.sampled_energy_available);
end
