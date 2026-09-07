classdef testReferencePositionContinuity < matlab.unittest.TestCase
    methods (Test)
        function cancelsOnlyCommandInducedPositionJump(testCase)
            state = struct('t',50,'Xt',999*ones(30,1),'reference_state', ...
                struct('v_cmd',320/600,'a_cmd',(320/600)/2,'position_offset_m',0));
            control = struct('v_cmd',320/600+0.024,'a_cmd',(320/600+0.024)/2, ...
                'reference_mode','position_continuous_v1');
            [reference,offset] = update_reference_position_offset(state,control);
            testCase.verifyEqual(offset,-1.176,'AbsTol',1e-12);
            testCase.verifyEqual(reference.v_cmd,control.v_cmd);
            testCase.verifyEqual(reference.a_cmd,control.a_cmd);
        end
        function unchangedCommandDoesNotAccumulateOffsets(testCase)
            state = struct('t',55,'reference_state',struct('v_cmd',0.56, ...
                'a_cmd',0.28,'position_offset_m',-1.176));
            control = struct('v_cmd',0.56,'a_cmd',0.28,'reference_mode','position_continuous_v1');
            [~,offset] = update_reference_position_offset(state,control);
            testCase.verifyEqual(offset,-1.176,'AbsTol',1e-12);
        end
        function legacyModeKeepsZeroOffset(testCase)
            state = struct('t',50);
            control = struct('v_cmd',0.56,'a_cmd',0.28);
            [~,offset] = update_reference_position_offset(state,control);
            testCase.verifyEqual(offset,0);
        end
        function nonzeroTimeRequiresReferenceHistory(testCase)
            testCase.verifyError(@() update_reference_position_offset(struct('t',50), ...
                struct('v_cmd',0.56,'a_cmd',0.28,'reference_mode','position_continuous_v1')), ...
                'update_reference_position_offset:MissingHistory');
        end
    end
end
