classdef testExactQpRebuild < matlab.unittest.TestCase
    methods (Test)
        function identicalIncludesFsm(testCase)
            [actual,qp,row] = localFixture();
            comparison = compare_exact_qp_rebuild(actual,qp,row);
            testCase.verifyTrue(comparison.exact_match);
            testCase.verifyEmpty(comparison.mismatched_fields);
        end
        function hiddenFsmMismatchFails(testCase)
            [actual,qp,row] = localFixture();
            actual.fsm_internal_state.timer = 1;
            comparison = compare_exact_qp_rebuild(actual,qp,row);
            testCase.verifyFalse(comparison.exact_match);
            testCase.verifyEqual(comparison.mismatched_fields,{'fsm_internal_state'});
        end
        function tinyQpMismatchFails(testCase)
            [actual,qp,row] = localFixture();
            actual.H(1) = actual.H(1)+eps;
            comparison = compare_exact_qp_rebuild(actual,qp,row);
            testCase.verifyFalse(comparison.exact_match);
            testCase.verifyEqual(comparison.mismatched_fields,{'H'});
        end
    end
end

function [actual,qp,row] = localFixture()
    problem = struct('H',eye(2),'g',ones(2,1),'Aineq',zeros(0,2), ...
        'bineq',zeros(0,1),'Aeq',[1 1],'beq',0);
    qp = struct('problem',problem,'Xt',zeros(30,1),'Ut',zeros(12,1), ...
        'Xd',zeros(30,1),'Ud',zeros(12,1));
    row = struct('FSM',ones(4,1),'fsm_after',struct('timer',0));
    actual = problem;
    actual.Xt = qp.Xt;
    actual.Ut = qp.Ut;
    actual.Xd = qp.Xd;
    actual.Ud = qp.Ud;
    actual.FSM = row.FSM;
    actual.fsm_internal_state = row.fsm_after;
end
