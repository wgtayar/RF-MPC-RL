classdef testPhase3TerminalSemantics < matlab.unittest.TestCase
    properties (TestParameter)
        Failure = struct( ...
            'numerical',struct('input','numerical_solver_failure','expected','numerical_solver_failure_unrecovered'), ...
            'mathematical',struct('input','mathematical_constraint_infeasible','expected','mathematical_constraint_infeasible'), ...
            'invalid',struct('input','invalid_state','expected','invalid_state'), ...
            'dynamic',struct('input','dynamic_safety_violation','expected','dynamic_safety_violation'), ...
            'no_action',struct('input','no_safe_action_available','expected','no_safe_action_available'), ...
            'unknown',struct('input','unclassified_solver_failure','expected','unclassified_solver_failure'))
    end
    methods (Test)
        function preservesFailureClass(testCase,Failure)
            [state,cfg,out] = testCase.fixture();
            out.completed_horizon = false;
            out.terminal_reason = Failure.input;
            [reason,done] = resolve_phase3_terminal(state,out,0,cfg);
            testCase.verifyEqual(reason,Failure.expected);
            testCase.verifyTrue(done);
        end
        function recoveredSolverIsNotTerminal(testCase)
            [state,cfg,out] = testCase.fixture();
            out.terminal_reason = 'numerical_solver_failure_recovered';
            [reason,done] = resolve_phase3_terminal(state,out,0,cfg);
            testCase.verifyEqual(reason,'');
            testCase.verifyFalse(done);
        end
        function invalidStateTakesPriorityOverMission(testCase)
            [state,cfg,out] = testCase.fixture();
            state.Xt(1) = NaN;
            [reason,done] = resolve_phase3_terminal(state,out,320,cfg);
            testCase.verifyEqual(reason,'invalid_state');
            testCase.verifyTrue(done);
        end
        function missionCompletion(testCase)
            [state,cfg,out] = testCase.fixture();
            [reason,done] = resolve_phase3_terminal(state,out,320,cfg);
            testCase.verifyEqual(reason,'mission_complete');
            testCase.verifyTrue(done);
        end
        function invalidBatteryIsNotSilentlyAccepted(testCase)
            [state,cfg,out] = testCase.fixture();
            state.battery.soc_pct = NaN;
            [reason,done] = resolve_phase3_terminal(state,out,0,cfg);
            testCase.verifyEqual(reason,'invalid_state');
            testCase.verifyTrue(done);
        end
        function batteryTerminal(testCase)
            [state,cfg,out] = testCase.fixture();
            state.battery.margin_norm = 0.1;
            [reason,done] = resolve_phase3_terminal(state,out,0,cfg);
            testCase.verifyEqual(reason,'battery_terminal');
            testCase.verifyTrue(done);
        end
        function timeLimit(testCase)
            [state,cfg,out] = testCase.fixture();
            state.t = 600;
            [reason,done] = resolve_phase3_terminal(state,out,0,cfg);
            testCase.verifyEqual(reason,'time_limit');
            testCase.verifyTrue(done);
        end
    end
    methods (Static,Access = private)
        function [state,cfg,out] = fixture()
            state = struct('Xt',zeros(30,1),'Ut',zeros(12,1),'t',0.1, ...
                'battery',struct('margin_norm',0.95,'soc_pct',95));
            cfg = struct('MISSION',struct('D_TARGET_M',320),'MISSION_DURATION',600, ...
                'BATTERY',struct('terminal_margin',0.2));
            out = struct('completed_horizon',true,'terminal_reason','horizon_complete');
        end
    end
end
