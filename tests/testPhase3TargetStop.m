classdef testPhase3TargetStop < matlab.unittest.TestCase
    methods (Test)
        function successfulTargetStopIsNotASolverFailure(testCase)
            [state,cfg,out] = localFixture();
            [reason,done] = resolve_phase3_terminal(state,out,320,cfg);
            testCase.verifyEqual(reason,'mission_complete');
            testCase.verifyTrue(done);
        end
        function invalidStateStillTakesPriority(testCase)
            [state,cfg,out] = localFixture();
            state.Xt(2) = NaN;
            testCase.verifyEqual(resolve_phase3_terminal(state,out,320,cfg),'invalid_state');
        end
        function rejectsTargetMarkerBeforeTarget(testCase)
            [state,cfg,out] = localFixture();
            testCase.verifyError(@() resolve_phase3_terminal(state,out,319.99,cfg), ...
                'resolve_phase3_terminal:TargetStop');
        end
        function rejectsTargetMarkerWithFailedSolve(testCase)
            [state,cfg,out] = localFixture();
            out.qp_failed_count = 1;
            testCase.verifyError(@() resolve_phase3_terminal(state,out,320,cfg), ...
                'resolve_phase3_terminal:TargetStop');
        end
        function rejectsTargetMarkerWithoutIntegration(testCase)
            [state,cfg,out] = localFixture();
            out.integrated_steps = 0;
            testCase.verifyError(@() resolve_phase3_terminal(state,out,320,cfg), ...
                'resolve_phase3_terminal:TargetStop');
        end
        function rejectsTargetMarkerWithoutOptIn(testCase)
            [state,cfg,out] = localFixture();
            cfg = rmfield(cfg,'PHASE3');
            testCase.verifyError(@() resolve_phase3_terminal(state,out,320,cfg), ...
                'resolve_phase3_terminal:TargetStop');
        end
        function missionKeepsExistingBatteryTiePrecedence(testCase)
            [state,cfg,out] = localFixture();
            state.battery.margin_norm = 0.1;
            testCase.verifyEqual(resolve_phase3_terminal(state,out,320,cfg),'mission_complete');
        end
        function solverFailureIsNotHiddenByPosition(testCase)
            [state,cfg,out] = localFixture();
            out.terminal_reason = 'numerical_solver_failure';
            out.qp_failed_count = 1;
            testCase.verifyEqual(resolve_phase3_terminal(state,out,320,cfg), ...
                'numerical_solver_failure_unrecovered');
        end
        function rejectsAlreadyReachedTargetBeforeSolving(testCase)
            state = struct('Xt',zeros(30,1));
            cfg = struct('CHUNK_DURATION',0.1);
            options = struct('stop_at_position_x_m',0);
            testCase.verifyError(@() simulate_mpc_horizon(state,struct(),cfg,options), ...
                'simulate_mpc_horizon:TargetAlreadyReached');
        end
    end
end

function [state,cfg,out] = localFixture()
    state = struct('Xt',zeros(30,1),'Ut',zeros(12,1),'t',1.03, ...
        'battery',struct('margin_norm',0.95,'soc_pct',95));
    cfg = struct('MISSION',struct('D_TARGET_M',320),'MISSION_DURATION',600, ...
        'BATTERY',struct('terminal_margin',0.2), ...
        'PHASE3',struct('options',struct('mission_end_mode','mpc_step_target_v1')));
    out = struct('completed_horizon',false,'terminal_reason','mission_target_reached', ...
        'qp_failed_count',0,'integrated_steps',3);
end
