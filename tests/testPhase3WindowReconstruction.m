classdef testPhase3WindowReconstruction < matlab.unittest.TestCase
    properties (TestParameter)
        terminalCase = struct('numerical','numerical_solver_failure_unrecovered', ...
            'mathematical','mathematical_constraint_infeasible', ...
            'unknown','unclassified_solver_failure','invalid','invalid_state', ...
            'target','mission_complete','mission_chunk','mission_complete', ...
            'timeout','time_limit','battery','battery_terminal');
    end
    methods (Test)
        function preservesTerminalIdentity(testCase,terminalCase)
            [window,terminal,~,audit] = localScenario(terminalCase);
            testCase.verifyEqual(terminal,terminalCase);
            testCase.verifyEqual(window.terminal_reason,terminalCase);
            testCase.verifyTrue(audit.is_done);
            testCase.verifyFalse(audit.saved_window_or_outcome_used);
            testCase.verifyFalse(audit.battery_model_recomputed);
        end
        function reconstructsPhysicalWindowMetrics(testCase)
            [w,terminal,~] = localScenario('none');
            testCase.verifyEqual(terminal,'');
            testCase.verifyEqual(w.window_distance_m,0.003,AbsTol=1e-14);
            testCase.verifyEqual(w.distance_end_m,0.003,AbsTol=1e-14);
            testCase.verifyEqual(w.duration_s,0.03,AbsTol=1e-14);
            testCase.verifyEqual(w.charge_As,0.2,AbsTol=1e-14);
            testCase.verifyEqual(w.tracking_error_mean,0.030014,AbsTol=1e-14);
            testCase.verifyEqual(w.control_effort_mean,36,AbsTol=1e-14);
            testCase.verifyEqual(w.qp_solve_count,3);
            testCase.verifyTrue(w.feasible);
        end
        function targetStopsOnFirstCrossing(testCase)
            [w,terminal] = localScenario('target_stop');
            testCase.verifyEqual(terminal,'mission_complete');
            testCase.verifyEqual(w.duration_s,0.02,AbsTol=1e-14);
            testCase.verifyEqual(w.qp_solve_count,2);
            testCase.verifyEqual(w.window_distance_m,0.002,AbsTol=1e-14);
            testCase.verifyTrue(w.feasible);
        end
        function invalidStateKeepsExplicitFeatureFallback(testCase)
            [w,terminal] = localScenario('invalid_state');
            testCase.verifyEqual(terminal,'invalid_state');
            testCase.verifyTrue(w.invalid_state_feature_fallback);
            testCase.verifyEqual(w.window_distance_m,0);
            testCase.verifyEqual(w.duration_s,0.03,AbsTol=1e-14);
            testCase.verifyFalse(w.feasible);
        end
        function recoveredEventIsNotTerminal(testCase)
            [w,terminal] = localScenario('recovered');
            testCase.verifyEqual(terminal,'');
            testCase.verifyEqual(w.recovered_solver_events,1);
            testCase.verifyTrue(w.feasible);
        end
        function auditTriggerIsNotCalibratedSafetyTerminal(testCase)
            [~,terminal] = localScenario('audit_trigger');
            testCase.verifyEqual(terminal,'');
        end
        function integrationExceptionIsNotNormalTransition(testCase)
            testCase.verifyError(@() localScenario('exception'),'phase3window:Exception');
        end
        function truncatedChunkRejected(testCase)
            [~,~,~,~,a,after] = localScenario('none');
            a.attempts = 2;
            testCase.verifyError(@() phase3reward.finishChunk(a,after),'phase3window:Truncated');
        end
        function unrecordedCurrentRejected(testCase)
            [~,~,~,~,a,after] = localScenario('none');
            after.current_total(end) = 11;
            testCase.verifyError(@() phase3reward.finishChunk(a,after),'phase3window:Endpoint');
        end
        function shortNonterminalDecisionRejected(testCase)
            [~,~,chunk,~,~,~,cfg] = localScenario('none');
            cfg.APPLY_EVERY = 2;
            testCase.verifyError(@() phase3reward.reconstructWindow({chunk},cfg),'phase3window:ShortDecision');
        end
        function chunkAfterTerminalRejected(testCase)
            [~,~,chunk,~,~,~,cfg] = localScenario('mission_complete');
            cfg.APPLY_EVERY = 2;
            testCase.verifyError(@() phase3reward.reconstructWindow({chunk,chunk},cfg),'phase3window:AfterTerminal');
        end
        function inconsistentStartingProgressRejected(testCase)
            [~,~,chunk,~,~,~,cfg] = localScenario('none');
            chunk.before.decision_bookkeeping.distance_m = 2;
            testCase.verifyError(@() phase3reward.reconstructWindow({chunk},cfg),'phase3window:Progress');
        end
        function rowAfterTargetStopRejected(testCase)
            [~,~,~,~,a,~,cfg] = localScenario('target_stop');
            row = struct('segment',a.segment,'context',a.context,'iteration',a.attempts+1);
            testCase.verifyError(@() phase3reward.accumulateChunk(a,row,[],cfg),'phase3window:Sequence');
        end
    end
end

function [window,terminal,chunk,audit,a,after,cfg] = localScenario(kind)
    source = bootstrap_RF_MPC_RL();
    bundle = load(fullfile(source,'rlEnv_MPC_R.mat'),'cfg');
    cfg = bundle.cfg;
    cfg.CHUNK_DURATION = 0.03;
    cfg.APPLY_EVERY = 1;
    cfg.MISSION_DURATION = 1;
    cfg.MISSION.D_TARGET_M = 1;
    cfg.PHASE3.options = struct();
    n = 3;
    start = 0;
    if strcmp(kind,'time_limit')
        start = 0.97;
    elseif strcmp(kind,'target_stop')
        cfg.PHASE3.options.mission_end_mode = 'mpc_step_target_v1';
        cfg.MISSION.D_TARGET_M = 0.0015;
        n = 2;
    elseif strcmp(kind,'mission_complete')
        cfg.MISSION.D_TARGET_M = 0.0015;
    end
    h = struct('last_R',ones(3,1),'v_req',cfg.V_REQ_FIXED,'a_req',cfg.A_REQ_FIXED, ...
        'v_exec',0.5,'prev_gamma_v',cfg.GAMMA_V_MISSION,'prev_gamma_a',cfg.GAMMA_A_MIN, ...
        'previous_applied_action',[zeros(3,1);cfg.GAMMA_V_MISSION;cfg.GAMMA_A_MIN]);
    action = resolve_supervisory_action([zeros(3,1);cfg.GAMMA_V_MISSION;cfg.GAMMA_A_MIN], ...
        h,cfg,0.95*ones(3,1),1.05*ones(3,1));
    control = struct('action_execution',action,'R',action.R_applied,'v_cmd',action.v_exec,'a_cmd',action.a_exec);
    before = struct('Xt',zeros(30,1),'Ut',ones(12,1),'t',start, ...
        'current_time',[],'current_total',[],'battery',struct('soc_pct',80,'margin_norm',0.8), ...
        'supervisory_state',h,'decision_bookkeeping',struct('initial_position_x',0,'distance_m',0, ...
        'prev_Ieq_window',0,'a_exec',action.a_exec));
    before.Xt(4) = 0.1;
    context = struct('episode',1,'decision',1,'chunk',1);
    snapshot = struct('state',before,'parameters',struct('simTimeStep',0.01,'Tst',0.2), ...
        'control',control,'context',context);
    a = [];
    state = before;
    for k = 1:n
        row = struct('segment',1,'context',context,'iteration',k,'time_before',state.t, ...
            'state_before',compact_exact_mpc_state(state),'Xt_qp',state.Xt,'Xd',zeros(30,1), ...
            'FSM',ones(4,1),'action_execution',action,'integrated',true, ...
            'solver',struct('success',true,'classification','success'), ...
            'current_sample_committed',true,'current_sample_time',state.t,'current_sample_A',10, ...
            'dynamic_event',strcmp(kind,'audit_trigger'));
        if strcmp(kind,'recovered') && k==1
            row.solver.classification = 'numerical_solver_failure_recovered';
        end
        if k==n && any(strcmp(kind,{'numerical_solver_failure_unrecovered','mathematical_constraint_infeasible', ...
                'unclassified_solver_failure','exception'}))
            row.integrated = false;
            row.current_sample_committed = false;
            if strcmp(kind,'exception')
                row.solver.integration_exception = struct('identifier','synthetic');
            else
                row.solver.success = false;
                row.solver.classification = kind;
            end
        else
            state.t = start+k*0.01;
            state.Xt(1) = k*0.001;
            if strcmp(kind,'invalid_state') && k==n
                state.Xt(1) = NaN;
                row.current_sample_committed = false;
            end
        end
        if row.current_sample_committed
            state.current_time(end+1,1) = row.current_sample_time;
            state.current_total(end+1,1) = 10;
        end
        row.state_after = compact_exact_mpc_state(state);
        row.time_after = state.t;
        a = phase3reward.accumulateChunk(a,row,snapshot,cfg);
    end
    after = state;
    if strcmp(kind,'battery_terminal')
        after.battery.margin_norm = 0;
    end
    chunk = phase3reward.finishChunk(a,after);
    [window,terminal,audit] = phase3reward.reconstructWindow({chunk},cfg);
end
