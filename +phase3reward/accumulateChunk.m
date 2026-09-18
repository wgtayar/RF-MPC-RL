function a = accumulateChunk(a,row,snapshot,cfg)
%accumulateChunk Derive chunk accounting/stop events from captured MPC rows.
% This read-only validator never trusts a saved outcome or decision window.
    if isempty(a)
        p = snapshot.parameters;
        before = snapshot.state;
        requested = min(cfg.CHUNK_DURATION,cfg.MISSION_DURATION-before.t);
        target = Inf;
        if isfield(cfg.PHASE3.options,'mission_end_mode') && ...
                strcmp(cfg.PHASE3.options.mission_end_mode,'mpc_step_target_v1')
            target = before.decision_bookkeeping.initial_position_x+cfg.MISSION.D_TARGET_M;
            assert(before.Xt(1)<target,'phase3window:Target','A chunk must start before its configured target.');
        end
        a = struct('before',before,'parameters',p,'control',snapshot.control, ...
            'segment',row.segment,'context',row.context,'expected_steps',round(requested/p.simTimeStep), ...
            'requested_duration',requested,'target_x',target,'attempts',0,'integrated_steps',0, ...
            'failed_steps',0,'recovered_events',0,'tracking_sum',0,'effort_sum',0, ...
            'last_time',before.t,'last_state',struct(),'last_FSM',[],'last_stance',NaN, ...
            'reason',"horizon_complete",'sample_times',zeros(0,1),'sample_currents',zeros(0,1));
        assert(isequal(snapshot.context,row.context) && a.expected_steps>0 && ...
            abs(a.expected_steps*p.simTimeStep-requested)<1e-8 && ...
            isequaln(compact_exact_mpc_state(before),row.state_before), ...
            'phase3window:Start','Chunk snapshot/context/time grid must match its first row.');
    end
    assert(row.segment==a.segment && isequal(row.context,a.context) && ...
        row.iteration==a.attempts+1 && a.attempts<a.expected_steps && ...
        a.reason=="horizon_complete" && abs(row.time_before-a.last_time)<1e-8 && ...
        isequaln(row.action_execution,a.control.action_execution), ...
        'phase3window:Sequence','Rows must follow the chunk grid without continuing after a stop.');
    a.attempts = a.attempts+1;
    a.recovered_events = a.recovered_events+double(strcmp(row.solver.classification,'numerical_solver_failure_recovered'));
    a.last_state = row.state_after;
    a.last_time = row.time_after;
    a.last_FSM = row.FSM;
    a.last_stance = min(a.parameters.Tst,0.2/norm(row.Xt_qp(4:5)));
    if ~row.solver.success
        assert(~row.integrated,'phase3window:Integration','Failed QP cannot have integrated.');
        a.failed_steps = a.failed_steps+1;
        a.reason = string(row.solver.classification);
        assert(a.reason~="horizon_complete" && strlength(a.reason)>0, ...
            'phase3window:Solver','A failed solver needs its captured failure classification.');
    elseif ~row.integrated
        assert(isfield(row.solver,'integration_exception'), ...
            'phase3window:Integration','Successful unintegrated attempt needs an explicit integration exception.');
        a.reason = "integration_exception";
    elseif any(~isfinite(row.state_after.Xt))
        a.reason = "invalid_state";
    else
        a.integrated_steps = a.integrated_steps+1;
        a.tracking_sum = a.tracking_sum+sum((row.state_after.Xt-row.Xd(:,1)).^2);
        a.effort_sum = a.effort_sum+sum(row.state_after.Ut.^2);
        if row.state_after.Xt(1)>=a.target_x
            a.reason = "mission_target_reached";
        end
    end
    if row.current_sample_committed
        a.sample_times(end+1,1) = row.current_sample_time;
        a.sample_currents(end+1,1) = row.current_sample_A;
    end
end
