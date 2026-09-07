function validate_exact_state_row(row, previous)
%validate_exact_state_row Reject incomplete or discontinuous new MPC records.
    required = {'time_before','time_after','iteration','state_before','state_after', ...
        'Xt_qp','Ut_qp','Xd','Ud','FSM','fsm_after','solver','health_before', ...
        'health_after','dynamic_event','current_sample_A','current_sample_time', ...
        'current_sample_committed','action_execution','configuration_sha256', ...
        'source_sha','row_id','segment','context','snapshot_reference','integrated'};
    if ~all(isfield(row,required))
        error('validate_exact_state_row:MissingField','Exact-state row lacks required fields.');
    end
    stateFields = {'Xt','Ut','t','fsm_internal_state','knee_proxy_state', ...
        'battery','current_sample_count','mpc_warm_start'};
    if ~all(isfield(row.state_before,stateFields)) || ~all(isfield(row.state_after,stateFields))
        error('validate_exact_state_row:IncompleteState','Hidden state is incomplete.');
    end
    validateattributes(row.Xt_qp,{'double'},{'numel',30});
    validateattributes(row.Ut_qp,{'double'},{'numel',12});
    validateattributes(row.FSM,{'double'},{'numel',4});
    validateattributes([row.time_before,row.time_after],{'double'},{'finite','numel',2});
    validateattributes([row.row_id,row.segment,row.iteration, ...
        row.context.episode,row.context.decision,row.context.chunk], ...
        {'numeric'},{'integer','positive','numel',6});
    if row.state_before.t ~= row.time_before || row.state_after.t ~= row.time_after
        error('validate_exact_state_row:StateTime','State timestamps differ from row timestamps.');
    end
    actionFields = {'candidate_action','applied_action','previous_applied_action', ...
        'gamma_v_raw','gamma_a_raw','gamma_v_applied','gamma_a_applied', ...
        'R_applied','v_req','a_req','v_exec','a_exec', ...
        'delta_gamma_v_applied','delta_v_exec','previous_action_complete'};
    if ~all(isfield(row.action_execution,actionFields))
        error('validate_exact_state_row:IncompleteAction','Raw/applied action data are incomplete.');
    end
    solverFields = {'strategy','success','exitflag','output','lambda','solver_options', ...
        'initial_point','iterations','wall_time_s','diagnostics'};
    if ~all(isfield(row.solver,solverFields))
        error('validate_exact_state_row:IncompleteSolver','Solver evidence is incomplete.');
    end
    if row.time_after < row.time_before || ...
            (row.integrated && row.time_after <= row.time_before)
        error('validate_exact_state_row:TimeOrder','Invalid integration time ordering.');
    end
    if ~row.action_execution.previous_action_complete
        error('validate_exact_state_row:MissingActionHistory','Prior applied action is incomplete.');
    end
    if nargin >= 2 && ~isempty(previous)
        if row.row_id ~= previous.row_id+1
            error('validate_exact_state_row:RowGap','MPC row IDs are not contiguous.');
        end
        sameEpisode = row.context.episode == previous.context.episode;
        if sameEpisode && (abs(row.time_before-previous.time_after) > 1e-9 || ...
                ~isequaln(row.state_before.Xt,previous.state_after.Xt) || ...
                ~isequaln(row.state_before.Ut,previous.state_after.Ut))
            error('validate_exact_state_row:StateGap','MPC time or dynamic state is discontinuous.');
        end
        if sameEpisode && (~isequaln(row.state_before.fsm_internal_state, ...
                previous.state_after.fsm_internal_state) || ...
                ~isequaln(row.state_before.knee_proxy_state,previous.state_after.knee_proxy_state))
            error('validate_exact_state_row:HiddenStateGap','FSM or proxy state is discontinuous.');
        end
        if row.segment == previous.segment && row.iteration ~= previous.iteration+1
            error('validate_exact_state_row:IterationGap','Iteration is not contiguous within segment.');
        end
        if sameEpisode && (row.context.decision < previous.context.decision || ...
                (row.context.decision == previous.context.decision && ...
                row.context.chunk < previous.context.chunk))
            error('validate_exact_state_row:ContextOrder','Decision/chunk index moves backward.');
        end
    end
end
