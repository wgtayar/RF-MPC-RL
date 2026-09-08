function comparison = compare_exact_qp_rebuild(actual, archivedQP, archivedRow)
%compare_exact_qp_rebuild Require exact QP and post-FSM inputs, not a tolerance.
    names = {'H','g','Aineq','bineq','Aeq','beq','Xt','Ut','Xd','Ud','FSM','fsm_internal_state'};
    expected = archivedQP.problem;
    for name = {'Xt','Ut','Xd','Ud'}
        expected.(name{1}) = archivedQP.(name{1});
    end
    expected.FSM = archivedRow.FSM;
    expected.fsm_internal_state = archivedRow.fsm_after;
    matches = false(numel(names),1);
    for k = 1:numel(names)
        matches(k) = isfield(actual,names{k}) && isequaln(actual.(names{k}),expected.(names{k}));
    end
    comparison = struct('exact_match',all(matches),'fields',{names}, ...
        'matches',matches,'mismatched_fields',{names(~matches)}, ...
        'problem_override_used',false);
end
