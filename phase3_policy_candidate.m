function candidate = phase3_policy_candidate(policy, observation, time, cfg)
%phase3_policy_candidate Evaluate a frozen feedforward actor or baseline.
    if strcmp(policy.kind,'baseline')
        candidate = baseline_supervisory_action(policy.id,time,cfg);
    else
        [result,nextState] = evaluate(policy.actor,{observation});
        assert(isempty(nextState),'phase3_policy_candidate:StatefulActor', ...
            'A recurrent actor requires explicit captured policy-state handling.');
        candidate = double(result{1}(:));
    end
    validateattributes(candidate,{'double'},{'numel',5,'finite','real'});
end
