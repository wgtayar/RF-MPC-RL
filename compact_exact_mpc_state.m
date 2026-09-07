function compact = compact_exact_mpc_state(state)
%compact_exact_mpc_state Keep dynamic state; reference growing history by count.
    compact = state;
    compact.current_sample_count = numel(state.current_time);
    drop = intersect(fieldnames(compact), ...
        {'current_time','current_total','knee_template','knee_parameters'});
    compact = rmfield(compact,drop);
end
