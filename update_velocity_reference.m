function reference = update_velocity_reference(state, control)
%UPDATE_VELOCITY_REFERENCE Retarget an acceleration-limited desired trajectory.
% The retained segment is an exact desired-motion history, not a fit to the
% measured robot state. Repeated commands retain it unchanged, making the
% result independent of simulation chunk boundaries. Acceleration is a
% positive magnitude used for both speeding up and slowing down.
    validateattributes(state.t, {'numeric'}, {'scalar','real','finite','nonnegative'});
    validateattributes(control.v_cmd, {'numeric'}, {'scalar','real','finite','nonnegative'});
    validateattributes(control.a_cmd, {'numeric'}, {'scalar','real','finite','positive'});
    if isfield(control,'reference_mode')
        assert(strcmp(control.reference_mode,'velocity_continuous_v2'), ...
            'update_velocity_reference:Mode','Expected velocity_continuous_v2.');
    end

    if isfield(state,'reference_state') && ~isempty(state.reference_state)
        previous = state.reference_state;
        % Evaluation also validates the stored schema and segment domain.
        [position, velocity] = evaluate_velocity_reference(previous,state.t);
        if isequal(previous.v_cmd,control.v_cmd) && isequal(previous.a_cmd,control.a_cmd)
            reference = previous;
            return
        end
    elseif state.t == 0
        % Preserve the existing mission convention: desired x=v=0 at t=0.
        position = 0;
        velocity = 0;
    else
        error('update_velocity_reference:MissingHistory', ...
            'A nonzero-time continuation requires its saved v2 desired-reference state.');
    end

    reference = struct('schema_version','reference_velocity_state_v2', ...
        'mode','velocity_continuous_v2','t_start_s',state.t, ...
        'x_start_m',position,'v_start_mps',velocity, ...
        'v_cmd',control.v_cmd,'a_cmd',control.a_cmd);
end
