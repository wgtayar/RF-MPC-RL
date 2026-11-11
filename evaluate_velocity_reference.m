function [position, velocity, acceleration] = evaluate_velocity_reference(reference, time)
%EVALUATE_VELOCITY_REFERENCE Exact ramp followed by constant-velocity motion.
% Position and velocity are continuous at retargeting and ramp completion.
% Acceleration is piecewise constant; this is not a jerk-limited reference.
% TIME may be a scalar or array; outputs have its shape. Evaluate only at or
% after this segment's start. Old times require the corresponding old state.
    required = {'schema_version','mode','t_start_s','x_start_m', ...
        'v_start_mps','v_cmd','a_cmd'};
    assert(isstruct(reference) && isscalar(reference) && all(isfield(reference,required)), ...
        'evaluate_velocity_reference:InvalidState','Missing v2 reference-state fields.');
    assert(strcmp(reference.schema_version,'reference_velocity_state_v2') && ...
        strcmp(reference.mode,'velocity_continuous_v2'), ...
        'evaluate_velocity_reference:Schema','Reference history belongs to a different reference mode.');
    validateattributes(reference.t_start_s,{'numeric'},{'scalar','real','finite','nonnegative'});
    validateattributes(reference.x_start_m,{'numeric'},{'scalar','real','finite'});
    validateattributes(reference.v_start_mps,{'numeric'},{'scalar','real','finite','nonnegative'});
    validateattributes(reference.v_cmd,{'numeric'},{'scalar','real','finite','nonnegative'});
    validateattributes(reference.a_cmd,{'numeric'},{'scalar','real','finite','positive'});
    validateattributes(time,{'numeric'},{'real','finite','nonnegative','nonempty'});
    elapsed = time-reference.t_start_s;
    % A replay grid can differ from its stored boundary by floating-point
    % roundoff. Permit only that numerical tolerance, never a past segment.
    tolerance = 32*eps(max(1,max(abs([time(:);reference.t_start_s]))));
    assert(all(elapsed(:) >= -tolerance), ...
        'evaluate_velocity_reference:BeforeSegment','Cannot evaluate before the saved segment start.');
    elapsed = max(elapsed,0);
    delta = reference.v_cmd-reference.v_start_mps;
    signedAcceleration = sign(delta)*reference.a_cmd;
    rampDuration = abs(delta)/reference.a_cmd;
    rampElapsed = min(elapsed,rampDuration);
    cruiseElapsed = max(elapsed-rampDuration,0);
    position = reference.x_start_m + reference.v_start_mps*rampElapsed + ...
        0.5*signedAcceleration*rampElapsed.^2 + reference.v_cmd*cruiseElapsed;
    velocity = reference.v_start_mps + signedAcceleration*rampElapsed;
    complete = elapsed >= rampDuration-tolerance;
    velocity(complete) = reference.v_cmd;
    acceleration = signedAcceleration*double(~complete);
end
