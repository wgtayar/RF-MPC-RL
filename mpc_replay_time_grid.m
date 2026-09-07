function [stepTimes, nextTimes] = mpc_replay_time_grid(stateTime, dt, numberSteps, control)
%mpc_replay_time_grid Preserve archived floating-point timestep arithmetic.
% With no archived origin, retain the legacy per-horizon time convention.
    validateattributes(stateTime,{'double'},{'scalar','finite','nonnegative'});
    validateattributes(dt,{'double'},{'scalar','finite','positive'});
    validateattributes(numberSteps,{'double'},{'scalar','integer','nonnegative'});
    origin = stateTime;
    offset = 0;
    if isfield(control,'integration_time_origin_s')
        origin = control.integration_time_origin_s;
        validateattributes(origin,{'double'},{'scalar','finite','nonnegative'});
        offset = round((stateTime-origin)/dt);
        assert(offset >= 0 && abs(origin+dt*offset-stateTime) <= 8*eps(max(stateTime,1)), ...
            'mpc_replay_time_grid:OffGrid', ...
            'Restored time is not on the archived integration grid.');
    end
    stepTimes = origin + dt*(offset+(0:numberSteps-1));
    nextTimes = origin + dt*(offset+(1:numberSteps));
end
