function [reference, offset] = update_reference_position_offset(state, control)
%update_reference_position_offset Preserve desired position at command changes.
% This opt-in adapter changes position only. Velocity commands and the core
% reference/FSM/QP mathematics remain unchanged for a controlled ablation.
    mode = 'legacy_absolute_time';
    if isfield(control,'reference_mode')
        mode = char(control.reference_mode);
    end
    validateattributes(control.v_cmd,{'numeric'},{'scalar','finite','nonnegative'});
    validateattributes(control.a_cmd,{'numeric'},{'scalar','finite','positive'});
    offset = 0;
    switch mode
        case 'legacy_absolute_time'
            % No offset in the inherited reference convention.
        case 'position_continuous_v1'
            if state.t > 0
                if ~isfield(state,'reference_state')
                    error('update_reference_position_offset:MissingHistory', ...
                        'Supply the actual previous command and reference offset; do not anchor to measured position.');
                end
                previous = state.reference_state;
                if isequal([previous.v_cmd,previous.a_cmd],[control.v_cmd,control.a_cmd])
                    offset = previous.position_offset_m;
                else
                    offset = previous.position_offset_m + ...
                        localPosition(state.t,previous.v_cmd,previous.a_cmd) - ...
                        localPosition(state.t,control.v_cmd,control.a_cmd);
                end
            end
        otherwise
            error('update_reference_position_offset:UnknownMode','Unknown reference mode %s.',mode);
    end
    reference = struct('schema_version','reference_position_state_v1','mode',mode, ...
        'v_cmd',control.v_cmd,'a_cmd',control.a_cmd,'position_offset_m',offset);
end

function position = localPosition(time, velocity, acceleration)
    if time < velocity/acceleration
        position = 0.5*acceleration*time^2;
    else
        position = velocity*time-0.5*velocity^2/acceleration;
    end
end
