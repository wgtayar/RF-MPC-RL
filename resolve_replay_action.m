function [control, execution] = resolve_replay_action(candidate, state, cfg, lowerR, upperR)
%resolve_replay_action Use the environment action path from an exact replay state.

    arguments
        candidate (5,1) double {mustBeReal, mustBeFinite}
        state (1,1) struct
        cfg (1,1) struct
        lowerR (3,1) double {mustBePositive, mustBeFinite}
        upperR (3,1) double {mustBePositive, mustBeFinite}
    end
    if ~isfield(state, 'supervisory_state')
        error('resolve_replay_action:MissingActionHistory', ...
            'An exact supervisory_state is required; do not infer prior gammas from a candidate.');
    end
    previous = state.supervisory_state;
    previous.last_R = state.R(:);
    execution = resolve_supervisory_action(candidate, previous, cfg, lowerR, upperR);
    control = struct('R', execution.R_applied, 'v_cmd', execution.v_exec, ...
        'a_cmd', execution.a_exec, 'action', execution.applied_action, ...
        'action_execution', execution);
end
