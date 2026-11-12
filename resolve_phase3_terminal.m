function [reason, isDone] = resolve_phase3_terminal(state, out, distance, cfg)
%resolve_phase3_terminal Keep physical and numerical terminal causes distinct.
    reason = '';
    values = [state.Xt(:);state.Ut(:);state.battery.margin_norm;state.battery.soc_pct;distance];
    if ~isreal(values) || any(~isfinite(values))
        reason = 'invalid_state';
    elseif isfield(state.battery,'voltage_limit_steps') && ...
            (state.battery.voltage_limit_steps>0 || state.battery.torque_limit_steps>0)
        reason='actuator_limit';
    elseif strcmp(out.terminal_reason,'mission_target_reached')
        assert(isfield(cfg,'PHASE3') && isfield(cfg.PHASE3,'options') && ...
            isfield(cfg.PHASE3.options,'mission_end_mode') && ...
            strcmp(cfg.PHASE3.options.mission_end_mode,'mpc_step_target_v1') && ...
            distance>=cfg.MISSION.D_TARGET_M && out.qp_failed_count==0 && out.integrated_steps>0, ...
            'resolve_phase3_terminal:TargetStop','Target-stop evidence and configured mission must agree.');
        reason = 'mission_complete';
    elseif ~out.completed_horizon
        switch char(out.terminal_reason)
            case {'numerical_solver_failure','numerical_solver_failure_unrecovered'}
                reason = 'numerical_solver_failure_unrecovered';
            case {'mathematical_constraint_infeasible','mathematical_infeasibility'}
                reason = 'mathematical_constraint_infeasible';
            case {'invalid_state','dynamic_safety_violation','no_safe_action_available', ...
                    'battery_power_limit','leg_reach_limit','battery_depleted'}
                reason = char(out.terminal_reason);
            otherwise
                reason = 'unclassified_solver_failure';
        end
    elseif localBackward(state,distance,cfg)
        reason = 'backward_motion';
    elseif distance >= cfg.MISSION.D_TARGET_M
        reason = 'mission_complete';
    elseif state.battery.margin_norm <= cfg.BATTERY.terminal_margin
        reason = 'battery_terminal';
    elseif state.t >= cfg.MISSION_DURATION-1e-9
        reason = 'time_limit';
    end
    isDone = ~isempty(reason);
end

function backward = localBackward(state,~,cfg)
    % Reward v3 (investigator 2026-11-12): net backward motion over a
    % decision is a failure. Speed commands are >= V_MIN > 0, so this only
    % detects a loss of forward locomotion, never a commanded reversal.
    % Uses the signed displacement from the episode origin (the distance
    % argument is clipped at zero for reward progress and would hide
    % motion behind the origin; 2026-11-12 review). The decision's start
    % progress is clipped at zero, which is exact because an episode starts
    % at the origin and any earlier backward decision already terminated it.
    backward = false;
    if ~(isfield(cfg,'REWARD') && isfield(cfg.REWARD,'version') && ...
            strcmp(cfg.REWARD.version,'reward_energy_mission_v3'))
        return
    end
    assert(isfield(state,'decision_bookkeeping') && ...
        all(isfield(state.decision_bookkeeping,{'distance_m','initial_position_x'})), ...
        'resolve_phase3_terminal:Bookkeeping', ...
        'Reward v3 backward detection requires decision start progress and episode origin.');
    signedDisplacement = state.Xt(1)-state.decision_bookkeeping.initial_position_x;
    backward = signedDisplacement < state.decision_bookkeeping.distance_m - 1e-9;
end
