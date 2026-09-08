function [reason, isDone] = resolve_phase3_terminal(state, out, distance, cfg)
%resolve_phase3_terminal Keep physical and numerical terminal causes distinct.
    reason = '';
    if any(~isfinite([state.Xt(:);state.Ut(:);state.battery.margin_norm;state.battery.soc_pct;distance]))
        reason = 'invalid_state';
    elseif ~out.completed_horizon
        switch char(out.terminal_reason)
            case {'numerical_solver_failure','numerical_solver_failure_unrecovered'}
                reason = 'numerical_solver_failure_unrecovered';
            case {'mathematical_constraint_infeasible','mathematical_infeasibility'}
                reason = 'mathematical_constraint_infeasible';
            case {'invalid_state','dynamic_safety_violation','no_safe_action_available'}
                reason = char(out.terminal_reason);
            otherwise
                reason = 'unclassified_solver_failure';
        end
    elseif distance >= cfg.MISSION.D_TARGET_M
        reason = 'mission_complete';
    elseif state.battery.margin_norm <= cfg.BATTERY.terminal_margin
        reason = 'battery_terminal';
    elseif state.t >= cfg.MISSION_DURATION-1e-9
        reason = 'time_limit';
    end
    isDone = ~isempty(reason);
end
