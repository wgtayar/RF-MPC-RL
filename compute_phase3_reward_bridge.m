function [reward, info] = compute_phase3_reward_bridge(window, cfg)
%compute_phase3_reward_bridge Legacy formula for environment comparison only.
% This explicit adapter is NOT calibrated reward v2 and is not training-ready.
    legacyWindow = window;
    catastrophic = {'numerical_solver_failure_unrecovered', ...
        'mathematical_constraint_infeasible','invalid_state', ...
        'dynamic_safety_violation','no_safe_action_available','unclassified_solver_failure'};
    if ismember(window.terminal_reason,catastrophic)
        legacyWindow.terminal_reason = 'infeasible';
    end
    [reward,info] = compute_rl_reward(legacyWindow,cfg);
    info.version = 'reward_phase3_legacy_bridge_v1';
    info.training_promoted = false;
    info.actual_terminal_reason = window.terminal_reason;
    info.legacy_formula_terminal_category = legacyWindow.terminal_reason;
    info.recovered_solver_events = window.recovered_solver_events;
end
