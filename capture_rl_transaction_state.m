function snapshot = capture_rl_transaction_state(simulationState, decisionState, controllerState)
%capture_rl_transaction_state Capture a complete pre-decision state clone.
% This helper is a Phase-2 prototype and is not active in rlStepFunction.

    requiredSimulation = {'Xt','Ut','t','fsm_internal_state', ...
        'battery','knee_proxy_state','current_time','current_total', ...
        'R','previous_action','decision_bookkeeping','mpc_warm_start'};
    missing = requiredSimulation(~isfield(simulationState, requiredSimulation));
    if ~isempty(missing)
        error('capture_rl_transaction_state:IncompleteSimulationState', ...
            'Simulation state is missing: %s.', strjoin(missing, ', '));
    end
    if nargin < 2 || ~isstruct(decisionState)
        error('capture_rl_transaction_state:DecisionStateRequired', ...
            'decisionState must be a struct.');
    end
    if nargin < 3 || ~isstruct(controllerState)
        error('capture_rl_transaction_state:ControllerStateRequired', ...
            'controllerState must be a struct.');
    end

    snapshot = struct();
    snapshot.schema_version = 'rl_transaction_state_v1';
    snapshot.captured_at = string(datetime('now', 'TimeZone', 'local'));
    snapshot.payload = struct('simulation', simulationState, ...
        'decision', decisionState, 'controller', controllerState);
end
