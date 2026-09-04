function [simulationState, decisionState, controllerState] = restore_rl_transaction_state(snapshot)
%restore_rl_transaction_state Restore a validated Phase-2 transaction clone.
% This helper is a Phase-2 prototype and is not active in rlStepFunction.

    if ~isstruct(snapshot) || ~isfield(snapshot, 'schema_version') || ...
            string(snapshot.schema_version) ~= "rl_transaction_state_v1" || ...
            ~isfield(snapshot, 'payload')
        error('restore_rl_transaction_state:InvalidSnapshot', ...
            'Snapshot is not an rl_transaction_state_v1 payload.');
    end
    required = {'simulation','decision','controller'};
    if ~all(isfield(snapshot.payload, required))
        error('restore_rl_transaction_state:IncompletePayload', ...
            'Transaction payload is incomplete.');
    end
    simulationState = snapshot.payload.simulation;
    decisionState = snapshot.payload.decision;
    controllerState = snapshot.payload.controller;
end
