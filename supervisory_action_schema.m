function schema = supervisory_action_schema(cfg)
%SUPERVISORY_ACTION_SCHEMA Preserve historical action mapping on old configs.
schema = 'action_v2_rate_limited';
if isfield(cfg,'ACCELERATION')
    assert(strcmp(cfg.ACCELERATION.action_model,'bounded_transition_v1'), ...
        'governor:AccelerationModel','Unknown acceleration action model.');
    schema = 'action_v3_transition_acceleration';
end
end
