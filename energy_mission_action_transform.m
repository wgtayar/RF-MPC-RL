function [physical,audit] = energy_mission_action_transform(normalized,cfg)
%energy_mission_action_transform Dimensionless agent output to physical control.
% Noise is applied before this transform, so every action has the same
% fraction-of-range exploration. Existing controller clipping/rate limiting
% remains in resolve_replay_action and is recorded by the native environment.
    validateattributes(normalized,{'double','single'}, ...
        {'real','finite','numel',5},mfilename,'normalized');
    lower = [-cfg.DR_MAX*ones(3,1);cfg.GAMMA_V_MIN;cfg.GAMMA_A_MIN];
    upper = [ cfg.DR_MAX*ones(3,1);cfg.GAMMA_V_MAX;cfg.GAMMA_A_MAX];
    assert(all(isfinite([lower;upper])) && all(lower < upper), ...
        'energy_mission_action_transform:Bounds','Physical bounds must be finite and ordered.');
    requested = double(normalized(:));
    bounded = min(max(requested,-1),1);
    scale = (upper-lower)/2;
    bias = (upper+lower)/2;
    physical = bias+scale.*bounded;
    audit = struct('schema','energy_mission_normalized_action_v1', ...
        'requested_normalized',requested,'bounded_normalized',bounded, ...
        'physical_candidate',physical,'physical_lower',lower,'physical_upper',upper, ...
        'scale',scale,'bias',bias,'normalization_clipped',any(requested ~= bounded), ...
        'downstream_transform','resolve_replay_action');
end
