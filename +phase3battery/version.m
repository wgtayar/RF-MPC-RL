function name = version(cfg)
%version Resolve an explicit battery feedback contract without changing cfg.
    name = 'battery_feedback_legacy_prefix_v1';
    if isfield(cfg,'feedback_version')
        value = cfg.feedback_version;
        assert((ischar(value) && isrow(value)) || ...
            (isstring(value) && isscalar(value) && ~ismissing(value)), ...
            'phase3battery:Version','Battery feedback version must be scalar text.');
        name = char(value);
    end
    assert(any(strcmp(name,{'battery_feedback_legacy_prefix_v1', ...
        'battery_feedback_timestamp_aligned_v2'})), ...
        'phase3battery:Version','Unknown battery feedback version.');
end
