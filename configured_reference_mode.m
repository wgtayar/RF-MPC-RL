function mode = configured_reference_mode(cfg)
%CONFIGURED_REFERENCE_MODE Saved configurations without a marker retain v1.
mode = 'position_continuous_v1';
if isfield(cfg,'REFERENCE')
    assert(isfield(cfg.REFERENCE,'mode'),'reference:Config','Missing declared reference mode.');
    mode = validatestring(cfg.REFERENCE.mode, ...
        {'legacy_absolute_time','position_continuous_v1','velocity_continuous_v2'});
end
end
