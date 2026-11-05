function model = swing_model_of(cfg)
%SWING_MODEL_OF Swing-foot model declared by a configuration.
% 'held_target_v0'   : historical servo dpf = Kp_sw*(pfd - pf) with pfd held
%                      for the whole MPC interval (sawtooth speed peaks).
% 'tracked_bezier_v1': swing feet track the continuous Bezier plan with
%                      velocity feed-forward (see fcns/dynamics_SRB.m).
% Configurations without cfg.SWING keep the historical model so archived
% runs reproduce unchanged.
model = 'held_target_v0';
if isstruct(cfg) && isfield(cfg,'SWING') && isfield(cfg.SWING,'model')
    model = char(cfg.SWING.model);
end
assert(ismember(model,{'held_target_v0','tracked_bezier_v1'}), ...
    'swing_model_of:Unknown','Unknown swing model "%s".',model);
end
