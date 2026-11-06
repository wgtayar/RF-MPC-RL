function out = attitude_telemetry(action, acc, value)
%ATTITUDE_TELEMETRY Shared per-step body attitude accumulation (logged only).
% Used identically by simulate_mpc_horizon and the dataset reconstruction so
% captured windows validate exactly. Enabled only when cfg.STABILITY exists.
%   threshold = attitude_telemetry('threshold', cfg)   ([] when disabled)
%   acc = attitude_telemetry('start', threshold)
%   acc = attitude_telemetry('step', acc, Xt)
%   summary = attitude_telemetry('summary', acc)
switch action
    case 'threshold'
        cfg = acc; out = [];
        if isstruct(cfg) && isfield(cfg,'STABILITY') && isfield(cfg.STABILITY,'tilt_threshold_rad')
            out = cfg.STABILITY.tilt_threshold_rad;
            validateattributes(out,{'double'},{'scalar','real','finite','positive'});
        end
    case 'start'
        out = struct('threshold',acc,'steps',0,'max_tilt',0,'tilt_sq',0, ...
            'max_rate',0,'exceed',0,'excess_sq',0);
    case 'step'
        out = acc;
        rotation = reshape(value(7:15),3,3);
        tilt = acos(min(max(rotation(3,3),-1),1));
        out.steps = out.steps+1;
        out.max_tilt = max(out.max_tilt,tilt);
        out.tilt_sq = out.tilt_sq+tilt^2;
        out.max_rate = max(out.max_rate,norm(value(16:18)));
        out.exceed = out.exceed+(tilt>out.threshold);
        out.excess_sq = out.excess_sq+max(0,tilt-out.threshold)^2;
    case 'summary'
        out = struct('version','attitude_window_v1','integrated_steps',acc.steps, ...
            'max_tilt_rad',acc.max_tilt,'tilt_square_sum',acc.tilt_sq, ...
            'max_body_rate_rad_s',acc.max_rate,'tilt_exceedance_steps',acc.exceed, ...
            'tilt_excess_square_sum',acc.excess_sq,'tilt_threshold_rad',acc.threshold, ...
            'definition','Tilt is the angle between body z and world z after each integrated MPC step; rate is the body angular-velocity norm.');
    otherwise
        error('attitude_telemetry:Action','Unknown action.');
end
end
