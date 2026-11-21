function [speed,audit] = energy_mission_b4_headroom_speed(previous,headroom,s,params)
%ENERGY_MISSION_B4_HEADROOM_SPEED Decision rule of the B4 headroom-feedback governor.
% [speed,audit] = energy_mission_b4_headroom_speed(previous,headroom,s,params)
% returns the speed command (m/s) for one 5 s supervisory decision of the B4
% baseline (BASELINES_V3, row B4). The rule is pure: it holds no state.
%
% Inputs
%   previous  speed command of the previous decision (m/s); NaN at the first
%             decision of a mission.
%   headroom  drive-voltage headroom per series cell (V), the raw value of
%             the observation entry drive_headroom_per_series_cell; NaN when
%             no headroom has been measured yet (has_drive_headroom = 0).
%   s         estimated SOC (fraction), the observation entry estimated_soc.
%   params    struct with h_low_V, h_high_V, v_star_m_s, v_cap_table ([] or an
%             N-by-2 matrix [estimated SOC, cap speed m/s]), v_min_m_s and
%             v_max_m_s (action bounds), step_m_s (0.1) and max_change_m_s
%             (the per-decision speed-command change cap, 0.2).
%
% Rule
%   cap = v_cap(s): linear interpolation of v_cap_table in s, clamped at the
%         table ends; v_max_m_s when the table is empty. The cap is limited
%         to [v_min_m_s, v_max_m_s].
%   First decision: command min(v_star, cap).
%   Otherwise:  headroom < h_low                    -> previous - step
%               headroom > h_high and previous < v* -> min(previous + step, v*)
%               no headroom measured or otherwise   -> previous
%   The result is bounded to [v_min, cap] and, after the first decision, to
%   previous +/- max_change. The change cap takes precedence over the speed
%   cap, as in the environment governor: if v_cap(s) falls by more than
%   max_change in one decision, the command reaches it over several
%   decisions. A lower request takes precedence over a raise request.
    k = localParams(params);
    validateattributes(previous,{'double'},{'scalar','real'},mfilename,'previous');
    validateattributes(headroom,{'double'},{'scalar','real'},mfilename,'headroom');
    validateattributes(s,{'double'},{'scalar','real','finite'},mfilename,'s');
    tol = 1e-9;
    cap = localCap(k,s);
    first = isnan(previous);
    if first
        event = 'start';
        target = k.v_star_m_s;
    elseif ~isfinite(headroom)
        event = 'hold_no_headroom';
        target = previous;
    elseif headroom < k.h_low_V
        event = 'lower';
        target = previous-k.step_m_s;
    elseif headroom > k.h_high_V && previous < k.v_star_m_s-tol
        event = 'raise';
        target = min(previous+k.step_m_s,k.v_star_m_s);
    else
        event = 'hold';
        target = previous;
    end
    bounded = min(max(target,k.v_min_m_s),cap);
    capLimited = target > cap+tol;
    speed = bounded;
    rateLimited = false;
    if ~first
        speed = min(max(bounded,previous-k.max_change_m_s),previous+k.max_change_m_s);
        rateLimited = abs(speed-bounded) > tol;
    end
    speed = round(speed,9);
    audit = struct('schema','b4_headroom_governor_v1','previous_command_m_s',previous, ...
        'headroom_per_cell_V',headroom,'soc_estimated',s,'event',event, ...
        'target_m_s',target,'cap_m_s',cap,'cap_limited',capLimited, ...
        'rate_limited',rateLimited,'command_m_s',speed, ...
        'reduced',~first && speed < previous-tol,'increased',~first && speed > previous+tol);
end

function cap = localCap(k,s)
if isempty(k.v_cap_table)
    cap = k.v_max_m_s;
else
    t = k.v_cap_table;
    if size(t,1) == 1
        cap = t(1,2);
    else
        cap = interp1(t(:,1),t(:,2),min(max(s,t(1,1)),t(end,1)),'linear');
    end
end
cap = min(max(cap,k.v_min_m_s),k.v_max_m_s);
end

function k = localParams(k)
names = {'h_low_V','h_high_V','v_star_m_s','v_cap_table','v_min_m_s','v_max_m_s','step_m_s','max_change_m_s'};
assert(isstruct(k) && isscalar(k) && all(isfield(k,names)),'b4Governor:Params', ...
    'B4 parameters need %s.',strjoin(names,', '));
validateattributes([k.h_low_V k.h_high_V],{'double'},{'real','finite'},mfilename,'thresholds');
assert(k.h_low_V <= k.h_high_V,'b4Governor:Thresholds','Require h_low_V <= h_high_V.');
validateattributes([k.v_min_m_s k.v_max_m_s],{'double'},{'real','finite','positive','increasing'},mfilename,'speed bounds');
validateattributes(k.v_star_m_s,{'double'},{'scalar','real','>=',k.v_min_m_s,'<=',k.v_max_m_s},mfilename,'v_star_m_s');
validateattributes(k.step_m_s,{'double'},{'scalar','real','finite','positive'},mfilename,'step_m_s');
validateattributes(k.max_change_m_s,{'double'},{'scalar','real','finite','>=',k.step_m_s},mfilename,'max_change_m_s');
k.v_cap_table = localCapTable(k.v_cap_table);
end

function t = localCapTable(t)
if isempty(t), t = zeros(0,2); return, end
validateattributes(t,{'double'},{'2d','ncols',2,'real','finite'},mfilename,'v_cap_table');
assert(all(diff(t(:,1)) > 0) && all(t(:,1) >= 0 & t(:,1) <= 1) && all(t(:,2) > 0), ...
    'b4Governor:CapTable','v_cap_table rows are [estimated SOC in 0..1 increasing, positive cap speed].');
end
