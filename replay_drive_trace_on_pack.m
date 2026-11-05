function T = replay_drive_trace_on_pack(drive, seriesCounts, initialSocs, cellCapacityAh, minimumSoc)
%REPLAY_DRIVE_TRACE_ON_PACK Interval-level offline battery replay of a drive trace.
% drive: simulate_mpc_horizon drive_trace rows [t_before t_after watts
% required_V ...]. For each candidate series count (1P, the team cell table
% with the given capacity) and initial SOC, every MPC interval is solved at
% its duration-weighted mean power: the loaded bus at interval start is
% compared with the interval's sampled peak required voltage, then the plant
% advances at that constant power. This is a screening replay, not the
% native resolved check (which also samples within-interval power peaks);
% the selected pack must still be qualified in native closed-loop runs.
if nargin < 5, minimumSoc = .2; end
watts = drive(:,3); requiredV = drive(:,4); dt = drive(:,2) - drive(:,1);
assert(all(isfinite([watts;requiredV;dt])) && all(dt > 0) && all(watts >= 0), ...
    'packReplay:Trace','Drive trace must be finite with positive intervals.');
rows = cell(0,10);
for ns = seriesCounts(:).'
    for z = initialSocs(:).'
        b = phase3battery.manuscriptConfig(); b.n_series = ns; b.n_parallel = 1;
        b.C_nom_Ah = cellCapacityAh; b.SOC_init = z; b.metric_init = 100*z;
        b.use_pack_sizing = false; s = robotenergy.start(b);
        minimumHeadroom = inf; atTime = NaN; maximumPackA = 0; failure = "";
        completed = 0;
        for k = 1:numel(watts)
            try
                y = robotenergy.instantaneousPack(s, watts(k));
            catch exception
                failure = string(exception.identifier); break
            end
            headroom = y.packV - requiredV(k);
            if headroom < minimumHeadroom, minimumHeadroom = headroom; atTime = drive(k,1); end
            maximumPackA = max(maximumPackA, y.packA);
            if headroom < 0, failure = "voltage_headroom"; break, end
            s = robotenergy.advancePlant(s, watts(k), dt(k));
            if s.soc < minimumSoc, failure = "soc_floor"; break, end
            completed = k;
        end
        rows(end+1,:) = {ns, z, completed == numel(watts), s.soc, minimumHeadroom, ...
            atTime, maximumPackA, s.chargeAs, s.energyWh, failure}; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'n_series','initial_soc','complete', ...
    'final_soc','minimum_headroom_V','minimum_headroom_time_s','maximum_pack_A', ...
    'charge_As','energy_Wh','failure'});
end
