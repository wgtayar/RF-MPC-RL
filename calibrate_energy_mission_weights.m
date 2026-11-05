function report = calibrate_energy_mission_weights(comparisonResultFile)
%CALIBRATE_ENERGY_MISSION_WEIGHTS Balanced time/charge preference from matched runs.
% Reads a run_energy_mission_comparison result.json. For each initial-SOC
% group of completed fixed-speed policies, the per-episode preference cost is
%   J(w) = w*t/T + (1-w)*Q/Qref,
% the same normalized terms used by compute_energy_mission_reward. The
% indifference weight between the fastest and slowest completed policies is
%   w* = dQn/(dTn + dQn),  dTn = (t_slow-t_fast)/T,  dQn = (Q_fast-Q_slow)/Qref.
% Above w* the fastest policy wins, below it the slowest wins, and between
% them the winner depends on the curvature of the measured time-charge front.
% The balanced default is the mean w* over the initial-SOC groups.
r = jsondecode(fileread(comparisonResultFile));
cases = r.cases; if iscell(cases), cases = [cases{:}]; end
assert(~isempty(cases),'energyWeights:Empty','No comparison cases.');
socs = unique([cases.initial_soc]);
groups = struct([]); grid = linspace(0,1,1001);
for z = socs
    c = cases([cases.initial_soc]==z & [cases.mission_complete]);
    assert(numel(c)>=2,'energyWeights:Cases','Need two completed policies per SOC group.');
    b = setup_energy_mission_rl(localSetup(comparisonResultFile,z)); sc = energy_mission_scales(b.cfg);
    tn = [c.time_s]/sc.mission_duration_s; qn = [c.charge_As]/sc.reference_charge_As;
    [~,fast] = min(tn); [~,slow] = max(tn);
    dT = tn(slow)-tn(fast); dQ = qn(fast)-qn(slow);
    assert(dT>0,'energyWeights:Time','Policies must differ in completion time.');
    if dQ <= 0
        wStar = 1; % faster is also no more costly in charge: time always wins.
    else
        wStar = dQ/(dT+dQ);
    end
    winners = zeros(size(grid));
    for k = 1:numel(grid)
        [~,winners(k)] = min(grid(k)*tn + (1-grid(k))*qn);
    end
    speeds = [c.requested_speed_m_s];
    groups(end+1).initial_soc = z; %#ok<AGROW>
    groups(end).requested_speed_m_s = speeds;
    groups(end).time_s = [c.time_s]; groups(end).charge_As = [c.charge_As];
    groups(end).normalized_time = tn; groups(end).normalized_charge = qn;
    groups(end).indifference_time_weight = wStar;
    groups(end).winner_speed_by_weight = struct('time_weight',grid(:), ...
        'winner_speed_m_s',speeds(winners(:)).');
    groups(end).weight_band_with_interior_winner = localInteriorBand(grid,speeds(winners),speeds([fast slow]));
end
report = struct('schema','energy_mission_weight_calibration_v1', ...
    'comparison_source_sha',r.source_sha,'comparison_result_sha256',sha256_file(comparisonResultFile), ...
    'groups',groups,'balanced_time_weight',mean([groups.indifference_time_weight]), ...
    'scope',['Fixed-speed matched short missions only. The balanced weight makes the ' ...
    'fastest and slowest tested policies tie, so the time-charge tradeoff decides the winner.']);
end
function setup = localSetup(resultFile,z)
manifest = jsondecode(fileread(fullfile(fileparts(resultFile),'manifest.json')));
setup = manifest.options.setup_options;
setup.initial_soc = z;
end
function band = localInteriorBand(grid,winnerSpeeds,extremes)
interior = ~ismember(winnerSpeeds,extremes);
band = [];
if any(interior), band = [min(grid(interior)) max(grid(interior))]; end
end
