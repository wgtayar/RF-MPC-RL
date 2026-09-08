function result = phase3_target_crossing(steps, target, current, batteryCfg)
%phase3_target_crossing Bracket target crossing without extrapolating energy.
    result = struct('detected',false,'time_linear_s',NaN,'time_bracket_s',[NaN,NaN], ...
        'charge_As',NaN,'soc_pct',NaN,'sampled_energy_available',false, ...
        'time_method','linear_position_interpolation_inside_recorded_MPC_step', ...
        'energy_method','linear_current_interpolation_then_existing_battery_model');
    crossed = steps.integrated & isfinite(steps.distance_before) & isfinite(steps.distance_after) & ...
        steps.distance_before < target & steps.distance_after >= target;
    k = find(crossed,1);
    if isempty(k)
        return
    end
    fraction = (target-steps.distance_before(k))/ ...
        (steps.distance_after(k)-steps.distance_before(k));
    time = steps.time_before(k)+fraction*(steps.time_after(k)-steps.time_before(k));
    result.detected = true;
    result.time_linear_s = time;
    result.time_bracket_s = [steps.time_before(k),steps.time_after(k)];
    if numel(current.time) < 2 || time < current.time(1) || time > current.time(end)
        return
    end
    assert(all(diff(current.time) > 0) && all(isfinite(current.total)), ...
        'phase3_target_crossing:CurrentTrace','Current samples must be finite and strictly ordered.');
    before = current.time < time;
    times = [current.time(before);time];
    currents = [abs(current.total(before));interp1(current.time,abs(current.total),time,'linear')];
    result.charge_As = trapz(times,currents);
    battery = evaluate_battery_feedback(times,currents,batteryCfg,struct());
    result.soc_pct = battery.soc_pct;
    result.sampled_energy_available = true;
end
