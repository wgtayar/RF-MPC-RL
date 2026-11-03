function result = phase3_target_crossing(steps, target, current, batteryCfg)
%phase3_target_crossing Bracket target crossing without extrapolating energy.
% For a fixed-pack ledger, detected means a crossing on a committed energy
% interval with no actuator-limit violation up to and including that interval.
% An invalid requested trajectory is retained only as geometric_crossing.
% Legacy proxy traces retain their historical geometric-only semantics.
    result = struct('detected',false,'time_linear_s',NaN,'time_bracket_s',[NaN,NaN], ...
        'charge_As',NaN,'soc_pct',NaN,'sampled_energy_available',false, ...
        'time_method','linear_position_interpolation_inside_recorded_MPC_step', ...
        'energy_method','linear_current_interpolation_then_existing_battery_model');
    fixedPack = isfield(current,'ledger');
    if fixedPack
        required = {'voltage_limit_steps','torque_limit_steps','energy_committed'};
        assert(all(isfield(steps,required)), 'phase3_target_crossing:Feasibility', ...
            'Fixed-pack crossings require actuator counts and energy-commit evidence.');
        n = numel(steps.integrated);
        for field = required(1:2)
            value = steps.(field{1});
            assert(isnumeric(value) && isreal(value) && numel(value)==n && ...
                all(isfinite(value(:))) && all(value(:)>=0) && all(value(:)==fix(value(:))), ...
                'phase3_target_crossing:Feasibility','Actuator counts must be finite nonnegative integers.');
        end
        assert(islogical(steps.energy_committed) && numel(steps.energy_committed)==n, ...
            'phase3_target_crossing:Feasibility','Require one logical energy-commit flag per row.');
        assert(isnumeric(current.ledger) && isreal(current.ledger) && ...
            isequal(size(current.ledger),[n,6]), ...
            'phase3_target_crossing:Ledger','Require charge, energy and SOC endpoint pairs per row.');
        result.schema = 'fixed_pack_target_crossing_v2';
        result.energy_Wh = NaN;
        result.qualification = 'no_geometric_crossing';
        result.geometric_crossing = struct('detected',false,'time_linear_s',NaN, ...
            'time_bracket_s',[NaN,NaN]);
    end
    crossed = steps.integrated & isfinite(steps.distance_before) & isfinite(steps.distance_after) & ...
        steps.distance_before < target & steps.distance_after >= target;
    k = find(crossed,1);
    if isempty(k)
        return
    end
    fraction = (target-steps.distance_before(k))/ ...
        (steps.distance_after(k)-steps.distance_before(k));
    time = steps.time_before(k)+fraction*(steps.time_after(k)-steps.time_before(k));
    if fixedPack
        result.geometric_crossing = struct('detected',true,'time_linear_s',time, ...
            'time_bracket_s',[steps.time_before(k),steps.time_after(k)]);
        % Counts are cumulative, so a later crossing cannot erase an earlier
        % actuator failure in a diagnostic rollout with enforcement disabled.
        if any(steps.voltage_limit_steps(1:k)>0) || any(steps.torque_limit_steps(1:k)>0)
            result.qualification = 'actuator_limit';
            return
        end
        if ~all(steps.energy_committed(1:k))
            result.qualification = 'energy_not_committed';
            return
        end
        assert(all(isfinite(current.ledger(k,:))), 'phase3_target_crossing:Ledger', ...
            'A qualified crossing requires finite charge, energy and SOC endpoints.');
        result.qualification = 'actuator_feasible';
    end
    result.detected = true;
    result.time_linear_s = time;
    result.time_bracket_s = [steps.time_before(k),steps.time_after(k)];
    if fixedPack
        endpoints=reshape(current.ledger(k,:),2,3);
        value=[1-fraction,fraction]*endpoints;
        result.charge_As=value(1); result.energy_Wh=value(2); result.soc_pct=value(3);
        result.energy_method='interpolation_of_recorded_pack_interval_totals_and_estimates';
        result.sampled_energy_available=true; return
    end
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
