function [observation,audit] = build_energy_mission_observation(state,legacyObservation,lastRow,cfg,window)
%BUILD_ENERGY_MISSION_OBSERVATION Replace old battery/current normalization.
% Estimated states and an observable Coulomb counter are exposed; true SOC
% and true RC states are deliberately not policy inputs. A forecast is an
% explicitly labeled historical-rate extrapolation, not a viability claim.
    if nargin<5, window=struct(); end
    schema = energy_mission_observation_schema(cfg);
    units = schema.electrical_units;
    [~,base] = build_phase3_observation_v2(state,legacyObservation,lastRow,cfg,schema.base_schema);
    assert(isfield(state,'energy') && all(isfield(state.energy,{'estimate','chargeAs'})), ...
        'energyMission:ElectricalState','Require explicit estimated battery state and booked charge.');
    estimate = state.energy.estimate(:);
    validateattributes(estimate,{'double'},{'numel',4,'real','finite'});
    validateattributes(state.energy.chargeAs,{'double'}, ...
        {'scalar','real','finite','nonnegative'});
    if state.t==0 && isempty(fieldnames(window))
        current = 0;
    else
        assert(isfield(window,'Ieq_window'),'energyMission:CurrentHistory', ...
            'Provide the actual current decision mean, not stale bookkeeping or clipped legacy current.');
        current = window.Ieq_window;
        validateattributes(current,{'double'},{'scalar','real','finite','nonnegative'});
    end
    raw = base.raw;
    raw(base.schema.names=="battery_margin") = estimate(1);
    raw(base.schema.names=="previous_Ieq") = current/units.one_C_pack_current_A;
    charge = state.energy.chargeAs;
    distance = state.Xt(1)-state.decision_bookkeeping.initial_position_x;
    hasForecast = isreal(distance) && isfinite(distance) && distance>0 && state.t>0;
    forecast = NaN; surplus = NaN;
    if hasForecast
        remaining = max(units.target_distance_m-distance,0);
        forecast = charge/distance*remaining;
        reserve = units.pack_capacity_As*(estimate(1)-cfg.BATTERY.terminal_margin);
        surplus = reserve-forecast;
        hasForecast = isfinite(forecast) && isfinite(surplus);
        if ~hasForecast, forecast=NaN; surplus=NaN; end
    end
    voltage = optionalFinite(state.battery,'pack_voltage_V');
    headroom = optionalFinite(state.battery,'drive_voltage_headroom_V');
    electrical = [charge/units.reference_charge_As;forecast/units.reference_charge_As; ...
        surplus/units.reference_charge_As;double(hasForecast); ...
        voltage/units.pack_voltage_unit_V;headroom/units.pack_voltage_unit_V; ...
        estimate(2:4);double(isfinite(voltage));double(isfinite(headroom))];
    raw = [raw;electrical];
    assert(numel(raw)==schema.dimension,'energyMission:Dimension','Observation schema mismatch.');
    available = isfinite(raw);
    encoded = raw; encoded(~available)=0;
    observation = asinh(encoded./schema.scales);
    observation(schema.identity_transform) = encoded(schema.identity_transform);
    audit = struct('schema',schema,'raw',raw,'available',available, ...
        'clipped',false(size(raw)),'base_audit',base, ...
        'current_mean_pack_A',current,'forecast_available',hasForecast, ...
        'projected_remaining_charge_As',forecast,'projected_reserve_surplus_As',surplus, ...
        'true_soc_exposed',false, ...
        'normalization_scope','Current uses actual decision Coulombs per second divided by pack 1C; no clipping or safety-rating claim.');
end

function value = optionalFinite(s,name)
    value = NaN;
    if isfield(s,name) && isnumeric(s.(name)) && isscalar(s.(name)) && ...
            isreal(s.(name)) && isfinite(s.(name))
        value = s.(name);
    end
end
