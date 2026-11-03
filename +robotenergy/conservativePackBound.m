function bound=conservativePackBound(b,peakVoltage,peakPower,energyWh,minimumStartSoc)
%CONSERVATIVEPACKBOUND Sufficient current/voltage/reserve bounds, not an optimum.
% Icap is the measured characterization amplitude, not a hardware current rating.
s=robotenergy.start(b); floorSoc=b.terminal_margin;
assert(minimumStartSoc>floorSoc && minimumStartSoc<=1, ...
    'packBound:Reserve','Require positive usable SOC reserve.');
validateattributes([peakVoltage peakPower energyWh],{'numeric'},{'real','finite','nonnegative'});
Icap=b.assumptions.characterization_discharge_C*b.C_nom_Ah;
assert(Icap>0 && isfinite(Icap),'packBound:Current','Require a positive characterized current amplitude.');
[ocvMin,ocvMax]=ocvExtrema(s,floorSoc,1);
points=unique([floorSoc;1;s.grid(s.grid>=floorSoc & s.grid<=1)]);
values=interp1(s.grid,s.values,points,'linear');
assert(all(values>0,'all'),'packBound:Parameters','RC parameters must remain strictly positive.');
maxR=max(values(:,1:4),[],1); Rtotal=sum(maxR);
Vlow=ocvMin-Icap*Rtotal;
Emin=ocvMin-Icap*sum(maxR(2:4));
rootMargin=Emin-2*Icap*Rtotal;
assert(Vlow>0 && rootMargin>0,'packBound:Root', ...
    'These sufficient bounds do not establish a positive bus and increasing discharge-power branch.');
ns=max(1,ceil(peakVoltage/Vlow));
parallelPower=peakPower/(ns*Vlow*Icap);
parallelReserve=energyWh/(ns*Vlow*b.C_nom_Ah*(minimumStartSoc-floorSoc));
np=max([1,ceil(parallelPower),ceil(parallelReserve)]);
bound=struct('method','sufficient_rc_envelope_first_series_then_parallel_v1', ...
    'soc_domain',[floorSoc,1],'minimum_start_soc',minimumStartSoc, ...
    'characterization_current_A',Icap,'ocv_min_V',ocvMin,'ocv_max_V',ocvMax, ...
    'maximum_resistances_ohm',maxR,'summed_resistance_bound_ohm',Rtotal, ...
    'cell_voltage_lower_bound_V',Vlow,'high_root_monotonicity_margin_V',rootMargin, ...
    'sampled_peak_voltage_V',peakVoltage,'sampled_peak_power_W',peakPower, ...
    'mission_energy_Wh',energyWh,'n_series',ns,'n_parallel',np,'cell_count',ns*np, ...
    'parallel_power_lower_count',parallelPower,'parallel_reserve_lower_count',parallelReserve, ...
    'guaranteed_loaded_bus_lower_bound_V',ns*Vlow, ...
    'required_full_charge_bus_capability_V',ns*ocvMax, ...
    'nominal_capacity_Ah',np*b.C_nom_Ah, ...
    'reported_nominal_pack_voltage_V',ns*b.assumptions.cell_nominal_voltage_reported_V, ...
    'selection_scope','Conservative sufficient design for supplied sampled demand, not globally minimum cell count; current criterion is data support, not a device rating.');
end
function [low,high]=ocvExtrema(s,lower,upper)
assert(lower>=s.grid(1) && upper<=s.grid(end),'packBound:Coverage','OCV domain must be characterized.');
[breaks,coefs,~,order]=unmkpp(s.ocv);
points=[lower;upper;s.grid(s.grid>=lower & s.grid<=upper)];
for k=1:size(coefs,1)
    critical=roots(coefs(k,1:end-1).*(order-1:-1:1));
    critical=real(critical(abs(imag(critical))<eps));
    critical=critical(critical>=0 & critical<=breaks(k+1)-breaks(k))+breaks(k);
    points=[points;critical(critical>=lower & critical<=upper)]; %#ok<AGROW>
end
volts=ppval(s.ocv,points); low=min(volts); high=max(volts);
end
