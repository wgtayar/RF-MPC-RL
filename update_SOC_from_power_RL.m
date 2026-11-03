function [socFraction,meanPackA,time,power,packA] = update_SOC_from_power_RL(trace)
% Compatibility reader for a canonical energy trace (SOC in percent).
% Old centroidal-only inputs cannot recover holding or swing demand.
assert(istable(trace) && all(ismember({'Time','PowerW','PackCurrent','SOC'}, ...
 trace.Properties.VariableNames)) && height(trace)>0,'robotenergy:Trace','Supply the energy provider table.');
socFraction=trace.SOC(end)/100; time=trace.Time; power=trace.PowerW; packA=trace.PackCurrent;
meanPackA=mean(packA); % uniformly sampled MPC intervals
end
