function plant = plantMismatch(b)
%PLANTMISMATCH Validated evaluation-only plant/estimator mismatch (protocol A4).
% b.plant_mismatch declares a simulated battery that differs from the frozen
% nominal pack while the estimator keeps the nominal model:
%   resistance_scale    multiplies the plant cell R0, R1, R2, R3 (the RC
%                       capacitances are unchanged, so the RC time constants
%                       scale with the resistances);
%   capacity_Ah         plant cell capacity that books the true SOC;
%   soc_estimate_offset initial estimator SOC minus true SOC at a fresh start
%                       (robotenergy.start); a carried battery keeps its
%                       estimator state.
%   estimator_resistance_scale (optional; A4 control) multiplies the
%                       ESTIMATOR's R0..R3 (robotenergy.estimatorView): a
%                       battery management system that has re-identified the
%                       pack resistance. Absent, the estimator stays nominal.
% The frozen specification (C_nom_Ah, n_series, n_parallel, cell table) and
% every estimator, observation and reward quantity stay nominal. Only
% robotenergy.advancePlant and robotenergy.instantaneousPack read the plant.
m = b.plant_mismatch;
names = {'version','resistance_scale','capacity_Ah','soc_estimate_offset','scope'};
optional = {'estimator_resistance_scale'};
assert(isstruct(m) && isscalar(m) && all(ismember(names,fieldnames(m))) && ...
    all(ismember(fieldnames(m),[names optional])), ...
    'robotenergy:PlantMismatch','Plant mismatch must declare exactly %s.',strjoin(names,', '));
assert(strcmp(m.version,'plant_battery_mismatch_v1'),'robotenergy:PlantMismatch', ...
    'Unknown plant mismatch version.');
validateattributes(m.resistance_scale,{'double'},{'scalar','real','finite','positive'});
validateattributes(m.capacity_Ah,{'double'},{'scalar','real','finite','positive'});
validateattributes(m.soc_estimate_offset,{'double'},{'scalar','real','finite'});
plant = struct('version',m.version,'resistance_scale',m.resistance_scale, ...
    'capacity_Ah',m.capacity_Ah,'soc_estimate_offset',m.soc_estimate_offset);
if isfield(m,'estimator_resistance_scale')
    validateattributes(m.estimator_resistance_scale,{'double'},{'scalar','real','finite','positive'});
    plant.estimator_resistance_scale = m.estimator_resistance_scale;
end
end
