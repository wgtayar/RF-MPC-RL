function cfg = fixedPackConfig(specificationFile)
%FIXEDPACKCONFIG Frozen simulated pack from a source-controlled specification.
% One specification is shared by every episode of a profile. With no
% argument the historical 20S1P/2 Ah diagnostic pack is returned so archived
% configurations reproduce; the energy mission profile names its own file.
% Historical manuscriptConfig remains the original unlocked 4S6P profile.
if nargin<1 || isempty(specificationFile)
    specificationFile='fixed_pack_specification.json';
end
root=fileparts(mfilename('fullpath'));
spec=jsondecode(fileread(fullfile(root,specificationFile)));
cfg=phase3battery.manuscriptConfig();
assert(strcmp(sha256_file(cfg.model_options.parameter_source),spec.cell_model_sha256), ...
    'robotenergy:PackCellModel','Frozen pack cell table changed; qualify a new specification explicitly.');
cfg.n_series=spec.n_series; cfg.n_parallel=spec.n_parallel;
cfg.C_nom_Ah=spec.cell_capacity_Ah; cfg.pack_voltage=spec.reported_nominal_voltage_V;
cfg.use_pack_sizing=false; cfg.fixed_pack_spec=spec;
cfg.energy_provider='fixed_pack_joint_v1';
cfg.streaming_estimator_version='fixed_pack_resolved_ekf_v3';
if isfield(spec,'cell_identity')
    cfg.assumptions.cell_identity_status=spec.cell_identity;
    cfg.assumptions.cell_capacity_source=spec.cell_capacity_source;
    cfg.assumptions.cell_table_policy=spec.cell_table_policy;
end
cfg.assumptions.pack_policy=['Frozen simulated electrical design ' spec.id ...
    '; ideal identical cells and ideal drive; full-charge bus capability ' ...
    num2str(spec.required_full_charge_bus_capability_V,12) ' V; physical hardware compatibility unvalidated.'];
end
