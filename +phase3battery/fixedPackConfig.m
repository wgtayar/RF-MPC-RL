function cfg = fixedPackConfig()
%FIXEDPACKCONFIG Frozen simulated pack selected from resolved mission demand.
% One source-controlled specification is shared by every new RL episode.
% Historical manuscriptConfig remains the original unlocked 4S6P profile.
root=fileparts(mfilename('fullpath'));
spec=jsondecode(fileread(fullfile(root,'fixed_pack_specification.json')));
cfg=phase3battery.manuscriptConfig();
assert(strcmp(sha256_file(cfg.model_options.parameter_source),spec.cell_model_sha256), ...
    'robotenergy:PackCellModel','Frozen pack cell table changed; qualify a new specification explicitly.');
cfg.n_series=spec.n_series; cfg.n_parallel=spec.n_parallel;
cfg.C_nom_Ah=spec.cell_capacity_Ah; cfg.pack_voltage=spec.reported_nominal_voltage_V;
cfg.use_pack_sizing=false; cfg.fixed_pack_spec=spec;
cfg.energy_provider='fixed_pack_joint_v1';
cfg.streaming_estimator_version='fixed_pack_resolved_ekf_v3';
cfg.assumptions.pack_policy=['Frozen simulated electrical design ' spec.id ...
    '; ideal identical cells and ideal drive; full-charge bus capability ' ...
    num2str(spec.required_full_charge_bus_capability_V,12) ' V; physical hardware compatibility unvalidated.'];
end
