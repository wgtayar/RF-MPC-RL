function bundle=energy_mission_evaluation_bundle(original,overrides)
%ENERGY_MISSION_EVALUATION_BUNDLE Change only declared scenario/dependent fields.
assert(isstruct(overrides) && isscalar(overrides) && all(ismember(fieldnames(overrides), ...
 {'initial_soc','target_distance_m','mission_duration_s','seed'})), ...
 'energyPolicy:ScenarioOverrides','Unsupported scenario override.');
bundle=original;
% Evaluation always uses a declared fixed start, even when the checkpoint was
% trained with per-episode initial-SOC sampling.
if isfield(bundle.cfg,'INITIAL_SOC'),bundle.cfg.INITIAL_SOC.mode='fixed';end
if isempty(fieldnames(overrides)),return;end
c=original.cfg;o=c.ENERGY_RL.options;
for n=fieldnames(overrides).',o.(n{1})=overrides.(n{1});end
if isfield(c,'INITIAL_SOC')
 socRange=c.INITIAL_SOC.range;
else
 socRange=[.95 1];
end
validateattributes(o.initial_soc,{'double'},{'scalar','real','finite'});
assert(o.initial_soc>=socRange(1)-1e-12 && o.initial_soc<=socRange(2)+1e-12,'energyMission:InitialSOC', ...
 'Evaluation start SOC must lie in the configuration''s declared initial-SOC range.');
validateattributes([o.target_distance_m,o.mission_duration_s],{'double'},{'size',[1,2],'real','finite','positive'});
validateattributes(o.seed,{'double'},{'scalar','integer','nonnegative','finite','<=',2^32-1});
p=get_params(0);assert(abs(o.mission_duration_s/p.simTimeStep-round(o.mission_duration_s/p.simTimeStep))<1e-8, ...
 'energyMission:DurationGrid','Mission must remain on the MPC time grid.');
c.BATTERY.SOC_init=o.initial_soc;c.BATTERY.metric_init=100*o.initial_soc;
c.MISSION.D_TARGET_M=o.target_distance_m;c.MISSION_DURATION=o.mission_duration_s;c.RNG_SEED=o.seed;
c.EP_STEPS=ceil(c.MISSION_DURATION/(c.CHUNK_DURATION*c.APPLY_EVERY));
c.MISSION.WINDOW_TARGET_M=c.MISSION.D_TARGET_M/c.EP_STEPS;c.V_REQ_FIXED=c.MISSION.D_TARGET_M/c.MISSION_DURATION;
maxV=c.V_MIN+c.GAMMA_V_MAX*(c.V_MAX-c.V_MIN);
assert(c.V_REQ_FIXED>=c.V_MIN && c.V_REQ_FIXED<=maxV,'energyMission:MissionPace','Requested average pace exceeds archived bounds.');
c.GAMMA_V_MISSION=(c.V_REQ_FIXED-c.V_MIN)/(c.V_MAX-c.V_MIN);
c.START_HIGH.gamma_v_init=c.GAMMA_V_MISSION;c.START_HIGH.gamma_v_floor=c.GAMMA_V_MISSION;
c.ENERGY_RL.options=o;
if isfield(c,'INITIAL_SOC'),c.INITIAL_SOC.mode='fixed';end
% The reference is the archived unit, never a fresh file from current setup.
c.ENERGY_RL.reference_charge_As=c.ENERGY_RL.reference.charge_As/c.ENERGY_RL.reference.distance_m*c.MISSION.D_TARGET_M;
energy_mission_scales(c);bundle.cfg=c;
end
