function bundle = setup_energy_mission_rl(options)
%SETUP_ENERGY_MISSION_RL Explicit mission/charge objective on the fixed plant.
% Locked plant: tracked swing, 9S1P Nancy 2 Ah pack, declared electronics and
% driver losses. No load scaling, cell resizing, or SOC reset within a mission.
if nargin<1, options=struct(); end
assert(isstruct(options) && isscalar(options),'energyMission:Options','Options must be a scalar struct.');
defaults=struct('initial_soc',.95,'target_distance_m',320,'mission_duration_s',600, ...
    'decision_interval_s',5,'time_weight',.34,'seed',20260903);
% Provisional time weight 0.34 (charge 0.66) is inherited from
% calibrate_energy_mission_weights on the 2026-11-05 matched comparisons:
% indifference between the slowest and fastest tested fixed speeds is
% w*=0.337 (95% start) and 0.334 (100% start); an interior speed wins for
% w in about [0.26, 0.51]. That older plant differs from the current 9S
% configuration: compare current-plant time/charge before selecting a weight.
% Five-second decisions reuse the existing chunk/comparison cadence. The
% per-action speed/R caps are unchanged, so actions can accumulate ten times
% more often than at the old 50-second default. This is a provisional
% supervisory design, not a physical acceleration or stability guarantee.
assert(all(ismember(fieldnames(options),fieldnames(defaults))), ...
    'energyMission:Options','Unknown mission option.');
for f=fieldnames(defaults).'
    if ~isfield(options,f{1}),options.(f{1})=defaults.(f{1});end
end
assert(ismember(options.initial_soc,[.95,1]),'energyMission:InitialSOC', ...
    'Operational profiles start at 95 or 100 percent; diagnostic low-SOC tests are separate.');
validateattributes(options.target_distance_m,{'double'},{'scalar','real','finite','positive'});
validateattributes(options.mission_duration_s,{'double'},{'scalar','real','finite','positive'});
validateattributes(options.decision_interval_s,{'double'},{'scalar','real','finite','positive'});
validateattributes(options.time_weight,{'double'},{'scalar','real','finite','>=',0,'<=',1});
validateattributes(options.seed,{'double'},{'scalar','integer','nonnegative','<=',2^32-1});
oldRng=rng; cleanup=onCleanup(@()rng(oldRng));
bundle=setup_RL_MPC('legacy_frozen_v1'); c=bundle.cfg;
% Plant revision 2026-11-05: swing feet track the continuous Bezier plan
% (no held-target speed peaks) and the pack is the locked 9S1P 18650 pack with Nancy 2 Ah cell parameters.
c.SWING=struct('model','tracked_bezier_v1', ...
 'description','Swing feet follow the per-step Bezier plan with velocity feed-forward and Kp_sw correction inside each MPC interval.');
minimumSoc=c.BATTERY.terminal_margin;
c.BATTERY=phase3battery.fixedPackConfig(energy_mission_pack_file());
c.BATTERY.terminal_margin=minimumSoc; c.ENERGY.minimum_soc=minimumSoc;
c.EXPERIMENT.fixed_pack_id=c.BATTERY.fixed_pack_spec.id;
% Declared non-drive losses (investigator decision 2026-11-06), drawn from the
% pack bus: constant electronics load and motor-driver (inverter) efficiency.
% Legs stay massless as in the RF-MPC single-rigid-body model; swing-leg
% inertial energy enters only through the knee swing-torque template.
c.ENERGY.auxW=15; c.ENERGY.inverterEfficiency=.90;
c.ENERGY.loss_declaration=struct('version','declared_bus_losses_v1', ...
 'electronics_W',15,'electronics_basis', ...
 'Declared budget: embedded computer ~8-10 W, 12 motor-driver boards logic/idle ~5 W, IMU/encoders/comms ~1 W; replace with measured hardware draw.', ...
 'inverter_efficiency',.90,'inverter_basis', ...
 'Declared central value for small FOC BLDC drivers at partial load (typically 0.85-0.95), applied to positive per-motor electrical power.', ...
 'regeneration','none','leg_mass','massless links (RF-MPC single rigid body)');
% Speed envelope 2026-11-06 (investigator): command cap 1.2 m/s. The
% 2026-11-06 development holds are QP-feasible to 1.2 m/s (0 failures); 1.0 and
% 1.1 m/s hold 60 s with tilt <= 0.093 rad, while 1.2 m/s diverges after ~11 s.
% The former 0.66 m/s cap (GAMMA_V_MAX 0.45 of 0.3-1.1) is retired here. The
% governor keeps its per-decision increment, 0.024 m/s (not acceleration).
c.V_MAX=1.2; c.GAMMA_V_MAX=1; c.DGAMMA_V_MAX=.024/(c.V_MAX-c.V_MIN);
% Attitude telemetry (simulate_mpc_horizon out.attitude, window.attitude):
% logged for the RL design phase, NOT an observation input. The exceedance
% threshold sits above every stable steady hold (<= 0.093 rad, transient
% 0.19 rad) and below the diverging 1.2 m/s run (0.25-0.33 rad).
c.STABILITY=struct('tilt_threshold_rad',.25, ...
 'scope','Telemetry exceedance threshold only; not an observation feature and no tilt termination is configured.');
c.BATTERY.SOC_init=options.initial_soc; c.BATTERY.metric_init=100*options.initial_soc;
c.MISSION.D_TARGET_M=options.target_distance_m; c.MISSION_DURATION=options.mission_duration_s;
% Keep established 5-second chunks, allowing a shorter explicitly requested
% decision interval only when it is an integer number of MPC timesteps.
c.CHUNK_DURATION=min(5,options.decision_interval_s);
c.APPLY_EVERY=options.decision_interval_s/c.CHUNK_DURATION;
assert(abs(c.APPLY_EVERY-round(c.APPLY_EVERY))<1e-10, ...
    'energyMission:DecisionGrid','Decision interval must be <=5 s or a multiple of 5 s.');
c.APPLY_EVERY=round(c.APPLY_EVERY);
p=get_params(0);
assert(abs(c.CHUNK_DURATION/p.simTimeStep-round(c.CHUNK_DURATION/p.simTimeStep))<1e-8 && ...
 abs(c.MISSION_DURATION/p.simTimeStep-round(c.MISSION_DURATION/p.simTimeStep))<1e-8, ...
 'energyMission:DurationGrid','Mission and chunks must lie on the MPC time grid.');
c.EP_STEPS=ceil(c.MISSION_DURATION/options.decision_interval_s);
c.MISSION.WINDOW_TARGET_M=c.MISSION.D_TARGET_M/c.EP_STEPS;
c.V_REQ_FIXED=c.MISSION.D_TARGET_M/c.MISSION_DURATION;
maxV=c.V_MIN+c.GAMMA_V_MAX*(c.V_MAX-c.V_MIN);
assert(c.V_REQ_FIXED>=c.V_MIN && c.V_REQ_FIXED<=maxV, ...
 'energyMission:MissionPace','Requested average mission pace is outside the configured governor range.');
c.GAMMA_V_MISSION=(c.V_REQ_FIXED-c.V_MIN)/(c.V_MAX-c.V_MIN);
c.START_HIGH.gamma_v_init=c.GAMMA_V_MISSION; c.START_HIGH.gamma_v_floor=c.GAMMA_V_MISSION;
c.RNG_SEED=options.seed;
reference=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
    'energy_mission_reference.json')));
assert(strcmp(reference.pack_id,c.BATTERY.fixed_pack_spec.id), ...
    'energyMission:ReferencePack','Reference normalization belongs to another pack.');
c.ENERGY_RL=struct('version','energy_mission_v1','options',options, ...
 'time_weight',options.time_weight,'charge_weight',1-options.time_weight, ...
 'reference_charge_As',reference.charge_As/reference.distance_m*c.MISSION.D_TARGET_M, ...
 'reference',reference,'observation_schema','observation_energy_mission_v1', ...
 'reward_schema','reward_energy_mission_v1','training_promoted',false, ...
 'normalization_scope','Measured reference charge per meter; scaling is a reward unit, never predicted consumption or a safety limit.', ...
 'preference_scope','Provisional scalarization for Pareto comparison; no selected best time-energy tradeoff.');
c.IEQ_REF=c.BATTERY.C_nom_Ah*c.BATTERY.n_parallel; % 1C pack unit, not a current rating.
c.LEGACY_REWARD=c.REWARD;
c.REWARD=struct('version','reward_energy_mission_v1', ...
 'description','Completion priority with undiscounted elapsed-time and committed pack-charge costs.');
c.TRAIN.discount_factor=1;
c.EXPERIMENT.id=['energy_mission_' c.BATTERY.fixed_pack_spec.id '_tracked_swing_timing_v3'];
c.EXPERIMENT.description='High-SOC locked 18650 pack, tracked swing, actual charge objective, captured supervisory interface; no trained-policy claim.';
bundle.cfg=c;
end
