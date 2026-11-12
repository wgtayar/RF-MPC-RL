function scales = energy_mission_scales(cfg)
%ENERGY_MISSION_SCALES Physical accounting units and explicit preference weights.
% The reference charge is frozen measured demand scaled to this mission by
% its profile builder. It is neither a current limit nor a battery resize.
    assert(isfield(cfg,'ENERGY_RL') && isstruct(cfg.ENERGY_RL), ...
        'energyMission:Configuration','An explicit energy/mission profile is required.');
    policy = cfg.ENERGY_RL;
    required = {'reference_charge_As','time_weight','charge_weight'};
    assert(all(isfield(policy,required)), ...
        'energyMission:Configuration','Named charge reference and preference weights are required.');
    weights = [policy.time_weight,policy.charge_weight];
    validateattributes(weights,{'double'},{'real','finite','nonnegative','size',[1,2]});
    assert(abs(sum(weights)-1)<=1e-12,'energyMission:Weights', ...
        'Time and charge preference weights must sum to one.');
    validateattributes(policy.reference_charge_As,{'double'}, ...
        {'scalar','real','finite','positive'});
    validateattributes([cfg.MISSION_DURATION,cfg.MISSION.D_TARGET_M],{'double'}, ...
        {'real','finite','positive','size',[1,2]});
    battery = cfg.BATTERY;
    assert(~battery.use_pack_sizing,'energyMission:FixedPack','The pack must stay fixed.');
    validateattributes([battery.n_series,battery.n_parallel],{'double'}, ...
        {'real','finite','positive','integer','size',[1,2]});
    validateattributes(battery.C_nom_Ah,{'double'},{'scalar','real','finite','positive'});
    validateattributes([battery.SOC_init,battery.terminal_margin],{'double'}, ...
        {'real','finite','>=',0,'<=',1,'size',[1,2]});
    assert(battery.SOC_init>battery.terminal_margin,'energyMission:Reserve', ...
        'Initial charge must exceed the declared mission reserve.');
    if isfield(cfg,'TRAIN') && isfield(cfg.TRAIN,'discount_factor')
        assert(cfg.TRAIN.discount_factor==1,'energyMission:Discount', ...
            'The finite-horizon additive objective requires an undiscounted return.');
    end
    capacity = 3600*battery.n_parallel*battery.C_nom_Ah;
    rewardVersion = 'reward_energy_mission_v1';
    if isfield(cfg,'REWARD') && isfield(cfg.REWARD,'version') && ...
            ismember(cfg.REWARD.version,{'reward_energy_mission_v2','reward_energy_mission_v3'})
        rewardVersion = cfg.REWARD.version;
    end
    if ismember(rewardVersion,{'reward_energy_mission_v2','reward_energy_mission_v3'})
        % v2: the success bonus and the cumulative-charge bound use the
        % largest admissible start (bonus_reference_soc) so they do not
        % depend on the episode's starting SOC. Charge is still costed only
        % by the coulombs actually consumed in each window.
        assert(isfield(policy,'bonus_reference_soc'),'energyMission:Configuration', ...
            'Reward v2 requires a declared bonus reference SOC.');
        validateattributes(policy.bonus_reference_soc,{'double'}, ...
            {'scalar','real','finite','>',battery.terminal_margin,'<=',1});
        assert(battery.SOC_init<=policy.bonus_reference_soc+1e-12,'energyMission:Reserve', ...
            'Initial SOC cannot exceed the bonus reference SOC.');
        budget = capacity*(policy.bonus_reference_soc-battery.terminal_margin);
        scalesVersion = 'energy_mission_scales_v2';
        if strcmp(rewardVersion,'reward_energy_mission_v3'), scalesVersion = 'energy_mission_scales_v3'; end
    else
        budget = capacity*(battery.SOC_init-battery.terminal_margin);
        scalesVersion = 'energy_mission_scales_v1';
    end
    scales = struct('version',scalesVersion,'reward_version',rewardVersion, ...
        'target_distance_m',cfg.MISSION.D_TARGET_M, ...
        'mission_duration_s',cfg.MISSION_DURATION, ...
        'reference_charge_As',policy.reference_charge_As, ...
        'pack_capacity_As',capacity,'available_charge_budget_As',budget, ...
        'one_C_pack_current_A',battery.n_parallel*battery.C_nom_Ah, ...
        'pack_voltage_unit_V',battery.n_series, ...
        'time_weight',weights(1),'charge_weight',weights(2), ...
        'completion_bonus',2+weights(1)+weights(2)*budget/policy.reference_charge_As, ...
        'failure_penalty',1, ...
        'normalization_scope','Charge reference and 1C are units, not physical safety ratings.');
    if strcmp(rewardVersion,'reward_energy_mission_v3')
        % v3 (investigator 2026-11-12): time and charge are each scaled to
        % their achievable span, terminal +1/-1. Penalties are the excess
        % over the best pace for the progress made, so per-decision terms
        % telescope to tau=(T-Tmin)/(Tmax-Tmin), kappa=(Q-Qmin)/(Qmax-Qmin).
        s3 = policy.standardization;
        validateattributes([s3.max_speed_m_s,s3.charge_floor_As_per_m,s3.charge_span_As_per_m], ...
            {'double'},{'real','finite','positive','size',[1,3]});
        minimumTime = cfg.MISSION.D_TARGET_M/s3.max_speed_m_s;
        assert(minimumTime<cfg.MISSION_DURATION,'energyMission:Standardization', ...
            'The mission deadline must exceed the fastest possible mission.');
        scales.minimum_time_s = minimumTime;
        scales.time_span_s = cfg.MISSION_DURATION-minimumTime;
        scales.charge_floor_As_per_m = s3.charge_floor_As_per_m;
        scales.charge_span_As = s3.charge_span_As_per_m*cfg.MISSION.D_TARGET_M;
        scales.completion_bonus = 1;
        scales.failure_penalty = 1;
    end
end
