function setup_RL_MPC()

    rootDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(rootDir, 'fcns'));
    addpath(fullfile(rootDir, 'fcns_MPC'));
    addpath(fullfile(rootDir, 'RL Midtraining Logs'));

    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');

    gait = 0;
    p = get_params(gait);

    initial_R = [p.R(1,1); p.R(2,2); p.R(3,3)];

    lower_abs = [0.95; 0.95; 0.95] .* initial_R;
    upper_abs = [1.05; 1.05; 1.05] .* initial_R;

    cfg.CHUNK_DURATION = 5;
    cfg.APPLY_EVERY = 10;
    cfg.MISSION_DURATION = 10 * 60;
    cfg.EP_STEPS = cfg.MISSION_DURATION / (cfg.CHUNK_DURATION * cfg.APPLY_EVERY);

    cfg.MISSION.D_TARGET_M = 320;
    cfg.MISSION.WINDOW_TARGET_M = cfg.MISSION.D_TARGET_M / cfg.EP_STEPS;

    cfg.V_MIN = 0.3;
    cfg.V_MAX = 1.1;
    cfg.A_MIN = 0.2;
    cfg.A_MAX = 4.0;
    cfg.TACC_MIN = 0.2;
    cfg.TACC_MAX = 2.0;
    cfg.RANDOMIZE_REQUEST = false;
    cfg.V_REQ_FIXED = cfg.MISSION.D_TARGET_M / cfg.MISSION_DURATION;
    cfg.A_REQ_FIXED = 1.0;

    % Low but nonzero R authority. This avoids the artificial DR0 constraint
    % while keeping R transitions gentler than the old 0.002 setting.
    cfg.DR_MAX = 0.0005;

    % Stability cap: v_cap = 0.3 + 0.45*(1.1-0.3) = 0.66 m/s.
    cfg.GAMMA_V_MIN = 0.0;
    cfg.GAMMA_V_MAX = 0.45;
    cfg.GAMMA_A_MIN = 0.1;
    cfg.GAMMA_A_MAX = 0.5;

    cfg.GAMMA_V_MISSION = (cfg.MISSION.D_TARGET_M / cfg.MISSION_DURATION - cfg.V_MIN) / ...
        max(cfg.V_MAX - cfg.V_MIN, eps);
    cfg.GAMMA_V_MISSION = min(max(cfg.GAMMA_V_MISSION, cfg.GAMMA_V_MIN), cfg.GAMMA_V_MAX);

    cfg.DGAMMA_V_MAX = 0.03;

    % High-start ablation disabled. Leave fields present for compatibility.
    cfg.START_HIGH.enable = false;
    cfg.START_HIGH.gamma_v_init = cfg.GAMMA_V_MISSION;
    cfg.START_HIGH.gamma_v_floor = cfg.GAMMA_V_MISSION;
    cfg.START_HIGH.n_floor_decisions = 0;

    cfg.TRACK_REF = 16.21;
    cfg.EFFORT_REF = 5.4e4;
    cfg.IEQ_REF = 90;

    cfg.BATTERY.metric_type = 'soc';
    cfg.BATTERY.SOC_init = 0.95;
    cfg.BATTERY.metric_init = 100 * cfg.BATTERY.SOC_init;
    cfg.BATTERY.metric_min = 20;
    cfg.BATTERY.metric_max = 100;
    cfg.BATTERY.terminal_margin = 0.2;
    cfg.BATTERY.C_nom_Ah = 2.0;
    cfg.BATTERY.pack_voltage = 12;
    cfg.BATTERY.DoD = 0.8;
    cfg.BATTERY.use_pack_sizing = false;
    cfg.BATTERY.n_series = 4;
    cfg.BATTERY.n_parallel = 6;
    cfg.BATTERY.decim = 10;
    cfg.BATTERY.make_plots = false;

    cfg.PROXY.betaMc = 0.5;
    cfg.PROXY.kneeCsv = fullfile(rootDir, 'Symmetric Knee Torques.csv');
    cfg.PROXY.alpha = 1;
    cfg.PROXY.beta = 2;
    cfg.PROXY.Kt = 0.0909;
    cfg.PROXY.eta = 0.90;
    cfg.PROXY.Nknee = 6 / 1.55;
    cfg.PROXY.tauHip4_joint = 6.114;
    cfg.PROXY.Nhip = 6;

    cfg.OBS.COM_SPEED_MAX = 8.0;
    cfg.OBS.TST_RATIO_MIN = 0.5;
    cfg.OBS.TST_RATIO_MAX = 1.1;
    cfg.OBS.STATE_NORM_MAX = 150.0;
    cfg.OBS.NOMINAL_TST = p.Tst;

    cfg.EXPERIMENT.id = 'scratch_320m_gv045_DR0005_oldPace_noStateRisk_v1';
    cfg.EXPERIMENT.description = ...
        'Stability recovery: 320m, gamma_v_max=0.45, DR_MAX=0.0005, old progress-first pace reward, no state-norm risk, adaptive shaping disabled.';

    cfg.REWARD.version = 'old_pace_stability_recovery_no_state_risk_v1_2026_07_03';
    cfg.REWARD.description = ...
        'Progress-first old pace reward, no adaptive shaping, no mission guard, terminal SOC bonus restored temporarily, state-norm risk disabled.';

    cfg.REWARD.w_pace = 8.0;
    cfg.REWARD.w_shortfall = 16.0;
    cfg.REWARD.w_ahead = 4.0;

    cfg.REWARD.w_lag_linear = 18.0;
    cfg.REWARD.w_lag_quad = 8.0;

    cfg.REWARD.w_risk = 2.0;

    cfg.REWARD.w_I = 0.15;
    cfg.REWARD.w_dsoc = 6.0;
    cfg.REWARD.w_track = 0.01;
    cfg.REWARD.w_effort = 0.005;
    cfg.REWARD.w_slow = 1.5;

    cfg.REWARD.nt_cap = 50.0;
    cfg.REWARD.nu_cap = 50.0;

    cfg.REWARD.v_floor_soft = 0.45;

    cfg.REWARD.soc_safe_thresh = 0.50;
    cfg.REWARD.soc_terminal_thresh = 0.2;
    cfg.REWARD.soc_gate_strength = 7.0;

    cfg.REWARD.risk_component_cap = 5.0;

    cfg.REWARD.risk_I_thr = 45.0;
    cfg.REWARD.risk_I_scale = 15.0;

    cfg.REWARD.risk_track_thr = cfg.TRACK_REF;
    cfg.REWARD.risk_track_ref = 1000.0;

    cfg.REWARD.risk_a_thr = 0.80;
    cfg.REWARD.risk_a_scale = 0.40;

    cfg.REWARD.risk_dv_thr = 0.06;
    cfg.REWARD.risk_dv_scale = 0.06;

    cfg.REWARD.risk_dgv_thr = 0.06;
    cfg.REWARD.risk_dgv_scale = 0.06;

    cfg.REWARD.risk_r2_thr = 0.02;
    cfg.REWARD.risk_r2_scale = 0.03;

    cfg.REWARD.dynamic_gate_v_thr = 0.35;
    cfg.REWARD.dynamic_gate_v_scale = 0.15;

    cfg.REWARD.alpha_I = 1.0;
    cfg.REWARD.alpha_track = 1.0;
    cfg.REWARD.alpha_a = 0.6;
    cfg.REWARD.alpha_dv = 1.2;
    cfg.REWARD.alpha_dgv = 1.2;
    cfg.REWARD.alpha_r2 = 0.8;

    % State norm appears distance-contaminated, so disable these terms.
    cfg.REWARD.alpha_state = 0.0;
    cfg.REWARD.alpha_com = 0.6;
    cfg.REWARD.alpha_speed_state = 0.0;

    % Temporarily restore a modest terminal SOC bonus to reproduce the stable
    % learning landscape. Remove after mission completion is stable.
    cfg.REWARD.complete_bonus = 200;
    cfg.REWARD.early_bonus = 80;
    cfg.REWARD.final_soc_bonus = 30;

    cfg.REWARD.infeasible_base = 70;
    cfg.REWARD.infeasible_remaining = 120;
    cfg.REWARD.infeasible_lag = 50;

    cfg.REWARD.battery_base = 35;
    cfg.REWARD.battery_remaining = 90;

    cfg.REWARD.time_limit_base = 50;
    cfg.REWARD.time_limit_remaining = 120;

    cfg.REWARD.min_window_target_m = cfg.V_MIN * cfg.CHUNK_DURATION * cfg.APPLY_EVERY;

    cfg.REWARD.I_budget_high = 65.0;
    cfg.REWARD.I_budget_low = 47.0;
    cfg.REWARD.I_budget_scale = 10.0;
    cfg.REWARD.w_I_budget = 2.0;

    cfg.REWARD.risk_state_thr = 180.0;
    cfg.REWARD.risk_state_scale = 120.0;

    cfg.REWARD.risk_com_thr = 0.55;
    cfg.REWARD.risk_com_scale = 0.30;

    cfg.REWARD.v_margin_above_req = 0.03;
    cfg.REWARD.v_excess_scale = 0.08;

    % Mission guard retained for logging/future experiments but disabled.
    cfg.REWARD.behind_lag_free = 0.01;
    cfg.REWARD.behind_lag_width = 0.06;
    cfg.REWARD.v_shortfall_scale = 0.08;
    cfg.REWARD.w_v_shortfall = 0.0;

    % Adaptive shaping retained for compatibility/logging but disabled.
    cfg.REWARD.ADAPT.enable = false;
    cfg.REWARD.ADAPT.lag_tolerance = 0.015;
    cfg.REWARD.ADAPT.schedule_gate_width = 0.060;
    cfg.REWARD.ADAPT.conserve_above50_weight = 0.25;
    cfg.REWARD.ADAPT.w_excess_speed = 0.35;
    cfg.REWARD.ADAPT.v_excess_slack = 0.020;
    cfg.REWARD.ADAPT.v_excess_scale = 0.080;
    cfg.REWARD.ADAPT.w_cap_use = 0.05;
    cfg.REWARD.ADAPT.cap_band = 0.015;
    cfg.REWARD.ADAPT.w_current_conserve = 0.0;
    cfg.REWARD.ADAPT.I_conserve_high = 52.0;
    cfg.REWARD.ADAPT.I_conserve_low = 46.0;
    cfg.REWARD.ADAPT.I_conserve_scale = 8.0;
    cfg.REWARD.ADAPT.w_efficiency_bonus = 0.0;
    cfg.REWARD.ADAPT.w_terminal_soc = 0.0;

    cfg.REWARD.q_pace_neutral = NaN;
    cfg.REWARD.q_pace_cap = NaN;

    cfg.RESET_R_EACH_EPISODE = true;

    cfg.LOG.enable = false;
    cfg.LOG.print_chunk = true;
    cfg.LOG.print_decision = true;
    cfg.LOG.print_episode = true;

    cfg.RUN.enabled = false;
    cfg.RUN.root_dir = '';
    cfg.RUN.run_dir = '';
    cfg.RUN.run_stamp = '';
    cfg.RUN.log_file = '';
    cfg.RUN.checkpoint_file = '';

    cfg.CHECKPOINT.every_decisions = 3;

    save(cfgPath, 'lower_abs', 'upper_abs', 'initial_R', 'cfg');

    obsInfo = rlNumericSpec([19 1], 'Name', 'observations');

    actionLower = [-cfg.DR_MAX; -cfg.DR_MAX; -cfg.DR_MAX; cfg.GAMMA_V_MIN; cfg.GAMMA_A_MIN];
    actionUpper = [cfg.DR_MAX; cfg.DR_MAX; cfg.DR_MAX; cfg.GAMMA_V_MAX; cfg.GAMMA_A_MAX];
    actionInfo = rlNumericSpec([5 1], ...
        'LowerLimit', actionLower, ...
        'UpperLimit', actionUpper, ...
        'Name', 'supervisory_action');

    env = rlFunctionEnv(obsInfo, actionInfo, @rlStepFunction, @rlResetFunction);

    actionScale = (actionUpper - actionLower) / 2;
    actionBias = (actionUpper + actionLower) / 2;

    actorLayers = [
        featureInputLayer(19, 'Name', 'obs')
        fullyConnectedLayer(400, 'Name', 'actor_fc1')
        reluLayer('Name', 'actor_relu1')
        fullyConnectedLayer(300, 'Name', 'actor_fc2')
        reluLayer('Name', 'actor_relu2')
        fullyConnectedLayer(5, 'Name', 'actor_fc3')
        tanhLayer('Name', 'actor_tanh')
        scalingLayer('Name', 'action', 'Scale', actionScale, 'Bias', actionBias)
    ];

    actor = rlDeterministicActorRepresentation( ...
        actorLayers, obsInfo, actionInfo, ...
        'Observation', {'obs'}, 'Action', {'action'}, ...
        rlOptimizerOptions('LearnRate', 1e-3, 'GradientThreshold', 1));

    statePath = [
        featureInputLayer(19, 'Name', 'obs')
        fullyConnectedLayer(400, 'Name', 'state_fc1')
        reluLayer('Name', 'state_relu1')
    ];

    actionPath = [
        featureInputLayer(5, 'Name', 'action')
        fullyConnectedLayer(400, 'Name', 'act_fc1')
        reluLayer('Name', 'act_relu1')
    ];

    commonPath = [
        additionLayer(2, 'Name', 'add')
        fullyConnectedLayer(300, 'Name', 'c_fc1')
        reluLayer('Name', 'c_relu1')
        fullyConnectedLayer(1, 'Name', 'Q_value')
    ];

    criticLG = layerGraph(statePath);
    criticLG = addLayers(criticLG, actionPath);
    criticLG = addLayers(criticLG, commonPath);
    criticLG = connectLayers(criticLG, 'state_relu1', 'add/in1');
    criticLG = connectLayers(criticLG, 'act_relu1', 'add/in2');

    critic = rlQValueRepresentation( ...
        criticLG, obsInfo, actionInfo, ...
        'Observation', {'obs'}, 'Action', {'action'}, ...
        rlOptimizerOptions('LearnRate', 1e-3, 'GradientThreshold', 1));

    agentOpts = rlDDPGAgentOptions( ...
        'SampleTime', 1, ...
        'TargetSmoothFactor', 1e-3, ...
        'DiscountFactor', 0.99, ...
        'MiniBatchSize', 64, ...
        'ExperienceBufferLength', 1e6);

    agentOpts.NoiseOptions.Variance = 0.04^2;
    agentOpts.NoiseOptions.VarianceDecayRate = 0;

    agent = rlDDPGAgent(actor, critic, agentOpts);

    save(cfgPath, 'env', 'agent', 'lower_abs', 'upper_abs', 'initial_R', 'cfg');

    fprintf('Setup Complete: %s\n', cfg.EXPERIMENT.id);
    fprintf('Mission %.1f m, gamma_v_max %.3f, v_cap %.3f m/s, DR_MAX %.6f\n', ...
        cfg.MISSION.D_TARGET_M, cfg.GAMMA_V_MAX, ...
        cfg.V_MIN + cfg.GAMMA_V_MAX * (cfg.V_MAX - cfg.V_MIN), cfg.DR_MAX);
end
