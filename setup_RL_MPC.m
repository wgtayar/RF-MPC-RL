function setup_RL_MPC()
    addpath fcns\ fcns_MPC\ 'RL Midtraining Logs'\
    rootDir = fileparts(mfilename('fullpath'));
    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');

    gait = 0;
    p = get_params(gait);

    initial_R = [p.R(1,1); p.R(2,2); p.R(3,3)];
    % lower_abs = 0.5 * initial_R;
    % upper_abs = 3.0 * initial_R;

    lower_abs = [0.7; 0.8; 0.8] .* initial_R;
    upper_abs = [1.4; 1.4; 1.4] .* initial_R;

    cfg.CHUNK_DURATION = 5;
    cfg.APPLY_EVERY = 10;
    cfg.MISSION_DURATION = 10 * 60;
    cfg.EP_STEPS = cfg.MISSION_DURATION / (cfg.CHUNK_DURATION * cfg.APPLY_EVERY);
    
    cfg.MISSION.D_TARGET_M = 300;
    cfg.MISSION.WINDOW_TARGET_M = cfg.MISSION.D_TARGET_M / cfg.EP_STEPS;
    
    cfg.V_MIN = 0.3;
    cfg.V_MAX = 1.1;
    cfg.A_MIN = 0.2;
    cfg.A_MAX = 4.0;
    cfg.TACC_MIN = 0.2;
    cfg.TACC_MAX = 2.0;
    cfg.RANDOMIZE_REQUEST = true;
    cfg.V_REQ_FIXED = 0.8;
    cfg.A_REQ_FIXED = 1.0;

    cfg.DR_MAX = 0.05; % was 0.25 then 0.15
    cfg.GAMMA_V_MIN = 0.1; % 0.5
    cfg.GAMMA_V_MAX = 0.5; % 1.0
    cfg.GAMMA_A_MIN = 0.1; % 0.5
    cfg.GAMMA_A_MAX = 0.5; % 1.0

    cfg.TRACK_REF = 16.21;
    cfg.EFFORT_REF = 5.4e4;
    cfg.IEQ_REF = 90;

    cfg.BATTERY.metric_type = 'soc';
    cfg.BATTERY.SOC_init = 0.95;
    cfg.BATTERY.metric_init = 100 * cfg.BATTERY.SOC_init;
    cfg.BATTERY.metric_min = 20;
    cfg.BATTERY.metric_max = 100;
    cfg.BATTERY.terminal_margin = 0.20;
    cfg.BATTERY.C_nom_Ah = 2.0;
    cfg.BATTERY.pack_voltage = 12;
    cfg.BATTERY.DoD = 0.8;
    cfg.BATTERY.use_pack_sizing = false;
    cfg.BATTERY.n_series = 4;
    cfg.BATTERY.n_parallel = 7;
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

    cfg.REWARD.w_dist = 12.0;
    cfg.REWARD.w_lag = 10.0;
    cfg.REWARD.w_risk = 2.5;
    cfg.REWARD.w_I = 0.15;
    cfg.REWARD.w_dsoc = 6.0;
    cfg.REWARD.w_track = 0.01;
    cfg.REWARD.w_effort = 0.005;
    cfg.REWARD.w_slow = 1.5;
    
    cfg.REWARD.v_floor_soft = 0.45;
    
    cfg.REWARD.risk_I_thr = 45.0;
    cfg.REWARD.risk_I_scale = 5.0;
    
    cfg.REWARD.risk_track_thr = cfg.TRACK_REF;
    cfg.REWARD.risk_track_scale = cfg.TRACK_REF;
    
    cfg.REWARD.risk_v_thr = 0.40;
    cfg.REWARD.risk_v_scale = 0.15;
    
    cfg.REWARD.risk_a_thr = 0.80;
    cfg.REWARD.risk_a_scale = 0.40;
    
    cfg.REWARD.risk_dv_thr = 0.08;
    cfg.REWARD.risk_dv_scale = 0.08;
    
    cfg.REWARD.risk_dgv_thr = 0.08;
    cfg.REWARD.risk_dgv_scale = 0.08;
    
    cfg.REWARD.risk_r2_thr = 0.02;
    cfg.REWARD.risk_r2_scale = 0.03;
    
    cfg.REWARD.alpha_I = 1.0;
    cfg.REWARD.alpha_track = 1.0;
    cfg.REWARD.alpha_v = 0.7;
    cfg.REWARD.alpha_a = 0.6;
    cfg.REWARD.alpha_dv = 1.2;
    cfg.REWARD.alpha_dgv = 1.2;
    cfg.REWARD.alpha_r2 = 0.8;
    
    cfg.REWARD.complete_bonus = 80;
    cfg.REWARD.early_bonus = 40;
    cfg.REWARD.final_soc_bonus = 20;
    
    cfg.REWARD.infeasible_base = 40;
    cfg.REWARD.infeasible_remaining = 80;
    
    cfg.REWARD.battery_base = 25;
    cfg.REWARD.battery_remaining = 60;
    
    cfg.REWARD.time_limit_base = 35;
    cfg.REWARD.time_limit_remaining = 70;

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
        rlOptimizerOptions("LearnRate", 1e-3, "GradientThreshold", 1));

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
        rlOptimizerOptions("LearnRate", 1e-3, "GradientThreshold", 1));

    agentOpts = rlDDPGAgentOptions( ...
        'SampleTime', 1, ...
        'TargetSmoothFactor', 1e-3, ...
        'DiscountFactor', 0.99, ...
        'MiniBatchSize', 64, ...
        'ExperienceBufferLength', 1e6);

    agentOpts.NoiseOptions.Variance = 0.15^2;
    agentOpts.NoiseOptions.VarianceDecayRate = 0;

    agent = rlDDPGAgent(actor, critic, agentOpts);

    save(cfgPath, 'env', 'agent', 'lower_abs', 'upper_abs', 'initial_R', 'cfg');
    disp('Setup Complete')
end