function setup_RL_MPC()
    gait = 0;
    p = get_params(gait);

    initial_R = [p.R(1,1); p.R(2,2); p.R(3,3)];
    lower_abs = 0.01 * initial_R;
    upper_abs = 5.00  * initial_R;

    % Observation: [nt; nu; last_R./upper_abs (3)]
    obsInfo  = rlNumericSpec([6 1], 'Name','observations');

    % Look again at this!! Make sure the 0.15 is actually possible
    actionInfo = rlNumericSpec([3 1], ...
        'LowerLimit', [-0.1; -0.15; -0.15], ...
        'UpperLimit',  [0.1; 0.15; 0.15], ...
        'Name','dR_frac');

    % Create env
    env = rlFunctionEnv(obsInfo, actionInfo, @rlStepFunction, @rlResetFunction);

    % Actor
    actorLayers = [
        featureInputLayer(6, 'Name','obs')
        fullyConnectedLayer(400, 'Name','actor_fc1')
        reluLayer('Name','actor_relu1')
        fullyConnectedLayer(300, 'Name','actor_fc2')
        reluLayer('Name','actor_relu2')
        fullyConnectedLayer(3,   'Name','actor_fc3')
        tanhLayer('Name','actor_tanh')
        scalingLayer('Name','action','Scale',0.10) % outputs in [-0.1, 0.1]
    ];

    actor = rlDeterministicActorRepresentation( ...
        actorLayers, obsInfo, actionInfo, ...
        'Observation',{'obs'}, 'Action',{'action'}, ...
        rlOptimizerOptions("LearnRate",1e-3,"GradientThreshold",1));

    % Critic (state path, action path, common path)
    statePath = [
        featureInputLayer(6, 'Name','obs')
        fullyConnectedLayer(400, 'Name','state_fc1')
        reluLayer('Name','state_relu1')
    ];

    actionPath = [
        featureInputLayer(3, 'Name','action')
        fullyConnectedLayer(400, 'Name','act_fc1')
        reluLayer('Name','act_relu1')
    ];

    commonPath = [
        additionLayer(2,'Name','add')
        fullyConnectedLayer(300,'Name','c_fc1')
        reluLayer('Name','c_relu1')
        fullyConnectedLayer(1,'Name','Q_value')
    ];

    criticLG = layerGraph(statePath);
    criticLG = addLayers(criticLG, actionPath);
    criticLG = addLayers(criticLG, commonPath);
    criticLG = connectLayers(criticLG, 'state_relu1','add/in1');
    criticLG = connectLayers(criticLG, 'act_relu1',  'add/in2');

    critic = rlQValueRepresentation( ...
        criticLG, obsInfo, actionInfo, ...
        'Observation',{'obs'}, 'Action',{'action'}, ...
        rlOptimizerOptions("LearnRate",1e-3,"GradientThreshold",1));

    % Agent
    agentOpts = rlDDPGAgentOptions( ...
        'SampleTime',1, ...
        'TargetSmoothFactor',1e-3, ...
        'DiscountFactor',0.99, ...
        'MiniBatchSize',64, ...
        'ExperienceBufferLength',1e6);

    % Exploration
    agentOpts.NoiseOptions.Variance = (0.08)^2; % σ ≈ 0.08 of ±0.10
    agentOpts.NoiseOptions.VarianceDecayRate = 0;

    agent = rlDDPGAgent(actor, critic, agentOpts);

    % Episode/hold configuration used by rlStepFunction
    cfg.EP_STEPS = 10; % steps per episode, X from g(X) in our cost fct
    cfg.APPLY_EVERY = 10;

    save('rlEnv_MPC_R.mat','env','agent','lower_abs','upper_abs','initial_R','cfg');
    disp('Setup Complete')
end

