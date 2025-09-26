% function setup_RL_MPC()
%     gait = 0;
%     p = get_params(gait);
%     initial_R_weights_unique = [p.R(1,1); p.R(2,2); p.R(3,3)]; % HAA, HFE, KFE
% 
%     % Observation space: tracking error + control effort
%     obsInfo = rlNumericSpec([2 1], 'Name', 'observations');
% 
%     % Action space (3 unique weights: HAA, HFE, KFE)
%     lower_bound = 0.01 * initial_R_weights_unique;
%     upper_bound = 5 * initial_R_weights_unique;
%     actionInfo = rlNumericSpec([3 1], ...
%         'LowerLimit', lower_bound, ...
%         'UpperLimit', upper_bound, ...
%         'Name', 'R_weights');
% 
%     % RL environment
%     env = rlFunctionEnv(obsInfo, actionInfo, @rlStepFunction, @rlResetFunction);
% 
%     % Actor network
%     actorNetwork = [
%         featureInputLayer(2, 'Name', 'observations')
%         fullyConnectedLayer(400, 'Name', 'fc1')
%         reluLayer('Name', 'relu1')
%         fullyConnectedLayer(300, 'Name', 'fc2')
%         reluLayer('Name', 'relu2')
%         fullyConnectedLayer(3, 'Name', 'fc3')
%         softplusLayer('Name', 'softplus') % ensure strictly positive outputs
%         scalingLayer('Name', 'scale', 'Scale', upper_bound) % scale to action bounds
%     ];
% 
%     % Initialize actor to output initial weights
%     % actorNetwork(6).Bias = initial_R_weights_unique ./ upper_bound; % scaled to [0,1]
% 
%     % actorOpts = rlRepresentationOptions('LearnRate',1e-4,'GradientThreshold',1);
%     actorOpts = rlOptimizerOptions("LearnRate",1e-3,"GradientThreshold",1);
%     actor = rlDeterministicActorRepresentation(actorNetwork, ...
%         obsInfo, actionInfo, ...
%         'Observation', {'observations'}, ...
%         'Action', {'scale'}, ...
%         actorOpts);
% 
%     % Critic network (unchanged except for action size)
%     statePath = [
%         featureInputLayer(2, 'Name', 'observations')
%         fullyConnectedLayer(400, 'Name', 'fc1')
%         reluLayer('Name', 'relu1')
%     ];
%     actionPath = [
%         featureInputLayer(3, 'Name', 'scale')
%         fullyConnectedLayer(400, 'Name', 'fc2')
%         reluLayer('Name', 'relu2')
%     ];
%     commonPath = [
%         additionLayer(2,'Name','add')
%         fullyConnectedLayer(300, 'Name', 'fc3')
%         reluLayer('Name', 'relu3')
%         fullyConnectedLayer(1, 'Name', 'Q_value')
%     ];
%     criticNetwork = layerGraph(statePath);
%     criticNetwork = addLayers(criticNetwork, actionPath);
%     criticNetwork = addLayers(criticNetwork, commonPath);
%     criticNetwork = connectLayers(criticNetwork, 'relu1', 'add/in1');
%     criticNetwork = connectLayers(criticNetwork, 'relu2', 'add/in2');
% 
%     % criticOpts = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1);
%     criticOpts = rlOptimizerOptions("LearnRate",1e-3,"GradientThreshold",1);
%     critic = rlQValueRepresentation(criticNetwork, ...
%         obsInfo, actionInfo, ...
%         'Observation', {'observations'}, ...
%         'Action', {'scale'}, ...
%         criticOpts);
% 
%     % DDPG Agent options
%     agentOpts = rlDDPGAgentOptions(...
%         'SampleTime',1, ...
%         'TargetSmoothFactor',1e-3, ...
%         'DiscountFactor',0.99, ...
%         'MiniBatchSize',64, ...
%         'ExperienceBufferLength',1e6);
%     agentOpts.NoiseOptions.Variance = 0.01 * mean(initial_R_weights_unique); % small initial noise
%     agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;
% 
%     agent = rlDDPGAgent(actor, critic, agentOpts);
% 
%     % Save environment and agent
%     save(fullfile(pwd, 'rlEnv_MPC_R.mat'), 'env', 'agent');
%     disp('RL environment and agent saved with symmetric p.R weights.');
% end


function setup_RL_MPC()
    gait = 0;
    p = get_params(gait);

    initial_R = [p.R(1,1); p.R(2,2); p.R(3,3)];
    lower_abs = 0.01 * initial_R;
    upper_abs = 5.00  * initial_R;

    % Observation: [nt; nu; last_R./upper_abs (3); sin(wt); cos(wt)]
    obsInfo  = rlNumericSpec([7 1], 'Name','observations');

    % Look again at this!! Make sure the 0.15 is actually possible
    actionInfo = rlNumericSpec([3 1], ...
        'LowerLimit', [-0.1; -0.15; -0.15], ...
        'UpperLimit',  [0.1; 0.15; 0.15], ...
        'Name','dR_frac');

    % Create env first (validation calls reset() then step())
    env = rlFunctionEnv(obsInfo, actionInfo, @rlStepFunction, @rlResetFunction);

    % Actor (one layer per row; no commas)
    actorLayers = [
        featureInputLayer(7, 'Name','obs')
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
        featureInputLayer(7, 'Name','obs')
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

    % === Agent ===
    agentOpts = rlDDPGAgentOptions( ...
        'SampleTime',1, ...
        'TargetSmoothFactor',1e-3, ...
        'DiscountFactor',0.99, ...
        'MiniBatchSize',64, ...
        'ExperienceBufferLength',1e6);

    % Exploration: wide; no early decay
    agentOpts.NoiseOptions.Variance = (0.08)^2; % σ ≈ 0.08 of ±0.10
    agentOpts.NoiseOptions.VarianceDecayRate = 0;

    agent = rlDDPGAgent(actor, critic, agentOpts);

    % Episode/hold configuration used by rlStepFunction
    cfg.EP_STEPS = 10; % steps per episode
    cfg.APPLY_EVERY = 10; % apply a new action once per episode (hold rest)

    save('rlEnv_MPC_R.mat','env','agent','lower_abs','upper_abs','initial_R','cfg');
    disp('Saved env/agent. v2.1: fixed layer arrays (no commas), explicit names.');
end

