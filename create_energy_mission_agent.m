function [agent,spec] = create_energy_mission_agent(observationInfo,actionInfo,cfg,overrides)
%create_energy_mission_agent Fresh TD3 networks for the active observation schema.
% Algorithm/API: https://www.mathworks.com/help/reinforcement-learning/ug/td3-agents.html
% Options: https://www.mathworks.com/help/reinforcement-learning/ref/rl.option.rltd3agentoptions.html
    if nargin < 4, overrides=struct(); end
    settings = energy_mission_agent_options(overrides);
    assert(isfield(cfg,'ENERGY_RL'),'create_energy_mission_agent:Config', ...
        'Legacy configurations cannot be passed to the fresh energy-mission agent.');
    assert(numel(observationInfo)==1 && numel(actionInfo)==1 && ...
        prod(actionInfo.Dimension)==5 && all(actionInfo.LowerLimit(:)==-1) && ...
        all(actionInfo.UpperLimit(:)==1),'create_energy_mission_agent:ActionSpec', ...
        'The agent requires one observation channel and five normalized actions.');
    nObs = prod(observationInfo.Dimension);
    observationSchema = energy_mission_observation_schema(cfg);
    assert(nObs==observationSchema.dimension, ...
        'create_energy_mission_agent:ObservationDimension', ...
        'Observation dimensions must match the active energy-mission schema.');
    h = settings.hidden_units;
    actorNetwork = dlnetwork([
        featureInputLayer(nObs,'Normalization','none','Name','observation')
        fullyConnectedLayer(h,'Name','actor_fc1')
        reluLayer('Name','actor_relu1')
        fullyConnectedLayer(h,'Name','actor_fc2')
        reluLayer('Name','actor_relu2')
        fullyConnectedLayer(5,'Name','actor_output')
        tanhLayer('Name','normalized_action')]);
    actor = rlContinuousDeterministicActor(actorNetwork,observationInfo,actionInfo, ...
        'ObservationInputNames','observation');
    critic1 = localCritic(nObs,h,observationInfo,actionInfo);
    critic2 = localCritic(nObs,h,observationInfo,actionInfo);
    opt = rlTD3AgentOptions('SampleTime',cfg.CHUNK_DURATION*cfg.APPLY_EVERY, ...
        'DiscountFactor',1,'TargetSmoothFactor',settings.target_smooth_factor, ...
        'PolicyUpdateFrequency',settings.policy_update_frequency, ...
        'TargetUpdateFrequency',settings.target_update_frequency, ...
        'MiniBatchSize',settings.mini_batch_size, ...
        'NumWarmStartSteps',settings.num_warm_start_steps, ...
        'ExperienceBufferLength',settings.experience_buffer_length, ...
        'LearningFrequency',settings.learning_frequency, ...
        'MaxMiniBatchPerEpoch',settings.max_minibatch_per_epoch, ...
        'NumEpoch',settings.num_epoch);
    opt.ActorOptimizerOptions = rlOptimizerOptions('LearnRate',settings.actor_learn_rate, ...
        'GradientThreshold',settings.gradient_threshold);
    criticOpt = rlOptimizerOptions('LearnRate',settings.critic_learn_rate, ...
        'GradientThreshold',settings.gradient_threshold);
    opt.CriticOptimizerOptions = [criticOpt,criticOpt];
    opt.ExplorationModel.StandardDeviation = settings.exploration_standard_deviation;
    opt.ExplorationModel.StandardDeviationMin = settings.exploration_standard_deviation;
    opt.ExplorationModel.StandardDeviationDecayRate = 0;
    opt.TargetPolicySmoothModel.StandardDeviation = settings.target_standard_deviation;
    opt.TargetPolicySmoothModel.StandardDeviationMin = settings.target_standard_deviation;
    opt.TargetPolicySmoothModel.StandardDeviationDecayRate = 0;
    opt.TargetPolicySmoothModel.LowerLimit = -settings.target_noise_clip;
    opt.TargetPolicySmoothModel.UpperLimit = settings.target_noise_clip;
    agent = rlTD3Agent(actor,[critic1,critic2],opt);
    [~,transform] = energy_mission_action_transform(zeros(5,1),cfg);
    spec = settings;
    spec.observation_dimension = nObs;
    spec.observation_schema = observationSchema.version;
    spec.action_schema = transform.schema;
    spec.action_physical_scale = transform.scale;
    spec.action_physical_bias = transform.bias;
    spec.exploration_physical_standard_deviation = ...
        settings.exploration_standard_deviation*transform.scale;
    spec.target_physical_standard_deviation = settings.target_standard_deviation*transform.scale;
    spec.sample_time_seconds = cfg.CHUNK_DURATION*cfg.APPLY_EVERY;
    spec.network = 'two_hidden_relu_layers_equal_width_independent_twin_critics';
    spec.device = 'cpu';
    spec.objective_discount_reason = 'undiscounted_finite_mission_additive_time_charge_cost';
end

function critic = localCritic(nObs,h,observationInfo,actionInfo)
    graph = layerGraph(featureInputLayer(nObs,'Normalization','none','Name','observation'));
    graph = addLayers(graph,featureInputLayer(5,'Normalization','none','Name','action'));
    graph = addLayers(graph,[concatenationLayer(1,2,'Name','state_action')
        fullyConnectedLayer(h,'Name','critic_fc1')
        reluLayer('Name','critic_relu1')
        fullyConnectedLayer(h,'Name','critic_fc2')
        reluLayer('Name','critic_relu2')
        fullyConnectedLayer(1,'Name','q')]);
    graph = connectLayers(graph,'observation','state_action/in1');
    graph = connectLayers(graph,'action','state_action/in2');
    critic = rlQValueFunction(dlnetwork(graph),observationInfo,actionInfo, ...
        'ObservationInputNames','observation','ActionInputNames','action');
end
