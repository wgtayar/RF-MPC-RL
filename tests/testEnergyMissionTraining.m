function tests = testEnergyMissionTraining
    tests=functiontests(localfunctions);
end

function testAffineActionBoundsAndScale(testCase)
    cfg=localConfig();
    [low,audit]=energy_mission_action_transform(-ones(5,1),cfg);
    high=energy_mission_action_transform(ones(5,1),cfg);
    middle=energy_mission_action_transform(zeros(5,1),cfg);
    verifyEqual(testCase,low,audit.physical_lower,'AbsTol',1e-14);
    verifyEqual(testCase,high,audit.physical_upper,'AbsTol',1e-14);
    verifyEqual(testCase,middle,(low+high)/2,'AbsTol',1e-14);
    verifyEqual(testCase,0.1*audit.scale(1:3),5e-5*ones(3,1),'AbsTol',1e-15);
    [bounded,clip]=energy_mission_action_transform([2;-2;0;0;0],cfg);
    verifyTrue(testCase,clip.normalization_clipped);
    verifyEqual(testCase,bounded(1:2),[cfg.DR_MAX;-cfg.DR_MAX]);
end

function testRejectUnknownOrDiscountedOptions(testCase)
    verifyError(testCase,@() energy_mission_agent_options(struct('discount_factor',0.99)), ...
        'energy_mission_agent_options:UnknownOption');
    verifyError(testCase,@() energy_mission_agent_options(struct('typo',1)), ...
        'energy_mission_agent_options:UnknownOption');
    verifyError(testCase,@() energy_mission_agent_options(struct('mini_batch_size',65)), ...
        'energy_mission_agent_options:Inconsistent');
end

function testFreshAgentDimensionAndNoise(testCase)
    previous=rng;
    cleanup=onCleanup(@() rng(previous)); %#ok<NASGU>
    rng(20261104,'twister');
    bundle=setup_energy_mission_rl();
    cfg=bundle.cfg;
    schema=energy_mission_observation_schema(cfg);
    verifyEqual(testCase,schema.dimension,87);
    obs=rlNumericSpec([schema.dimension,1]);
    act=rlNumericSpec([5,1],'LowerLimit',-ones(5,1),'UpperLimit',ones(5,1));
    [agent,spec]=create_energy_mission_agent(obs,act,cfg,struct('hidden_units',8));
    verifyEqual(testCase,spec.observation_dimension,schema.dimension);
    verifyEqual(testCase,agent.AgentOptions.DiscountFactor,1);
    verifyEqual(testCase,agent.AgentOptions.SampleTime,5);
    verifyEqual(testCase,spec.sample_time_seconds,5);
    verifyEqual(testCase,agent.AgentOptions.ExplorationModel.StandardDeviation,0.1);
    verifyEqual(testCase,spec.exploration_physical_standard_deviation(1:3),5e-5*ones(3,1),'AbsTol',1e-15);
    output=getAction(getActor(agent),{zeros(schema.dimension,1)});
    verifySize(testCase,output{1},[5,1]);
    verifyTrue(testCase,all(abs(output{1})<=1));
    critics=getCritic(agent);
    verifyEqual(testCase,numel(critics),2);
    verifyFalse(testCase,isequaln(getLearnableParameters(critics(1)),getLearnableParameters(critics(2))));
end

function testRejectWrongObservationDimension(testCase)
    bundle=setup_energy_mission_rl();
    obs=rlNumericSpec([37,1]);
    act=rlNumericSpec([5,1],'LowerLimit',-ones(5,1),'UpperLimit',ones(5,1));
    verifyError(testCase,@() create_energy_mission_agent(obs,act,bundle.cfg), ...
        'create_energy_mission_agent:ObservationDimension');
end

function cfg=localConfig()
    cfg=struct('DR_MAX',0.0005,'GAMMA_V_MIN',0,'GAMMA_V_MAX',0.45, ...
        'GAMMA_A_MIN',0.1,'GAMMA_A_MAX',0.5,'CHUNK_DURATION',5,'APPLY_EVERY',10, ...
        'ENERGY_RL',struct('version','energy_mission_v1'));
end
