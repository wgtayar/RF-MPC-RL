function policy = load_phase3_policy(id)
%load_phase3_policy Load an explicit baseline or fingerprinted historical actor.
    source = bootstrap_RF_MPC_RL();
    id = char(id);
    baselines = {'mission_average','fixed_R','fast_start','slow_start','hand_ramp','conservative'};
    policy = struct('id',id,'kind','baseline','actor',[], ...
        'checkpoint_path','','checkpoint_sha256','', ...
        'observation_schema','observation_v1_legacy', ...
        'inference_mode','deterministic_baseline','historical_configuration_verified',false);
    if ismember(id,baselines)
        return
    end
    switch id
        case 'Agent74'
            expected = '7006b492c0afd32999fe40977807702871a9c7f63a75db6fee7a2b911b5b3028';
        case 'Agent75'
            expected = '959ba19ab7d57b3c1caf338b29dfe871c4b766e539602e3a4ce4f7498ba31851';
        otherwise
            error('load_phase3_policy:UnknownPolicy','Unknown policy %s.',id);
    end
    path = fullfile(source,'RL Midtraining Logs','Successful Agents', ...
        'run_2026-07-03_05-38-23',[id,'.mat']);
    actual = sha256_file(path);
    assert(strcmp(actual,expected),'load_phase3_policy:CheckpointHash', ...
        'Historical checkpoint fingerprint differs; do not substitute a warm-start copy.');
    saved = load(path,'saved_agent');
    actor = getActor(saved.saved_agent);
    observation = getObservationInfo(saved.saved_agent);
    action = getActionInfo(saved.saved_agent);
    assert(isa(actor,'rl.function.rlContinuousDeterministicActor') && ...
        isequal(observation.Dimension,[19,1]) && isequal(action.Dimension,[5,1]), ...
        'load_phase3_policy:Interface','This evaluator requires a deterministic 19-to-5 actor.');
    policy.kind = 'actor';
    policy.actor = actor;
    policy.checkpoint_path = path;
    policy.checkpoint_sha256 = actual;
    policy.actor_action_lower = action.LowerLimit;
    policy.actor_action_upper = action.UpperLimit;
    policy.inference_mode = 'actor_evaluate_prediction_no_exploration';
end
