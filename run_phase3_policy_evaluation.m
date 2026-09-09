function report = run_phase3_policy_evaluation(policyId, outputRoot, options)
%run_phase3_policy_evaluation Evaluate a frozen policy with complete MPC capture.
% Defaults to a full current-config mission, legacy reference and default QP
% solver. max_decisions limits a probe without changing the mission definition.
    if nargin < 3
        options = struct();
    end
    options = localOptions(options);
    source = bootstrap_RF_MPC_RL();
    bundle = load(fullfile(source,'rlEnv_MPC_R.mat'),'cfg','initial_R','lower_abs','upper_abs');
    bundle.cfg.RNG_SEED = options.seed;
    bundle.cfg.POLICY_EVALUATION = struct('policy_id',char(policyId),'options',options, ...
        'scenario','current_config_not_claimed_historical_reproduction');
    previousRng = rng;
    rngCleanup = onCleanup(@() rng(previousRng));
    rng(options.seed,'twister');
    policy = load_phase3_policy(policyId);
    environmentOptions = struct('reference_mode',options.reference_mode,'solver_strategy',options.solver_strategy);
    if isfield(options,'initial_condition')
        environmentOptions.initial_condition = options.initial_condition;
    end
    if isfield(options,'observation_schema')
        assert(~strcmp(policy.kind,'actor') || strcmp(options.observation_schema,policy.observation_schema), ...
            'run_phase3_policy_evaluation:LegacyObservation','Historical actors require their original observation schema.');
        environmentOptions.observation_schema = options.observation_schema;
        policy.observation_schema = options.observation_schema;
    end
    [~,runId] = fileparts(outputRoot);
    metadata = struct('run_id',runId,'run_type','deterministic_policy_evaluation', ...
        'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'), ...
        'seed',options.seed,'source_policy',policy.id,'policy',rmfield(policy,'actor'));
    env = Phase3MpcEnvironment(bundle,outputRoot,metadata,environmentOptions);
    try
        report = localEvaluate(env,policy,bundle,options,runId);
    catch exception
        if ~env.Writer.Closed
            failure = struct('identifier',exception.identifier,'message',exception.message);
            save(fullfile(env.Writer.Root,'evaluation_exception.mat'),'failure');
            close(env,'policy_evaluation_exception');
        end
        rethrow(exception)
    end
    report.validation = validate_exact_state_dataset(outputRoot);
    assert(report.checkpoint_unchanged,'run_phase3_policy_evaluation:CheckpointChanged', ...
        'The original checkpoint changed during evaluation.');
    disp(rmfield(report,{'policy','metrics','validation'}));
end

function report = localEvaluate(env,policy,bundle,options,runId)
    if ~isempty(policy.checkpoint_path)
        mkdir(fullfile(env.Writer.Root,'policies'));
        target = fullfile(env.Writer.Root,'policies','source_checkpoint.mat');
        copyfile(policy.checkpoint_path,target);
        assert(strcmp(sha256_file(target),policy.checkpoint_sha256), ...
            'run_phase3_policy_evaluation:CheckpointCopy','Archived checkpoint bytes differ.');
    end
    observation = reset(env);
    status = 'probe_decision_limit';
    timer = tic;
    while env.Decision < options.max_decisions && ~env.Done
        candidate = phase3_policy_candidate(policy,observation,env.State.t,bundle.cfg);
        [observation,~,done,info] = step(env,candidate);
        fprintf('[%s] decision=%d time=%.2f distance=%.5f SOC=%.4f terminal=%s\n', ...
            policy.id,env.Decision,env.State.t,info.window.distance_end_m, ...
            env.State.battery.soc_pct,info.terminal_reason);
        if done
            status = info.terminal_reason;
        end
    end
    report = struct('source_sha',env.Writer.Manifest.source_sha,'source_dirty',env.Writer.Manifest.source_dirty, ...
        'run_id',runId,'policy',rmfield(policy,'actor'),'options',options,'outcome',status, ...
        'mission_complete',strcmp(status,'mission_complete'), ...
        'rollout_wall_s',toc(timer),'metrics',phase3_episode_metrics(env.Writer.Root,env.Config,env.Episode));
    report.checkpoint_unchanged = true;
    if ~isempty(policy.checkpoint_path)
        report.checkpoint_unchanged = strcmp(sha256_file(policy.checkpoint_path),policy.checkpoint_sha256);
    end
    save(fullfile(env.Writer.Root,'policy_evaluation.mat'),'report','-v7.3');
    close(env,status);
end

function options = localOptions(options)
    defaults = struct('reference_mode','legacy_absolute_time','solver_strategy','default', ...
        'max_decisions',inf,'seed',20260903);
    assert(all(ismember(fieldnames(options),[fieldnames(defaults);{'initial_condition';'observation_schema'}])), ...
        'run_phase3_policy_evaluation:Options','Unknown evaluation option.');
    if isfield(options,'initial_condition')
        options.initial_condition = validate_phase3_initial_condition(options.initial_condition);
    end
    if isfield(options,'observation_schema')
        options.observation_schema = validatestring(options.observation_schema, ...
            {'observation_v1_legacy','observation_v2_dynamic_health_candidate_v1', ...
            'observation_v2_dynamic_health_candidate_v2'});
    end
    for name = fieldnames(defaults).'
        if ~isfield(options,name{1})
            options.(name{1}) = defaults.(name{1});
        end
    end
    validateattributes(options.seed,{'double'},{'scalar','integer','nonnegative','finite','<=',2^32-1});
    validateattributes(options.max_decisions,{'double'},{'scalar','positive','real'});
    assert(isscalar(options.max_decisions) && options.max_decisions > 0 && ...
        (isinf(options.max_decisions) || options.max_decisions == round(options.max_decisions)), ...
        'run_phase3_policy_evaluation:DecisionLimit','Decision limit must be a positive integer or Inf.');
end
