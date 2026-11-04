function report = train_energy_mission_rl(outputRoot,options)
%train_energy_mission_rl Fresh, captured TD3 build or bounded native training.
% The default mode is build_only. Explicit mode='train' enables learning.
% A shorter max_steps_per_episode is a diagnostic truncation, not a complete
% mission or a performance result. No legacy worker MAT or actor is reused.
    if nargin < 2, options=struct(); end
    source = bootstrap_RF_MPC_RL();
    options = localOptions(options);
    sourceStart = localSourceState(source);
    training = strcmp(options.mode,'train');
    if training
        assert(~sourceStart.dirty,'train_energy_mission_rl:DirtySource', ...
            'Training requires a clean committed source tree before creating any output.');
        outputCanonical=char(java.io.File(char(outputRoot)).getCanonicalPath());
        assert(~strcmp(outputCanonical,source) && ...
            ~startsWith(string(outputCanonical),string(source)+filesep), ...
            'train_energy_mission_rl:OutputInsideSource', ...
            'Training evidence must be stored outside the source checkout.');
    end
    previousRng = rng;
    rngCleanup = onCleanup(@() rng(previousRng)); %#ok<NASGU>
    if ~isempty(options.seed), options.setup_options.seed=options.seed; end
    bundle = setup_energy_mission_rl(options.setup_options);
    options.seed = bundle.cfg.RNG_SEED;
    if isempty(options.max_steps_per_episode)
        options.max_steps_per_episode = bundle.cfg.EP_STEPS;
    end
    assert(options.max_steps_per_episode<=bundle.cfg.EP_STEPS, ...
        'train_energy_mission_rl:StepLimit','The step limit cannot exceed the configured finite mission horizon.');
    settings = energy_mission_agent_options(options.agent_options);
    bundle.cfg.TRAIN = settings;
    bundle.cfg.TRAIN.run_options = options;
    rng(options.seed,'twister');
    initialRng = rng;
    [~,runId] = fileparts(outputRoot);
    metadata = struct('run_id',runId,'run_type','energy_mission_td3_training_interface_v1', ...
        'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'), ...
        'seed',options.seed,'source_policy','fresh_energy_mission_td3_v1', ...
        'configuration_origin','fresh_setup_energy_mission_rl','training',training, ...
        'hyperparameters_validated_for_performance',false, ...
        'learning_rates_configured',[settings.actor_learn_rate settings.critic_learn_rate], ...
        'exploration_standard_deviation_configured_normalized',settings.exploration_standard_deviation, ...
        'learning_rates_used',[],'exploration_noise_used',[], ...
        'source_start',sourceStart);
    if training
        metadata.learning_rates_used = metadata.learning_rates_configured;
        metadata.exploration_noise_used = struct('type','independent_gaussian_normalized', ...
            'standard_deviation',settings.exploration_standard_deviation,'decay_rate',0);
    end
    env = [];
    trainingDir = '';
    wallClock = tic;
    try
        env = EnergyMissionTrainingEnvironment(bundle,outputRoot,metadata,options.environment_options);
        root = env.Native.Writer.Root;
        trainingDir = fullfile(root,'training');
        mkdir(trainingDir);
        diary(fullfile(trainingDir,'console.log'));
        diaryCleanup = onCleanup(@() diary('off')); %#ok<NASGU>
        save(fullfile(trainingDir,'request.mat'),'options','initialRng','metadata');
        [agent,agentSpec] = create_energy_mission_agent(getObservationInfo(env), ...
            getActionInfo(env),bundle.cfg,options.agent_options);
        before = localParameters(agent);
        save(fullfile(trainingDir,'initial_agent.mat'),'agent','agentSpec','initialRng','-v7.3');
        localJson(fullfile(trainingDir,'agent_specification.json'),agentSpec);
        stats = [];
        episodes = 0;
        steps = 0;
        if training
            trainOptions = rlTrainingOptions('MaxEpisodes',options.max_episodes, ...
                'MaxStepsPerEpisode',options.max_steps_per_episode, ...
                'ScoreAveragingWindowLength',min(20,options.max_episodes), ...
                'Verbose',true,'Plots','none','UseParallel',false, ...
                'StopTrainingCriteria','EpisodeCount','StopTrainingValue',options.max_episodes, ...
                'SaveAgentCriteria','EpisodeFrequency','SaveAgentValue',1, ...
                'SaveAgentDirectory',fullfile(trainingDir,'checkpoints'));
            save(fullfile(trainingDir,'training_options.mat'),'trainOptions');
            stats = train(agent,env,trainOptions);
            episodes = numel(stats.EpisodeIndex);
            if episodes>0, steps=stats.TotalAgentSteps(end); end
        end
        finalRng = rng;
        after = localParameters(agent);
        sourceEnd = localSourceState(source);
        if training
            assert(isequal(sourceStart,sourceEnd), ...
                'train_energy_mission_rl:SourceChanged', ...
                'Source SHA, branch or working-tree status changed during training.');
        end
        report = struct('schema','energy_mission_training_report_v1','run_id',runId, ...
            'mode',options.mode,'source_sha',env.Native.Writer.Manifest.source_sha, ...
            'source_dirty',env.Native.Writer.Manifest.source_dirty, ...
            'source_branch',env.Native.Writer.Manifest.source_branch, ...
            'source_status',env.Native.Writer.Manifest.source_status, ...
            'source_start',sourceStart,'source_finish',sourceEnd, ...
            'source_unchanged',isequal(sourceStart,sourceEnd), ...
            'configuration_sha256',env.Native.Writer.Manifest.configuration_sha256, ...
            'seed',options.seed,'agent_specification',agentSpec, ...
            'episodes_trained',episodes,'training_decisions',steps, ...
            'captured_episodes',env.Native.Episode,'mpc_rows',env.Native.Writer.RowCount, ...
            'diagnostic_step_truncation',options.max_steps_per_episode<bundle.cfg.EP_STEPS, ...
            'truncation_semantics','Toolbox IsDone=2 preserves critic bootstrap; native terminal IsDone=1 stops it.', ...
            'actor_parameters_changed',~isequaln(before.actor,after.actor), ...
            'critic_parameters_changed',[~isequaln(before.critic1,after.critic1) ...
                ~isequaln(before.critic2,after.critic2)], ...
            'performance_tuned',false,'training_promoted',false, ...
            'wall_clock_seconds',toc(wallClock),'outcome',[options.mode '_complete']);
        save(fullfile(trainingDir,'final_agent.mat'),'agent','agentSpec','stats','finalRng','-v7.3');
        save(fullfile(trainingDir,'report.mat'),'report');
        localJson(fullfile(trainingDir,'report.json'),report);
        diary off
        close(env,report.outcome);
        report.validation = validate_exact_state_dataset(root);
        validation = report.validation;
        save(fullfile(trainingDir,'validation.mat'),'validation');
        localJson(fullfile(trainingDir,'validation.json'),validation);
        localArtifactIndex(root);
    catch exception
        diary off
        failure = struct('identifier',exception.identifier,'message',exception.message, ...
            'stack',exception.stack,'options',options,'rng_state',rng, ...
            'source_start',sourceStart,'source_at_failure',localSourceState(source), ...
            'wall_clock_seconds',toc(wallClock),'outcome','training_interface_exception');
        if ~isempty(env)
            if isempty(trainingDir)
                trainingDir=fullfile(env.Native.Writer.Root,'training');
                if ~isfolder(trainingDir), mkdir(trainingDir); end
            end
            if exist('agent','var')
                interruptedRng=rng;
                save(fullfile(trainingDir,'interrupted_agent.mat'), ...
                    'agent','agentSpec','interruptedRng','-v7.3');
            end
            save(fullfile(trainingDir,'failure.mat'),'failure');
            localJson(fullfile(trainingDir,'failure.json'),failure);
            close(env,'training_interface_exception');
            if ~isfile(fullfile(trainingDir,'artifact_sha256.csv'))
                localArtifactIndex(env.Native.Writer.Root);
            end
        end
        rethrow(exception)
    end
end

function state = localSourceState(source)
    state=struct('sha',localGit(source,{'rev-parse','HEAD'}), ...
        'branch',localGit(source,{'branch','--show-current'}), ...
        'status',localGit(source,{'status','--porcelain'}));
    state.dirty=~isempty(state.status);
end

function output = localGit(source,arguments)
    % ProcessBuilder passes each argument literally, without shell expansion.
    command=java.util.ArrayList();
    for value=[{'git','-C',source},arguments]
        command.add(java.lang.String(value{1}));
    end
    builder=java.lang.ProcessBuilder(command);
    builder.redirectErrorStream(true);
    process=builder.start();
    reader=java.io.BufferedReader(java.io.InputStreamReader(process.getInputStream(),'UTF-8'));
    lines=cell(0,1);
    while true
        line=reader.readLine();
        if isempty(line), break; end
        lines{end+1,1}=char(line); %#ok<AGROW>
    end
    output=strjoin(lines,newline);
    status=process.waitFor();
    assert(status==0,'train_energy_mission_rl:SourceProvenance', ...
        'Cannot read Git source provenance: %s',output);
end

function options = localOptions(options)
    defaults = struct('mode','build_only','setup_options',struct(),'seed',[], ...
        'max_episodes',1,'max_steps_per_episode',[],'agent_options',struct(), ...
        'environment_options',struct());
    assert(isstruct(options) && isscalar(options) && ...
        all(ismember(fieldnames(options),fieldnames(defaults))), ...
        'train_energy_mission_rl:Options','Unknown training option.');
    names=fieldnames(defaults);
    for k=1:numel(names)
        if ~isfield(options,names{k}), options.(names{k})=defaults.(names{k}); end
    end
    options.mode=validatestring(options.mode,{'build_only','train'});
    validateattributes(options.max_episodes,{'numeric'},{'scalar','integer','positive','finite'});
    if ~isempty(options.max_steps_per_episode)
        validateattributes(options.max_steps_per_episode,{'numeric'},{'scalar','integer','positive','finite'});
    end
    if ~isempty(options.seed)
        validateattributes(options.seed,{'numeric'},{'scalar','integer','nonnegative','finite','<=',2^32-1});
    end
    validateattributes(options.setup_options,{'struct'},{'scalar'});
    validateattributes(options.environment_options,{'struct'},{'scalar'});
end

function parameters = localParameters(agent)
    critics=getCritic(agent);
    parameters=struct('actor',{getLearnableParameters(getActor(agent))}, ...
        'critic1',{getLearnableParameters(critics(1))}, ...
        'critic2',{getLearnableParameters(critics(2))});
end

function localJson(path,value)
    fid=fopen(path,'w');
    assert(fid>=0,'train_energy_mission_rl:Write','Cannot write %s.',path);
    cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end

function localArtifactIndex(root)
    target=fullfile(root,'training','artifact_sha256.csv');
    assert(~isfile(target),'train_energy_mission_rl:IndexExists','An artifact index already exists.');
    files=dir(fullfile(root,'**','*'));
    rows=cell(0,3);
    for k=1:numel(files)
        if files(k).isdir, continue; end
        path=fullfile(files(k).folder,files(k).name);
        rows(end+1,:)={path(numel(root)+2:end),files(k).bytes,sha256_file(path)}; %#ok<AGROW>
    end
    writetable(cell2table(rows,'VariableNames',{'file','bytes','sha256'}),target);
end
