function report = run_energy_mission_timing_qualification(outputRoot)
%RUN_ENERGY_MISSION_TIMING_QUALIFICATION Captured default-timing diagnostic.
% Three deterministic normalized requests: speed up, up, down; unchanged R
% and minimum acceleration action. Uses the normal 320 m / 600 s mission,
% then closes after 15 s. No actor, learning, noise or mission qualification.
source = bootstrap_RF_MPC_RL();
sourceStart = localSource(source);
assert(~sourceStart.dirty,'energyTiming:DirtySource','Commit source before this diagnostic.');
assert(~isfolder(outputRoot) && ~isfile(outputRoot),'energyTiming:Exists','Use a new output directory.');
canonical = char(java.io.File(char(outputRoot)).getCanonicalPath());
assert(~strcmp(canonical,source) && ~startsWith(string(canonical),string(source)+filesep), ...
    'energyTiming:OutputInsideSource','Evidence must be outside the source checkout.');
bundle = setup_energy_mission_rl(); cfg = bundle.cfg; p = get_params(0);
decisionSeconds = cfg.CHUNK_DURATION*cfg.APPLY_EVERY;
assert(decisionSeconds==5 && cfg.MISSION_DURATION==600 && cfg.MISSION.D_TARGET_M==320, ...
    'energyTiming:Defaults','This diagnostic qualifies the default 5 s / 320 m / 600 s configuration.');
actions = [zeros(3,3);1 1 -1;-1 -1 -1];
oldRng = rng; rngCleanup = onCleanup(@()rng(oldRng)); %#ok<NASGU>
rng(cfg.RNG_SEED,'twister'); initialRng = rng;
[~,runId] = fileparts(canonical);
metadata = struct('run_id',runId,'run_type','energy_mission_default_timing_diagnostic_v1', ...
    'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',cfg.RNG_SEED, ...
    'source_policy','deterministic_normalized_speed_up_up_down','source_start',sourceStart, ...
    'training',false,'learning_rates_used',[],'exploration_noise_used',[], ...
    'normalized_action_schedule',actions,'planned_decisions',3, ...
    'kernel',struct('mpc_step_s',p.simTimeStep,'decision_interval_s',decisionSeconds, ...
        'gait_id',0,'architecture',computer('arch')), ...
    'scope','Bounded action-path diagnostic; no actor, learning, full-mission or low-SOC qualification.');
env = []; root = canonical; timer = tic;
try
    env = EnergyMissionTrainingEnvironment(bundle,root,metadata,struct());
    root = env.Native.Writer.Root;
    save(fullfile(root,'timing_request.mat'),'actions','initialRng','metadata');
    reset(env); initialState = env.Native.State;
    windows = cell(3,1); executedSpeeds = zeros(3,1); appliedActions = zeros(5,3);
    for k = 1:3
        before = env.Native.State;
        physical = energy_mission_action_transform(actions(:,k),env.Native.Config);
        [~,expected] = resolve_replay_action(physical,before,env.Native.Config, ...
            bundle.lower_abs,bundle.upper_abs);
        [observation,reward,done,info] = step(env,actions(:,k));
        assert(isequaln(expected,info.action_execution),'energyTiming:Action','Applied action differs from the governor.');
        assert(all(isfinite(observation)) && isfinite(reward),'energyTiming:Transition','Nonfinite transition.');
        assert(~done && isempty(info.terminal_reason),'energyTiming:EarlyTerminal', ...
            'The bounded timing probe terminated early: %s.',info.terminal_reason);
        assert(abs(info.window.duration_s-decisionSeconds)<1e-8, ...
            'energyTiming:Duration','A decision did not integrate the default interval.');
        windows{k} = info.window;
        executedSpeeds(k) = info.action_execution.v_exec;
        appliedActions(:,k) = info.action_execution.applied_action;
        fprintf('[timing] decision=%d t=%.2f speed=%.6f SOC=%.6f QPs=%d\n', ...
            k,env.Native.State.t,executedSpeeds(k),env.Native.State.energy.soc,info.window.qp_solve_count);
    end
    finalState = env.Native.State;
    assert(executedSpeeds(1)>initialState.supervisory_state.v_exec && ...
        executedSpeeds(2)>executedSpeeds(1) && executedSpeeds(3)<executedSpeeds(2), ...
        'energyTiming:Direction','Expected two speed increases followed by a decrease.');
    capacityAs = 3600*cfg.BATTERY.C_nom_Ah*cfg.BATTERY.n_parallel;
    chargeAs = finalState.energy.chargeAs-initialState.energy.chargeAs;
    chargeResidual = sum(cellfun(@(w)w.charge_As,windows))-chargeAs;
    socResidual = initialState.energy.soc-finalState.energy.soc-chargeAs/capacityAs;
    assert(abs(chargeResidual)<1e-8 && abs(socResidual)<1e-10, ...
        'energyTiming:Conservation','Window charge, physical charge and SOC do not reconcile.');
    assert(env.Native.Writer.RowCount==round(3*decisionSeconds/p.simTimeStep), ...
        'energyTiming:Rows','Unexpected native row count.');
    sourceFinish = localSource(source);
    assert(isequal(sourceStart,sourceFinish),'energyTiming:SourceChanged','Source changed during the diagnostic.');
    report = struct('schema','energy_mission_timing_diagnostic_v1','source_start',sourceStart, ...
        'source_finish',sourceFinish,'configuration_sha256',env.Native.Writer.Manifest.configuration_sha256, ...
        'seed',cfg.RNG_SEED,'decisions',3,'mpc_rows',env.Native.Writer.RowCount, ...
        'duration_s',finalState.t-initialState.t,'decision_durations_s',cellfun(@(w)w.duration_s,windows), ...
        'initial_speed_m_s',initialState.supervisory_state.v_exec,'executed_speeds_m_s',executedSpeeds, ...
        'normalized_actions',actions,'applied_actions',appliedActions, ...
        'distance_m',finalState.Xt(1)-initialState.Xt(1),'initial_soc',initialState.energy.soc, ...
        'final_soc',finalState.energy.soc,'charge_As',chargeAs, ...
        'charge_conservation_residual_As',chargeResidual,'soc_conservation_residual',socResidual, ...
        'qp_solves',sum(cellfun(@(w)w.qp_solve_count,windows)), ...
        'physical_terminal_reason','','outcome','diagnostic_decision_limit', ...
        'diagnostic_truncation',true,'mission_complete',false,'training',false, ...
        'learning_rates_used',[],'exploration_noise_used',[], ...
        'actor_evaluated',false,'training_promoted',false,'wall_clock_seconds_before_validation',toc(timer), ...
        'scope',metadata.scope);
    localJson(fullfile(root,'timing_result.json'),report);
    close(env,'timing_diagnostic_complete');
    validation = validate_exact_state_dataset(root,true);
    validation.action_mapping = localValidateActions(root,actions);
    assert(isequal(sourceStart,localSource(source)),'energyTiming:SourceChanged','Source changed during validation.');
    validation.wall_clock_seconds_total = toc(timer);
    localJson(fullfile(root,'timing_validation.json'),validation);
    localIndex(root);
    report.validation = validation;
catch exception
    failure = struct('identifier',exception.identifier,'message',exception.message, ...
        'stack',exception.stack,'source_start',sourceStart,'wall_clock_seconds',toc(timer), ...
        'outcome','timing_diagnostic_failed','training_promoted',false,'rng_state',rng);
    try
        failure.source_at_failure = localSource(source);
    catch sourceError
        failure.source_read_error = sourceError.message;
    end
    if ~isempty(env)
        failure.last_state = env.Native.State;
        failure.mpc_rows = env.Native.Writer.RowCount;
        try
            close(env,'timing_diagnostic_failed');
        catch closeError
            failure.close_error = struct('identifier',closeError.identifier,'message',closeError.message);
        end
    end
    if isfolder(root)
        save(fullfile(root,'timing_failure.mat'),'failure','-v7.3');
        localJson(fullfile(root,'timing_failure.json'),rmfieldIfPresent(failure,'last_state'));
        localIndex(root);
    end
    rethrow(exception)
end
end

function result = localValidateActions(root,actions)
saved = load(fullfile(root,'manifest','configuration.mat'),'cfg'); cfg = saved.cfg;
index = readtable(fullfile(root,'mpc_steps','index.csv'),'TextType','string');
for k = 1:size(actions,2)
    name = sprintf('episode_000001_decision_%06d.mat',k);
    saved = load(fullfile(root,'decisions',name),'event'); record = saved.event.record;
    saved = load(fullfile(root,'normalized_actions',name),'audit'); audit = saved.audit;
    [physical,mapping] = energy_mission_action_transform(actions(:,k),cfg);
    assert(isequal(audit.requested_normalized,actions(:,k)) && ...
        isequal(audit.bounded_normalized,mapping.bounded_normalized) && ...
        isequal(audit.physical_candidate,physical) && isequal(record.action_execution.candidate_action,physical), ...
        'energyTiming:SavedAction','Saved normalized/physical actions differ from the declared schedule.');
    segment = find(index.first_row<=record.first_mpc_row & index.last_row>=record.first_mpc_row);
    assert(isscalar(segment),'energyTiming:SavedAction','Missing first-row snapshot.');
    rows = read_exact_state_rows(fullfile(root,index.file(segment)));
    row = rows{record.first_mpc_row-index.first_row(segment)+1};
    saved = load(fullfile(root,row.snapshot_reference.file),'snapshot');
    [~,execution] = resolve_replay_action(physical,saved.snapshot.state,cfg,cfg.PHASE3.lower_R,cfg.PHASE3.upper_R);
    assert(isequaln(execution,record.action_execution),'energyTiming:SavedGovernor','Saved governor action does not reconstruct.');
end
result = struct('valid',true,'decisions_reconstructed',size(actions,2), ...
    'normalized_mapping_verified',true,'governor_mapping_verified',true,'actor_evaluated',false);
end

function state = localSource(source)
state = struct('sha',localGit(source,{'rev-parse','HEAD'}), ...
    'branch',localGit(source,{'branch','--show-current'}),'status',localGit(source,{'status','--porcelain'}));
state.dirty = ~isempty(state.status);
end

function output = localGit(source,args)
command = java.util.ArrayList();
for value = [{'git','-C',source},args], command.add(java.lang.String(value{1})); end
builder = java.lang.ProcessBuilder(command); builder.redirectErrorStream(true); process = builder.start();
reader = java.io.BufferedReader(java.io.InputStreamReader(process.getInputStream(),'UTF-8')); lines = cell(0,1);
while true
    line = reader.readLine(); if isempty(line), break; end
    lines{end+1,1} = char(line); %#ok<AGROW>
end
output = strjoin(lines,newline); status = process.waitFor();
assert(status==0,'energyTiming:Source','Cannot read source provenance: %s',output);
end

function localJson(path,value)
fid = fopen(path,'w'); assert(fid>=0,'energyTiming:Write','Cannot write diagnostic evidence.');
cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end

function value = rmfieldIfPresent(value,name)
if isfield(value,name), value = rmfield(value,name); end
end

function localIndex(root)
target = fullfile(root,'timing_artifact_sha256.csv');
assert(~isfile(target),'energyTiming:IndexExists','An artifact index already exists.');
files = dir(fullfile(root,'**','*')); rows = cell(0,3);
for k = 1:numel(files)
    if files(k).isdir, continue; end
    path = fullfile(files(k).folder,files(k).name);
    rows(end+1,:) = {path(numel(root)+2:end),files(k).bytes,sha256_file(path)}; %#ok<AGROW>
end
writetable(cell2table(rows,'VariableNames',{'file','bytes','sha256'}),target);
end
