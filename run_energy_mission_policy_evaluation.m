function report=run_energy_mission_policy_evaluation(input,outputRoot,options)
%RUN_ENERGY_MISSION_POLICY_EVALUATION Captured actor inference without exploration.
if nargin<3,options=struct();end
source=bootstrap_RF_MPC_RL();options=localOptions(options);sourceStart=localSource(source);
assert(~sourceStart.dirty,'energyPolicy:DirtySource','Commit source before captured evaluation.');
canonical=char(java.io.File(char(outputRoot)).getCanonicalPath());
assert(~strcmp(canonical,source) && ~startsWith(string(canonical),string(source)+filesep), ...
 'energyPolicy:OutputInsideSource','Evaluation evidence belongs outside the source checkout.');
checkpoint=load_energy_mission_checkpoint(input);original=checkpoint.bundle;bundle=original;
bundle=energy_mission_evaluation_bundle(original,options.scenario_overrides);
previousRng=rng;cleanup=onCleanup(@()rng(previousRng)); %#ok<NASGU>
rng(bundle.cfg.RNG_SEED,'twister');initialRng=rng;provenance=rmfield(checkpoint,{'actor','bundle'});
bundle.cfg.POLICY_EVALUATION=struct('version','energy_mission_actor_evaluation_v1','options',options, ...
 'checkpoint',provenance,'training',false,'inference_mode','actor_evaluate_no_exploration');
[~,runId]=fileparts(outputRoot);
metadata=struct('run_id',runId,'run_type','energy_mission_deterministic_actor_evaluation', ...
 'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',bundle.cfg.RNG_SEED, ...
 'source_policy','captured_normalized_td3_checkpoint','checkpoint',provenance,'source_start',sourceStart, ...
 'training',false,'learning_rates_used',[],'exploration_noise_used',[],'inference_mode','actor_evaluate_no_exploration');
env=[];timer=tic;
try
 environmentOptions=original.cfg.PHASE3.options;
 if isfield(environmentOptions,'reward_schema_if_present'),environmentOptions=rmfield(environmentOptions,'reward_schema_if_present');end
 env=EnergyMissionTrainingEnvironment(bundle,outputRoot,metadata,environmentOptions);
 root=env.Native.Writer.Root;mkdir(fullfile(root,'policy'));
 copyfile(checkpoint.path,fullfile(root,'policy','checkpoint.mat'));
 assert(strcmp(sha256_file(fullfile(root,'policy','checkpoint.mat')),checkpoint.sha256), ...
  'energyPolicy:CheckpointCopy','Copied checkpoint bytes differ.');
 copyfile(fullfile(checkpoint.root,'manifest','configuration.mat'),fullfile(root,'policy','training_configuration.mat'));
 save(fullfile(root,'policy','inference_start.mat'),'initialRng','provenance','options');
 observation=reset(env);outcome='probe_decision_limit';totalReward=0;
 while env.Native.Decision<options.max_decisions && ~env.Native.Done
  [value,nextState]=evaluate(checkpoint.actor,{observation});
  assert(isempty(nextState),'energyPolicy:RecurrentActor','Recurrent policy state is unsupported by this capture.');
  normalized=double(value{1}(:));validateattributes(normalized,{'double'},{'numel',5,'real','finite'});
  assert(all(abs(normalized)<=1+1e-12),'energyPolicy:ActorBounds','Actor must emit normalized bounded actions.');
  [observation,reward,done,info]=step(env,normalized);totalReward=totalReward+reward;
  fprintf('[energy actor] decision=%d t=%.2f distance=%.6f SOC=%.6f terminal=%s\n', ...
   env.Native.Decision,env.Native.State.t,info.window.distance_end_m,env.Native.State.energy.soc,info.terminal_reason);
  if done,outcome=info.terminal_reason;end
 end
 sourceFinish=localSource(source);
 assert(isequal(sourceStart,sourceFinish),'energyPolicy:SourceChanged','Evaluation source changed.');
 assert(strcmp(sha256_file(checkpoint.path),checkpoint.sha256),'energyPolicy:CheckpointChanged','Original checkpoint changed.');
 report=struct('schema','energy_mission_actor_evaluation_v1','source_start',sourceStart,'source_finish',sourceFinish, ...
  'source_sha',sourceStart.sha,'source_dirty',false,'checkpoint',provenance,'checkpoint_unchanged',true,'options',options, ...
  'configuration_sha256',env.Native.Writer.Manifest.configuration_sha256,'evaluation_seed',bundle.cfg.RNG_SEED, ...
  'inference_mode','actor_evaluate_no_exploration','training',false,'episodes_trained',0, ...
  'learning_rates_used',[],'exploration_noise_used',[],'outcome',outcome,'mission_complete',strcmp(outcome,'mission_complete'), ...
  'decisions',env.Native.Decision,'mpc_rows',env.Native.Writer.RowCount,'total_reward',totalReward,'wall_clock_seconds',toc(timer), ...
  'metrics',phase3_episode_metrics(root,env.Native.Config,env.Native.Episode),'performance_tuned',false,'training_promoted',false);
 localJson(fullfile(root,'evaluation.json'),report);close(env,outcome);
 report.validation=validate_exact_state_dataset(root,true);
 report.validation.action_reconstruction=validate_energy_mission_policy_actions(root,checkpoint.actor);
 assert(isequal(sourceStart,localSource(source)),'energyPolicy:SourceChanged','Source changed during validation.');
 assert(strcmp(sha256_file(checkpoint.path),checkpoint.sha256),'energyPolicy:CheckpointChanged','Checkpoint changed during validation.');
 localJson(fullfile(root,'evaluation_validation.json'),report.validation);
catch e
 if ~isempty(env)
  current=localFailureState(source,checkpoint.path);
  failure=struct('current_evidence',current,'identifier',e.identifier,'message',e.message,'stack',e.stack,'source_start',sourceStart,'checkpoint',provenance,'options',options);
  localJson(fullfile(env.Native.Writer.Root,'evaluation_failure.json'),failure);close(env,'energy_policy_evaluation_exception');
 end
 rethrow(e)
end
end
function options=localOptions(options)
d=struct('max_decisions',inf,'scenario_id','training_scenario','scenario_overrides',struct());
assert(isstruct(options) && isscalar(options) && all(ismember(fieldnames(options),fieldnames(d))), ...
 'energyPolicy:Options','Unknown evaluation option.');explicitId=isfield(options,'scenario_id');
for n=fieldnames(d).',if ~isfield(options,n{1}),options.(n{1})=d.(n{1});end,end
validateattributes(options.max_decisions,{'double'},{'scalar','real','positive'});
assert(isinf(options.max_decisions) || options.max_decisions==floor(options.max_decisions), ...
 'energyPolicy:Options','Decision limit must be a positive integer or Inf.');
assert(isstruct(options.scenario_overrides) && isscalar(options.scenario_overrides) && ...
 all(ismember(fieldnames(options.scenario_overrides),{'initial_soc','target_distance_m','mission_duration_s','seed'})), ...
 'energyPolicy:ScenarioOverrides','Only initial SOC, distance, duration and seed may be explicitly overridden.');
options.scenario_id=char(options.scenario_id);
assert(~isempty(regexp(options.scenario_id,'^[A-Za-z0-9][A-Za-z0-9_.-]*$','once')), ...
 'energyPolicy:ScenarioId','Use an explicit ASCII scenario identifier.');
assert(isempty(fieldnames(options.scenario_overrides)) || explicitId,'energyPolicy:ScenarioId','Overrides require an explicit scenario_id.');
end
function s=localSource(source)
s=struct('sha',localGit(source,{'rev-parse','HEAD'}),'branch',localGit(source,{'branch','--show-current'}), ...
 'status',localGit(source,{'status','--porcelain'}));s.dirty=~isempty(s.status);
end
function output=localGit(source,args)
command=java.util.ArrayList();for v=[{'git','-C',source},args],command.add(java.lang.String(v{1}));end
builder=java.lang.ProcessBuilder(command);builder.redirectErrorStream(true);process=builder.start();
reader=java.io.BufferedReader(java.io.InputStreamReader(process.getInputStream(),'UTF-8'));lines=cell(0,1);
while true,line=reader.readLine();if isempty(line),break;end;lines{end+1,1}=char(line);end %#ok<AGROW>
output=strjoin(lines,newline);status=process.waitFor();assert(status==0,'energyPolicy:SourceProvenance','Cannot read source provenance: %s',output);
end
function localJson(path,value)
fid=fopen(path,'w');assert(fid>=0,'energyPolicy:Write','Cannot write evaluation evidence.');
c=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true)); %#ok<NASGU>
end

function state=localFailureState(source,path)
state=struct();
try,state.source=localSource(source);catch e,state.source_error=e.message;end
try,state.checkpoint_sha256=sha256_file(path);catch e,state.checkpoint_error=e.message;end
end
