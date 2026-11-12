function report = run_energy_mission_continuous_depletion(policy,outputRoot,options)
%RUN_ENERGY_MISSION_CONTINUOUS_DEPLETION Back-to-back missions on one battery.
% Starts at a fixed SOC (default 95%) and runs the configured 320 m / 600 s
% mission repeatedly. Each new mission starts at the previous mission's end
% position reset, but keeps the true SOC, cell RC state and estimator state
% (Phase3MpcEnvironment reset with a carried energy state). Stops at the
% first mission that does not complete (reserve, voltage or other terminal)
% or after max_missions.
%
% policy: a checkpoint input accepted by load_energy_mission_checkpoint
%         (deterministic actor, no exploration), or
%         struct('fixed_speed_m_s',v) for a fixed-speed baseline.
if nargin<3,options=struct();end
d=struct('initial_soc',.95,'max_missions',12,'run_id','');
assert(isstruct(options) && isscalar(options) && all(ismember(fieldnames(options),fieldnames(d))), ...
 'energyDepletion:Options','Unknown option.');
for n=fieldnames(d).',if ~isfield(options,n{1}),options.(n{1})=d.(n{1});end,end
validateattributes(options.max_missions,{'double'},{'scalar','integer','positive'});
source=bootstrap_RF_MPC_RL();sourceStart=localSource(source);
assert(~sourceStart.dirty,'energyDepletion:DirtySource','Commit source before captured evaluation.');
canonical=char(java.io.File(char(outputRoot)).getCanonicalPath());
assert(~startsWith(string(canonical),string(source)+filesep),'energyDepletion:OutputInsideSource', ...
 'Evidence belongs outside the source checkout.');
fixedSpeed=isstruct(policy) && isfield(policy,'fixed_speed_m_s');
if fixedSpeed
 bundle=setup_energy_mission_rl(struct('initial_soc',options.initial_soc,'initial_soc_mode','fixed'));
 c=bundle.cfg;gamma=(policy.fixed_speed_m_s-c.V_MIN)/(c.V_MAX-c.V_MIN);
 assert(gamma>=c.GAMMA_V_MIN && gamma<=c.GAMMA_V_MAX+1e-12,'energyDepletion:Speed','Speed outside action bounds.');
 gamma=min(gamma,c.GAMMA_V_MAX);provenance=struct('fixed_speed_m_s',policy.fixed_speed_m_s);
 sourcePolicy=sprintf('fixed_speed_%0.3f',policy.fixed_speed_m_s);
else
 checkpoint=load_energy_mission_checkpoint(policy);
 bundle=energy_mission_evaluation_bundle(checkpoint.bundle,struct('initial_soc',options.initial_soc));
 provenance=rmfield(checkpoint,{'actor','bundle'});sourcePolicy='captured_normalized_td3_checkpoint';
end
bundle.cfg.CONTINUOUS_DEPLETION=struct('version','continuous_depletion_v1','options',options, ...
 'carried_state','true SOC, cell RC voltages, estimator mean and covariance', ...
 'scope','Repeated identical missions on one battery; evaluation only, no learning.');
previousRng=rng;cleanup=onCleanup(@()rng(previousRng)); %#ok<NASGU>
rng(bundle.cfg.RNG_SEED,'twister');
[~,runId]=fileparts(outputRoot);if ~isempty(options.run_id),runId=options.run_id;end
metadata=struct('run_id',runId,'run_type','energy_mission_continuous_depletion', ...
 'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',bundle.cfg.RNG_SEED, ...
 'source_policy',sourcePolicy,'policy_provenance',provenance,'source_start',sourceStart, ...
 'training',false,'learning_rates_used',[],'exploration_noise_used',[]);
if fixedSpeed
 env=Phase3MpcEnvironment(bundle,outputRoot,metadata);native=env;
else
 o=checkpoint.bundle.cfg.PHASE3.options;
 if isfield(o,'reward_schema_if_present'),o=rmfield(o,'reward_schema_if_present');end
 env=EnergyMissionTrainingEnvironment(bundle,outputRoot,metadata,o);native=env.Native;
end
rows=cell(0,11);carried=[];timer=tic;finalOutcome='max_missions_reached';
try
 for mission=1:options.max_missions
  observation=reset(env,carried);startSoc=native.State.energy.soc;totalReward=0;outcome='';
  while ~native.Done
   if fixedSpeed
    action=[zeros(3,1);gamma;bundle.cfg.GAMMA_A_MIN];
   else
    [value,nextState]=evaluate(checkpoint.actor,{observation});
    assert(isempty(nextState),'energyDepletion:RecurrentActor','Recurrent actors are unsupported.');
    action=double(value{1}(:));
   end
   [observation,reward,done,info]=step(env,action);totalReward=totalReward+reward;
   if done,outcome=info.terminal_reason;end
  end
  e=native.State.energy;
  rows(end+1,:)={mission,startSoc,e.soc,e.estimate(1),native.State.t,info.window.distance_end_m, ...
   e.chargeAs,e.energyWh,totalReward,string(outcome),native.Decision}; %#ok<AGROW>
  fprintf('[depletion] mission=%d start=%.4f end=%.4f t=%.1f outcome=%s\n',mission,startSoc,e.soc,native.State.t,outcome);
  if ~strcmp(outcome,'mission_complete'),finalOutcome=outcome;break,end
  carried=struct('soc',e.soc,'rc',e.rc,'estimate',e.estimate,'cov',e.cov);
 end
 T=cell2table(rows,'VariableNames',{'mission','start_soc','end_soc','end_soc_estimate','duration_s', ...
  'distance_m','charge_As','energy_Wh','return','outcome','decisions'});
 root=native.Writer.Root;writetable(T,fullfile(root,'missions.csv'));
 report=struct('schema','energy_mission_continuous_depletion_v1','source_start',sourceStart, ...
  'policy',sourcePolicy,'policy_provenance',provenance,'options',options,'missions_completed',sum(T.outcome=="mission_complete"), ...
  'final_outcome',finalOutcome,'final_true_soc',native.State.energy.soc,'wall_clock_seconds',toc(timer), ...
  'training',false,'training_promoted',false);
 fid=fopen(fullfile(root,'continuous_depletion.json'),'w');fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
 report.missions=T;
 close(env,char(finalOutcome));
 report.validation=validate_exact_state_dataset(root,true);
 assert(isequal(sourceStart,localSource(source)),'energyDepletion:SourceChanged','Source changed during evaluation.');
catch e
 try,close(env,'continuous_depletion_exception');catch,end
 rethrow(e)
end
end

function s=localSource(source)
[st,sha]=system(sprintf('git -C "%s" rev-parse HEAD',source));assert(st==0);
[~,br]=system(sprintf('git -C "%s" branch --show-current',source));
[~,status]=system(sprintf('git -C "%s" status --porcelain',source));
s=struct('sha',strtrim(sha),'branch',strtrim(br),'status',strtrim(status));s.dirty=~isempty(s.status);
end
