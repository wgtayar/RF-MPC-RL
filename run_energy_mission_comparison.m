function report=run_energy_mission_comparison(outputRoot,options)
%RUN_ENERGY_MISSION_COMPARISON Matched native fixed-policy Pareto diagnostics.
% Short missions validate the new interface/action sensitivity. They do not
% establish full 320 m performance, learned value, or depleted-SOC behavior.
if nargin<2,options=struct();end
defaults=struct('setup_options',struct('target_distance_m',20,'mission_duration_s',50, ...
    'decision_interval_s',5),'speeds_m_s',[.45 .5 .66],'initial_soc',[.95 1]);
assert(all(ismember(fieldnames(options),fieldnames(defaults))),'energyComparison:Options','Unknown option.');
for f=fieldnames(defaults).'
 if ~isfield(options,f{1}),options.(f{1})=defaults.(f{1});end
end
validateattributes(options.speeds_m_s,{'double'},{'vector','real','finite','positive'});
assert(~isfolder(outputRoot) && ~isfile(outputRoot),'energyComparison:Exists','Use a new evidence directory.');
src=bootstrap_RF_MPC_RL(); sha=gitValue(src,'rev-parse HEAD');
assert(isempty(gitValue(src,'status --porcelain')),'energyComparison:Dirty','Commit source before qualification.');
mkdir(outputRoot); oldRng=rng; cleanup=onCleanup(@()rng(oldRng));
meta=struct('schema','native_energy_mission_comparison_v1','source_sha',sha, ...
 'source_branch',gitValue(src,'branch --show-current'),'source_dirty',false, ...
 'timestamp_utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
 'options',options,'training',false,'learning_rates_used',[],'exploration_noise_used',[]);
writeJson(outputRoot,'manifest',meta); results=cell(0,1);
try
 for z=options.initial_soc
  for v=options.speeds_m_s
   setup=options.setup_options;setup.initial_soc=z;b=setup_energy_mission_rl(setup);c=b.cfg;
   gamma=(v-c.V_MIN)/(c.V_MAX-c.V_MIN);
   assert(gamma>=c.GAMMA_V_MIN && gamma<=c.GAMMA_V_MAX+1e-12,'energyComparison:Speed','Requested policy exceeds action bounds.');
   gamma=min(gamma,c.GAMMA_V_MAX);
   name=sprintf('soc%03d_speed%03d',round(100*z),round(100*v));folder=fullfile(outputRoot,name);
   m=struct('run_id',name,'run_type','matched_native_fixed_policy_comparison', ...
    'monitor_root',fullfile(fileparts(src),'RL-MPC-Monitor'),'seed',c.RNG_SEED, ...
    'source_policy',name,'training',false,'learning_rates_used',[],'exploration_noise_used',[]);
   env=Phase3MpcEnvironment(b,folder,m);totalReward=0;sumCharge=0;sumDuration=0;decisions=struct([]);
   try
    reset(env);
    while ~env.Done
     [o,r,~,info]=step(env,[zeros(3,1);gamma;c.GAMMA_A_MIN]);
     assert(all(isfinite(o)) && isfinite(r));
     totalReward=totalReward+r;sumCharge=sumCharge+info.window.charge_As;sumDuration=sumDuration+info.window.duration_s;
     d=struct('time_s',env.State.t,'distance_m',info.window.distance_end_m, ...
      'charge_As',env.State.energy.chargeAs,'current_A',info.window.Ieq_window, ...
      'true_soc',env.State.energy.soc,'estimated_soc',env.State.energy.estimate(1), ...
      'v_applied',info.action_execution.v_exec,'reward',r,'charge_cost',info.reward_info.charge_cost, ...
      'time_cost',info.reward_info.time_cost,'terminal',info.terminal_reason);
     if isempty(decisions),decisions=d;else,decisions(end+1)=d;end %#ok<AGROW>
     fprintf('[%s] t=%.2f x=%.4f Q=%.6f SOC=%.6f terminal=%s\n',name,d.time_s,d.distance_m,d.charge_As,d.true_soc,d.terminal);
    end
    q=env.State.energy.chargeAs;dt=env.State.t;sc=energy_mission_scales(c);
    expected=min(max(info.window.distance_end_m/c.MISSION.D_TARGET_M,0),1)- ...
      sc.time_weight*dt/sc.mission_duration_s-sc.charge_weight*q/sc.reference_charge_As;
    if strcmp(info.terminal_reason,'mission_complete'),expected=expected+sc.completion_bonus;else,expected=expected-1;end
    assert(abs(sumCharge-q)<1e-8 && abs(sumDuration-dt)<1e-8 && abs(totalReward-expected)<1e-8, ...
      'energyComparison:Accounting','Window charge/time/reward must telescope to the mission totals.');
    assert(abs((z-env.State.energy.soc)-q/sc.pack_capacity_As)<1e-9, ...
      'energyComparison:Conservation','True SOC must match actual committed Coulombs.');
    result=struct('name',name,'initial_soc',z,'requested_speed_m_s',v, ...
     'source_sha',sha,'configuration_sha256',env.Writer.Manifest.configuration_sha256, ...
     'seed',c.RNG_SEED,'reward_version',c.REWARD.version,'outcome',info.terminal_reason, ...
     'mission_complete',strcmp(info.terminal_reason,'mission_complete'),'time_s',dt, ...
     'distance_m',info.window.distance_end_m,'charge_As',q,'energy_Wh',env.State.energy.energyWh, ...
     'mean_pack_current_A',q/max(dt,eps),'final_true_soc',env.State.energy.soc, ...
     'final_estimated_soc',env.State.energy.estimate(1),'total_reward',totalReward, ...
     'reward_accounting_error',totalReward-expected,'pack_id',env.State.energy.packId, ...
     'metrics',phase3_episode_metrics(folder,env.Config,env.Episode),'decisions',decisions);
    close(env,'matched_comparison_complete');
    result.dataset_validated=validate_exact_state_dataset(folder,true).valid;
    writeJson(folder,'result',result);results{end+1}=result; %#ok<AGROW>
   catch e
    if ~env.Writer.Closed,close(env,'comparison_exception');end
    rethrow(e)
   end
  end
 end
 % Pareto comparison is within each identical initial-SOC scenario and only
 % among successful missions. No scalar weight or winning design is selected.
 for k=1:numel(results)
  a=results{k};dominated=false;
  if a.mission_complete
   for j=1:numel(results)
    b=results{j};
    dominated=dominated || (b.initial_soc==a.initial_soc && b.mission_complete && ...
     b.time_s<=a.time_s && b.charge_As<=a.charge_As && (b.time_s<a.time_s || b.charge_As<a.charge_As));
   end
  end
  results{k}.pareto_nondominated=a.mission_complete && ~dominated;
 end
 assert(strcmp(sha,gitValue(src,'rev-parse HEAD')) && isempty(gitValue(src,'status --porcelain')), ...
  'energyComparison:SourceChanged','Source changed during comparison.');
 report=struct('schema',meta.schema,'source_sha',sha,'cases',{results}, ...
  'performance_tuned',false,'learned_policy_tested',false,'pack_changed',false, ...
  'scope','Matched short native missions, same dynamics, cell and initial-SOC cases; Pareto time versus actual total charge. No full mission or reserve-driven adaptation claim.');
 writeJson(outputRoot,'result',report);
catch e
 writeJson(outputRoot,'failure',struct('identifier',e.identifier,'message',e.message,'stack',{e.stack}));
 rethrow(e)
end
end
function v=gitValue(root,args)
[s,v]=system(sprintf('git -C "%s" %s',root,args));assert(s==0);v=strtrim(v);
end
function writeJson(root,name,value)
f=fopen(fullfile(root,[name '.json']),'w');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(value,'PrettyPrint',true));
end
