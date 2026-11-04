function report=run_pack_sizing_demand(root)
% Collect one complete governed-fast requested mission, not feasible motion.
% The voltage screen is deliberately unenforced ONLY for demand sizing.
% Dynamics, limits, force solver, pack and energy equations are unchanged.
src=bootstrap_RF_MPC_RL();
assert(~isfolder(root) && ~isfile(root),'sizing:Exists','Use a new output root.');
sha=gitValue(src,'rev-parse HEAD');
assert(isempty(gitValue(src,'status --porcelain')),'sizing:Dirty','Commit source before acquisition.');
b=setup_RL_MPC('legacy_frozen_v1'); c=b.cfg; s=initialize_mpc_replay_state(c,0);
mkdir(root); save(fullfile(root,'configuration.mat'),'b','-v7.3');
meta=struct('schema','requested_pack_sizing_trace_v1','source_sha',sha, ...
 'branch',gitValue(src,'branch --show-current'),'dirty',false,'matlab',version, ...
 'timestamp_utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
 'seed',c.RNG_SEED,'reward_version',c.REWARD.version,'training',false, ...
 'learning_rates_used',[],'exploration_noise_used',[], ...
 'policy','fast_with_existing_governor_and_rate_limit_min_acceleration', ...
 'enforce_drive_limits',false,'achieved_motion_claim',false, ...
 'scope','One uncensored nominal requested trajectory; not an RL action-distribution or hardware qualification.', ...
 'configuration_sha256',sha256_file(fullfile(root,'configuration.mat')), ...
 'workbook_sha256',sha256_file(c.BATTERY.model_options.parameter_source), ...
 'waveform_sha256',sha256_file(c.PROXY.kneeCsv));
writeJson(root,'manifest',meta);
origin=s.Xt(1); target=origin+c.MISSION.D_TARGET_M;
drive=zeros(round(c.MISSION_DURATION/.01),53); n=0; chunks=0; solves=0; failed=0;
terminal='time_limit';
try
 while s.t<c.MISSION_DURATION-1e-8
  if mod(round(s.t*100),round(c.CHUNK_DURATION*c.APPLY_EVERY*100))==0
   control=resolve_replay_action([zeros(3,1);c.GAMMA_V_MAX;c.GAMMA_A_MIN], ...
       s,c,b.lower_abs,b.upper_abs);
   control.reference_mode='position_continuous_v1';
  end
  before=s;
  options=struct('duration_s',min(c.CHUNK_DURATION,c.MISSION_DURATION-s.t), ...
    'capture_trace',false,'capture_drive_trace',true,'update_proxy',false, ...
    'update_battery',true,'enforce_drive_limits',false,'stop_at_position_x_m',target, ...
    'solver_strategy','default_one_shot_fallback_tight_primal_v1');
  [s,out]=simulate_mpc_horizon(s,control,c,options);
  chunks=chunks+1; count=size(out.drive_trace,1);
  drive(n+(1:count),:)=out.drive_trace; n=n+count;
  solves=solves+out.qp_solve_count; failed=failed+out.qp_failed_count;
  after=s;
  save(fullfile(root,sprintf('chunk_%04d.mat',chunks)), ...
      'before','after','control','options','out','-v7.3');
  fprintf('Sizing requested trace: t=%.2f x=%.3f samples=%d terminal=%s\n', ...
      s.t,s.Xt(1)-origin,n,out.terminal_reason);
  if out.terminal_reason~="horizon_complete"
   terminal=char(out.terminal_reason); break
  end
 end
 drive=drive(1:n,:); columns=out.drive_trace_columns;
 save(fullfile(root,'demand.mat'),'drive','columns','-v7.3');
 report=struct('source_sha',sha,'dirty',false,'terminal',terminal, ...
   'duration_s',s.t,'distance_m',s.Xt(1)-origin,'samples',n,'qp_solves',solves, ...
   'qp_failed',failed,'target_m',c.MISSION.D_TARGET_M,'deadline_s',c.MISSION_DURATION, ...
   'voltage_limit_intervals',s.energy.voltageLimitSteps,'torque_limit_intervals',s.energy.torqueLimitSteps, ...
   'requested_trace_complete',strcmp(terminal,'mission_target_reached'), ...
   'achieved_motion_claim',false,'demand_sha256',sha256_file(fullfile(root,'demand.mat')));
 assert(strcmp(sha,gitValue(src,'rev-parse HEAD')) && isempty(gitValue(src,'status --porcelain')), ...
   'sizing:ChangedSource','Source changed during acquisition.');
 writeJson(root,'result',report);
catch exception
 save(fullfile(root,'interrupted_state.mat'),'s','n','chunks','-v7.3');
 writeJson(root,'failure',struct('identifier',exception.identifier,'message',exception.message,'source_sha',sha));
 rethrow(exception)
end
end
function value=gitValue(src,args)
[status,value]=system(sprintf('git -C "%s" %s',src,args)); assert(status==0); value=strtrim(value);
end
function writeJson(root,name,value)
f=fopen(fullfile(root,[name '.json']),'w'); assert(f>=0); closeFile=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(value,'PrettyPrint',true));
end
