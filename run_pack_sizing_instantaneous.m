function report=run_pack_sizing_instantaneous(inputRoot,outputRoot,options)
%RUN_PACK_SIZING_INSTANTANEOUS Reconstruct continuous requested drive demand.
% No new MPC/QP solves. Each stored zero-order-held ODE interval is reintegrated
% with its original tolerances, then evaluated on uniform midpoint grids.
% Midpoints carry equal duration weights. Start/end samples are additionally
% retained for peak checks; they do not receive artificial percentile weight.
% All maxima remain sampled lower bounds on continuous-time peaks.
if nargin<3,options=struct();end
options=defaults(options);
src=bootstrap_RF_MPC_RL();sha=gitValue(src,'rev-parse HEAD');
assert(isempty(gitValue(src,'status --porcelain')),'instantSizing:Dirty','Commit source before analysis.');
assert(~isfolder(outputRoot) && ~isfile(outputRoot),'instantSizing:Exists','Use a new output root.');
original=jsondecode(fileread(fullfile(inputRoot,'manifest.json')));
originalResult=jsondecode(fileread(fullfile(inputRoot,'result.json')));
assert(originalResult.requested_trace_complete && originalResult.qp_failed==0, ...
 'instantSizing:Input','Require the completed requested trace without failed QPs.');
assert(strcmp(sha256_file(fullfile(inputRoot,'configuration.mat')),original.configuration_sha256), ...
 'instantSizing:InputHash','Configuration hash differs.');
assert(~isempty(regexp(original.source_sha,'^[0-9a-f]{40}$','once')),'instantSizing:Source','Invalid input source SHA.');
[status,~]=system(sprintf('git -C "%s" diff --quiet %s HEAD -- fcns/get_params.m fcns/dynamics_SRB.m fcns/fcn_invKin3.m',src,original.source_sha));
assert(status==0,'instantSizing:PhysicsChanged','Reconstruction requires unchanged dynamics and IK.');
saved=load(fullfile(inputRoot,'configuration.mat'),'b');cfg=saved.b.cfg;
steps=min(originalResult.samples,options.max_steps);fine=options.substeps;coarse=options.comparison_substeps;
q=2*lcm(fine,coarse);fineIndices=1+(2*(1:fine)-1)*(q/(2*fine));
coarseIndices=1+(2*(1:coarse)-1)*(q/(2*coarse));
selected=unique([1,fineIndices,coarseIndices,q+1]);
p=get_params(0);dt=p.simTimeStep;
mkdir(outputRoot);copyfile(fullfile(inputRoot,'configuration.mat'),fullfile(outputRoot,'configuration.mat'));
meta=struct('schema','instantaneous_pack_sizing_v1','source_sha',sha, ...
 'source_branch',gitValue(src,'branch --show-current'),'source_dirty',false, ...
 'analysis_file_sha256',sha256_file([mfilename('fullpath') '.m']), ...
 'input_source_sha',original.source_sha,'input_manifest_sha256',sha256_file(fullfile(inputRoot,'manifest.json')), ...
 'input_result_sha256',sha256_file(fullfile(inputRoot,'result.json')), ...
 'input_configuration_sha256',original.configuration_sha256,'matlab',version, ...
 'timestamp_utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
 'seed',cfg.RNG_SEED,'reward_version',cfg.REWARD.version,'training',false, ...
 'learning_rates_used',[],'exploration_noise_used',[],'options',options, ...
 'requested_steps',steps,'midpoint_duration_s',dt/fine,'comparison_midpoint_duration_s',dt/coarse, ...
 'quantile_definition','Nearest-rank empirical quantile: sorted(abs(values))(ceil(.95*N)); equal duration midpoint samples, per joint.', ...
 'scope','Reinterpretation of stored requested trajectories; no new QP, achieved-motion or hardware claim.', ...
 'peak_scope','Maximum over the union of fine/coarse midpoints and both interval endpoints; not a continuous upper bound.');
writeJson(fullfile(outputRoot,'manifest.json'),meta);
file=fullfile(outputRoot,'instantaneous.mat');store=matfile(file,'Writable',true);
store.analysis_metadata=meta;
initializeStore(store,steps,fine,coarse);
chunks=dir(fullfile(inputRoot,'chunk_*.mat'));[~,order]=sort({chunks.name});chunks=chunks(order);
processed=0;inputHashes=cell(0,2);timer=tic;globalPeak=struct('voltage_V',0);
try
 for f=1:numel(chunks)
  if processed==steps,break;end
  path=fullfile(chunks(f).folder,chunks(f).name);data=load(path);
  inputHashes(end+1,:)={chunks(f).name,sha256_file(path)}; %#ok<AGROW>
  assert(data.before.gait==0 && (~isfield(data.options,'enable_repository_disturbance') || ...
      ~data.options.enable_repository_disturbance),'instantSizing:Domain','Only the saved nominal trot is qualified.');
  p=get_params(data.before.gait);p.p_ext=zeros(3,1);count=min(size(data.out.drive_inputs,1),steps-processed);
  fineTime=zeros(count*fine,1);fineW=fineTime;fineV=fineTime;fineMotorPeak=fineTime;
  fineTau=zeros(count*fine,12);fineI=fineTau;fineMotorV=fineTau;
  coarseW=zeros(count*coarse,1);coarseTau=zeros(count*coarse,12);coarseI=coarseTau;
  stepValues=zeros(count,8);boundaryW=zeros(count,2);boundaryV=boundaryW;boundaryTorque=boundaryW;
  for k=1:count
   in=data.out.drive_inputs(k,:);timesRecorded=data.out.drive_trace(k,1:2);t0=timesRecorded(1);
   assert(abs(diff(timesRecorded)-dt)<1e-9,'instantSizing:TimeGrid','Stored interval differs from MPC timestep.');
   x0=in(1:30)';x1=in(31:60)';U=in(61:72)';Xd=in(73:102)';
   fsm=struct('FSM',in(103:106)','Ta',in(107:110)','Tb',in(111:114)');
   fun=@(t,x)dynamics_SRB(t,x,U,Xd,zeros(3,1),p);
   % Use the exact original two-point tspan. Uniform outputs are dense
   % evaluations and do not change the solver's original adaptive stepping.
   solution=ode45(fun,[t0,t0+dt],x0);times=t0+(0:q)*dt/q;X=deval(solution,times)';
   replayError=max(abs(X(end,:)'-x1));
   assert(replayError<=1e-10*max(1,norm(x1,inf)), ...
     'instantSizing:Replay','Saved ODE endpoint does not reproduce to numerical precision.');
   w=zeros(q+1,1);v=w;motorPeak=w;tau=zeros(q+1,12);amps=tau;motorV=tau;
   for j=selected
    x=X(j,:)';m=robotenergy.instantaneousLoad(x,fun(times(j),x),U,p,times(j),fsm, ...
      data.before.knee_template,data.before.knee_parameters.Tmc,cfg.ENERGY);
    w(j)=m.watts;v(j)=m.requiredV;motorPeak(j)=m.peakTorque;
    tau(j,:)=m.jointTorque(:)';amps(j,:)=m.phaseCurrent(:)';motorV(j,:)=m.motorVoltage(:)';
    if m.requiredV>globalPeak.voltage_V
     axis=m.voltageAxis;leg=m.voltageLeg;
     globalPeak=struct('voltage_V',m.requiredV,'step',processed+k,'time_s',times(j), ...
       'leg',leg,'axis',axis,'joint_speed_rad_s',m.jointSpeed(axis,leg), ...
       'joint_torque_Nm',m.jointTorque(axis,leg),'phase_current_A',m.phaseCurrent(axis,leg), ...
       'motor_speed_rad_s',m.motorSpeed(axis,leg),'total_power_W',m.watts, ...
       'endpoint',j==1 || j==q+1);
    end
   end
   a=(k-1)*fine+(1:fine);b=(k-1)*coarse+(1:coarse);
   fineTime(a)=times(fineIndices);fineW(a)=w(fineIndices);fineV(a)=v(fineIndices);fineMotorPeak(a)=motorPeak(fineIndices);
   fineTau(a,:)=tau(fineIndices,:);fineI(a,:)=amps(fineIndices,:);fineMotorV(a,:)=motorV(fineIndices,:);
   coarseW(b)=w(coarseIndices);coarseTau(b,:)=tau(coarseIndices,:);coarseI(b,:)=amps(coarseIndices,:);
   [vPeak,at]=max(v);peakTime=times(at);
   stepValues(k,:)=[t0,mean(w(fineIndices)),mean(w(coarseIndices)),vPeak,max(w),max(motorPeak),peakTime,replayError];
   boundaryW(k,:)=w([1,end]);boundaryV(k,:)=v([1,end]);boundaryTorque(k,:)=motorPeak([1,end]);
  end
  idx=processed*fine+(1:count*fine);cmp=processed*coarse+(1:count*coarse);st=processed+(1:count);
  store.sample_time_s(idx,1)=fineTime;store.watts(idx,1)=fineW;store.required_voltage_V(idx,1)=fineV;
  store.motor_torque_peak_Nm(idx,1)=fineMotorPeak;store.joint_torque_Nm(idx,1:12)=fineTau;
  store.phase_current_A(idx,1:12)=fineI;store.motor_voltage_V(idx,1:12)=fineMotorV;
  store.comparison_watts(cmp,1)=coarseW;store.comparison_joint_torque_Nm(cmp,1:12)=coarseTau;
  store.comparison_phase_current_A(cmp,1:12)=coarseI;store.step_summary(st,1:8)=stepValues;
  store.endpoint_watts(st,1:2)=boundaryW;store.endpoint_required_voltage_V(st,1:2)=boundaryV;
  store.endpoint_motor_torque_peak_Nm(st,1:2)=boundaryTorque;
  processed=processed+count;
  fprintf('Instantaneous sizing: %d/%d steps, %d midpoint samples, %.1f wall seconds\n',processed,steps,processed*fine,toc(timer));
  writeJson(fullfile(outputRoot,'progress.json'),struct('processed_steps',processed,'requested_steps',steps,'complete',false));
 end
 assert(processed==steps,'instantSizing:MissingRows','Not all requested input intervals were present.');
 store.step_summary_columns={'time_before_s','mean_power_W','comparison_mean_power_W','sampled_peak_voltage_V', ...
    'sampled_peak_power_W','sampled_peak_motor_torque_Nm','sampled_peak_voltage_time_s','replay_endpoint_error'};
 store.joint_order='column-major 3 axes per leg; abduction,hip,knee; legs1..4';
 store.input_chunk_hashes=inputHashes;
 fineStats=statistics(store,'',steps*fine,dt/fine);coarseStats=statistics(store,'comparison_',steps*coarse,dt/coarse);
 summary=store.step_summary;
 report=struct('schema',meta.schema,'source_sha',sha,'source_dirty',false,'input_source_sha',original.source_sha, ...
   'processed_steps',processed,'input_steps',originalResult.samples,'full_trace_analyzed',processed==originalResult.samples, ...
   'duration_s',processed*dt,'fine',fineStats,'comparison',coarseStats, ...
   'energy_relative_grid_difference',abs(fineStats.energy_Wh-coarseStats.energy_Wh)/max(fineStats.energy_Wh,eps), ...
   'maximum_saved_endpoint_error',max(summary(:,8)), ...
   'sampled_peak_voltage',globalPeak,'sampled_peak_power_W',max(summary(:,5)), ...
   'sampled_peak_motor_torque_Nm',max(summary(:,6)),'wall_s',toc(timer), ...
   'sample_file','instantaneous.mat','sample_file_sha256',sha256_file(file), ...
   'input_chunk_hashes',{inputHashes},'training',false,'achieved_motion_claim',false, ...
   'limitations','Uniform quadrature convergence is numerical evidence, not a peak upper bound or complete operating-domain qualification. Existing empirical swing/motor assumptions remain.');
 assert(strcmp(sha,gitValue(src,'rev-parse HEAD')) && isempty(gitValue(src,'status --porcelain')), ...
    'instantSizing:ChangedSource','Source changed during analysis.');
 writeJson(fullfile(outputRoot,'result.json'),report);
 writeJson(fullfile(outputRoot,'progress.json'),struct('processed_steps',processed,'requested_steps',steps,'complete',true));
catch exception
 writeJson(fullfile(outputRoot,'failure.json'),struct('source_sha',sha,'processed_steps',processed, ...
     'identifier',exception.identifier,'message',exception.message,'stack',{exception.stack}));
 rethrow(exception)
end
end

function initializeStore(store,n,fine,coarse)
for name={'sample_time_s','watts','required_voltage_V','motor_torque_peak_Nm'}
 store.(name{1})(n*fine,1)=0;
end
for name={'joint_torque_Nm','phase_current_A','motor_voltage_V'}
 store.(name{1})(n*fine,12)=0;
end
store.comparison_watts(n*coarse,1)=0;
store.comparison_joint_torque_Nm(n*coarse,12)=0;store.comparison_phase_current_A(n*coarse,12)=0;
store.step_summary(n,8)=0;store.endpoint_watts(n,2)=0;
store.endpoint_required_voltage_V(n,2)=0;store.endpoint_motor_torque_peak_Nm(n,2)=0;
end
function result=statistics(store,prefix,n,dt)
w=store.([prefix 'watts']);
result=struct('samples',n,'sample_duration_s',dt,'energy_Wh',sum(w)*dt/3600, ...
 'power_mean_W',mean(w),'power_p95_W',nearest95(w),'joint_torque_abs_p95_Nm',zeros(1,12), ...
 'phase_current_abs_p95_A',zeros(1,12),'phase_current_rms_A',zeros(1,12));
for j=1:12
 tau=store.([prefix 'joint_torque_Nm'])(:,j);amps=store.([prefix 'phase_current_A'])(:,j);
 result.joint_torque_abs_p95_Nm(j)=nearest95(abs(tau));
 result.phase_current_abs_p95_A(j)=nearest95(abs(amps));
 result.phase_current_rms_A(j)=sqrt(mean(amps.^2));
end
end
function value=nearest95(values)
values=sort(values(:));value=values(ceil(.95*numel(values)));
end
function options=defaults(options)
default=struct('substeps',32,'comparison_substeps',16,'max_steps',Inf);
assert(all(ismember(fieldnames(options),fieldnames(default))),'instantSizing:Options','Unknown option.');
for field=fieldnames(default)'
 if ~isfield(options,field{1}),options.(field{1})=default.(field{1});end
end
validateattributes(options.substeps,{'numeric'},{'scalar','integer','positive','finite'});
validateattributes(options.comparison_substeps,{'numeric'},{'scalar','integer','positive','finite'});
assert(options.substeps==2*options.comparison_substeps,'instantSizing:Grids','Use nested-factor-two midpoint resolutions.');
assert(isscalar(options.max_steps) && options.max_steps>0 && (isinf(options.max_steps) || ...
  options.max_steps==fix(options.max_steps)),'instantSizing:Steps','max_steps must be positive integer or Inf.');
end
function value=gitValue(src,args)
[status,value]=system(sprintf('git -C "%s" %s',src,args));assert(status==0);value=strtrim(value);
end
function writeJson(path,value)
f=fopen(path,'w');assert(f>=0);closeFile=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(value,'PrettyPrint',true));
end
