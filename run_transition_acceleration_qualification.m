function report = run_transition_acceleration_qualification(outputRoot)
%RUN_TRANSITION_ACCELERATION_QUALIFICATION Matched native post-startup probes.
% Five-second startup prefix, then six separately captured five-second forks.
% Fixed 320 m / 600 s mission, 9S pack, 95% SOC; no learning or added noise.
source=bootstrap_RF_MPC_RL(); sourceStart=localSource(source);
assert(~sourceStart.dirty,'accelProbe:DirtySource','Commit source before qualification.');
root=char(java.io.File(char(outputRoot)).getCanonicalPath());
assert(~isfolder(root) && ~isfile(root),'accelProbe:Exists','Use a fresh output root.');
assert(~strcmp(root,source) && ~startsWith(string(root),string(source)+filesep), ...
    'accelProbe:SourceOutput','Keep evidence outside the source checkout.');
bundle=setup_energy_mission_rl(); cfg=bundle.cfg; p=get_params(0);
assert(strcmp(cfg.REFERENCE.mode,'velocity_continuous_v2') && ...
    strcmp(cfg.ACCELERATION.action_model,'bounded_transition_v1'), ...
    'accelProbe:Version','This diagnostic requires the new transition contract.');
assert(cfg.CHUNK_DURATION*cfg.APPLY_EVERY==5 && cfg.MISSION_DURATION==600 && ...
    cfg.MISSION.D_TARGET_M==320 && cfg.BATTERY.SOC_init==.95 && ...
    cfg.BATTERY.n_series==9 && cfg.BATTERY.n_parallel==1, ...
    'accelProbe:Defaults','Require the fixed default mission, timing and pack.');
gamma=(cfg.V_REQ_FIXED-cfg.V_MIN)/(cfg.V_MAX-cfg.V_MIN);
holdNorm=2*(gamma-cfg.GAMMA_V_MIN)/(cfg.GAMMA_V_MAX-cfg.GAMMA_V_MIN)-1;
prefixAction=[zeros(3,1);holdNorm;1];
names={'hold','up_slow','up_fast','down_slow','down_fast','up_fast_repeat'};
actions=[zeros(3,6);holdNorm 1 1 -1 -1 1;1 -1 1 -1 1 1];
oldRng=rng; rngCleanup=onCleanup(@()rng(oldRng));
rng(cfg.RNG_SEED,'twister'); initialRng=rng; mkdir(root); timer=tic; env=[];
metadata=struct('run_type','matched_transition_acceleration_v1', ...
    'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',cfg.RNG_SEED, ...
    'source_start',sourceStart,'source_policy','deterministic_normalized_matched_forks', ...
    'training',false,'learning_rates_used',[],'exploration_noise_used',[], ...
    'kernel',struct('mpc_step_s',p.simTimeStep,'decision_interval_s',5,'gait_id',0), ...
    'scope','Bounded native action-path evidence only; no full mission, low-SOC, R-authority or trained-policy qualification.');
save(fullfile(root,'request.mat'),'bundle','metadata','initialRng','prefixAction','names','actions');
try
    prefix=fullfile(root,'prefix'); metadata.run_id='prefix';
    env=EnergyMissionTrainingEnvironment(bundle,prefix,metadata,struct());
    reset(env); prefixBefore=env.Native.State;
    prefixInfo=localStep(env,prefixAction,bundle); prefixState=env.Native.State;
    localConservation(prefixBefore,prefixState,prefixInfo.window,cfg);
    assert(abs(prefixState.t-5)<1e-10,'accelProbe:Prefix','Prefix must end after startup.');
    assert(abs(prefixInfo.action_execution.v_exec-cfg.V_REQ_FIXED)<1e-12, ...
        'accelProbe:Prefix','Prefix must keep the unchanged initial speed target.');
    close(env,'acceleration_prefix_complete'); env=[];
    prefixValidation=validate_exact_state_dataset(prefix,true);
    outcomes=cell(6,1); traces=cell(6,1); validations=cell(6,1); states=cell(6,1);
    rows=cell(6,12);
    for k=1:6
        branch=fullfile(root,names{k}); metadata.run_id=names{k}; metadata.source_dataset=prefix;
        metadata.normalized_action=actions(:,k);
        env=EnergyMissionTrainingEnvironment(bundle,branch,metadata,struct());
        resumeFromDecision(env.Native,prefix,1,1); before=env.Native.State;
        assert(isequaln(compact_exact_mpc_state(before),compact_exact_mpc_state(prefixState)) && ...
            isequaln(before.current_time,prefixState.current_time) && ...
            isequaln(before.current_total,prefixState.current_total), ...
            'accelProbe:Start','Matched branches did not restore the identical dynamic state.');
        info=localStep(env,actions(:,k),bundle); after=env.Native.State;
        residual=localConservation(before,after,info.window,cfg);
        assert(env.Native.Writer.RowCount==round(5/p.simTimeStep), ...
            'accelProbe:Rows','Expected 500 committed native steps per branch.');
        states{k}=after; outcomes{k}=info;
        close(env,'acceleration_branch_complete'); env=[];
        validations{k}=validate_exact_state_dataset(branch,true);
        traces{k}=localTrace(branch);
        writetable(traces{k},fullfile(root,[names{k} '_trace.csv']));
        rows(k,:)={names{k},info.action_execution.v_exec,info.action_execution.a_exec, ...
            info.window.duration_s,info.window.window_distance_m,info.window.charge_As, ...
            info.window.energy_Wh,info.window.Ieq_window,max(traces{k}.mean_pack_current_A), ...
            after.Xt(4),after.energy.soc,residual};
        fprintf('[acceleration] %s v=%.6f a=%.3f charge=%.9f As energy=%.9f Wh\n', ...
            names{k},rows{k,2},rows{k,3},rows{k,6},rows{k,7});
    end
    summary=cell2table(rows,'VariableNames',{'case_name','speed_target_m_s','acceleration_m_s2', ...
        'duration_s','distance_m','charge_As','energy_Wh','mean_current_A', ...
        'peak_interval_mean_current_A','final_forward_speed_m_s','final_soc','soc_residual'});
    writetable(summary,fullfile(root,'summary.csv'));
    comparisons=struct();
    for pair=[2 4;3 5]
        a=pair(1); b=pair(2); left=outcomes{a}; right=outcomes{b};
        assert(abs(left.action_execution.v_exec-right.action_execution.v_exec)<1e-12 && ...
            abs(left.action_execution.a_exec-.2)<1e-12 && abs(right.action_execution.a_exec-.5)<1e-12, ...
            'accelProbe:MatchedControl','Expected identical target and 0.2/0.5 acceleration.');
        delta=struct('desired_speed_max_abs_m_s',max(abs(traces{a}.desired_speed_start_m_s-traces{b}.desired_speed_start_m_s)), ...
            'body_speed_max_abs_m_s',max(abs(traces{a}.actual_speed_end_m_s-traces{b}.actual_speed_end_m_s)), ...
            'current_max_abs_A',max(abs(traces{a}.mean_pack_current_A-traces{b}.mean_pack_current_A)), ...
            'energy_difference_Wh',left.window.energy_Wh-right.window.energy_Wh);
        assert(all(abs([delta.desired_speed_max_abs_m_s,delta.body_speed_max_abs_m_s, ...
            delta.current_max_abs_A,delta.energy_difference_Wh])>1e-10), ...
            'accelProbe:NoEffect','Acceleration did not measurably affect reference, motion, current and energy.');
        comparisons.(names{a})=delta;
    end
    assert(summary.speed_target_m_s(2)>summary.speed_target_m_s(1) && ...
        summary.speed_target_m_s(4)<summary.speed_target_m_s(1), ...
        'accelProbe:Velocity','Expected a hold, increase and decrease after startup.');
    assert(max(abs(traces{2}.actual_speed_end_m_s-traces{1}.actual_speed_end_m_s))>1e-10 && ...
        max(abs(traces{4}.actual_speed_end_m_s-traces{1}.actual_speed_end_m_s))>1e-10, ...
        'accelProbe:Velocity','Post-startup velocity commands did not affect the body.');
    repeatExact=isequaln(outcomes{3}.window,outcomes{6}.window) && ...
        isequaln(outcomes{3}.action_execution,outcomes{6}.action_execution) && ...
        isequaln(compact_exact_mpc_state(states{3}),compact_exact_mpc_state(states{6})) && ...
        isequaln(outcomes{3}.next_observation,outcomes{6}.next_observation) && ...
        isequaln(outcomes{3}.reward,outcomes{6}.reward) && isequaln(traces{3},traces{6});
    assert(repeatExact,'accelProbe:Repeat','Identical restored action did not reproduce the captured trajectory.');
    sourceFinish=localSource(source);
    assert(isequal(sourceStart,sourceFinish),'accelProbe:SourceChanged','Source changed during qualification.');
    report=struct('schema','matched_transition_acceleration_v1','passed',true, ...
        'source_start',sourceStart,'source_finish',sourceFinish,'seed',cfg.RNG_SEED, ...
        'matched_start_exact',true,'repeated_branch_exact',repeatExact, ...
        'prefix_validation',prefixValidation,'branch_validations',{validations}, ...
        'comparisons',comparisons,'native_rows_total',round(35/p.simTimeStep), ...
        'training',false,'training_promoted',false,'learning_rates_used',[], ...
        'exploration_noise_used',[],'scope',metadata.scope,'wall_clock_seconds',toc(timer));
    save(fullfile(root,'result.mat'),'report','summary','outcomes','states','-v7.3');
    localJson(fullfile(root,'result.json'),report); localIndex(root);
catch exception
    failure=struct('identifier',exception.identifier,'message',exception.message, ...
        'stack',exception.stack,'source_start',sourceStart,'wall_clock_seconds',toc(timer),'rng_state',rng);
    if ~isempty(env)
        failure.last_state=env.Native.State;
        try; close(env,'acceleration_probe_failed'); catch closeError; failure.close_error=closeError.message; end
    end
    save(fullfile(root,'failure.mat'),'failure','-v7.3');
    if isfield(failure,'last_state'),failure=rmfield(failure,'last_state');end
    localJson(fullfile(root,'failure.json'),failure); localIndex(root); rethrow(exception)
end
end

function info=localStep(env,action,bundle)
before=env.Native.State; physical=energy_mission_action_transform(action,env.Native.Config);
[~,expected]=resolve_replay_action(physical,before,env.Native.Config,bundle.lower_abs,bundle.upper_abs);
[observation,reward,done,info]=step(env,action);
assert(isequaln(expected,info.action_execution),'accelProbe:Action','Action mapping mismatch.');
assert(all(isfinite(observation)) && isfinite(reward) && ~done && isempty(info.terminal_reason), ...
    'accelProbe:Transition','Invalid or early terminal transition: %s.',info.terminal_reason);
assert(abs(info.window.duration_s-5)<1e-8,'accelProbe:Duration','Expected five-second window.');
end

function residual=localConservation(before,after,window,cfg)
charge=after.energy.chargeAs-before.energy.chargeAs;
residual=before.energy.soc-after.energy.soc-charge/(3600*cfg.BATTERY.C_nom_Ah*cfg.BATTERY.n_parallel);
assert(abs(window.charge_As-charge)<1e-8 && abs(residual)<1e-10 && ...
    abs(window.energy_Wh-(after.energy.energyWh-before.energy.energyWh))<1e-10, ...
    'accelProbe:Conservation','Physical charge, SOC or energy failed reconciliation.');
end

function trace=localTrace(root)
index=readtable(fullfile(root,'mpc_steps','index.csv'),'TextType','string'); data=zeros(0,10);
for k=1:height(index)
    rows=read_exact_state_rows(fullfile(root,index.file(k)));
    for j=1:numel(rows)
        r=rows{j}; assert(r.current_sample_committed,'accelProbe:Capture','Uncommitted row.');
        data(end+1,:)=[r.time_before,r.time_after,r.state_before.Xt(4),r.state_after.Xt(4), ...
            r.Xd(4,1),r.current_sample_A,r.state_after.energy.energyWh-r.state_before.energy.energyWh, ...
            r.state_after.energy.chargeAs-r.state_before.energy.chargeAs, ...
            r.action_execution.v_exec,r.action_execution.a_exec]; %#ok<AGROW>
    end
end
trace=array2table(data,'VariableNames',{'interval_start_s','interval_end_s','actual_speed_start_m_s', ...
    'actual_speed_end_m_s','desired_speed_start_m_s','mean_pack_current_A','interval_energy_Wh', ...
    'interval_charge_As','speed_target_m_s','reference_acceleration_m_s2'});
end

function state=localSource(source)
state=struct('sha',localGit(source,{'rev-parse','HEAD'}),'branch',localGit(source,{'branch','--show-current'}), ...
    'status',localGit(source,{'status','--porcelain'})); state.dirty=~isempty(state.status);
end
function output=localGit(source,args)
command=java.util.ArrayList();
for value=[{'git','-C',source},args],command.add(java.lang.String(value{1}));end
builder=java.lang.ProcessBuilder(command);builder.redirectErrorStream(true);process=builder.start();
reader=java.io.BufferedReader(java.io.InputStreamReader(process.getInputStream(),'UTF-8'));lines=cell(0,1);
while true,line=reader.readLine();if isempty(line),break;end,lines{end+1,1}=char(line);end %#ok<AGROW>
output=strjoin(lines,newline);assert(process.waitFor()==0,'accelProbe:Source','Cannot read source provenance.');
end
function localJson(path,value)
fid=fopen(path,'w');assert(fid>=0,'accelProbe:Write','Cannot write evidence.');
cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
function localIndex(root)
files=dir(fullfile(root,'**','*'));rows=cell(0,3);target=fullfile(root,'artifact_sha256.csv');
for k=1:numel(files)
    if files(k).isdir,continue;end,path=fullfile(files(k).folder,files(k).name);
    if strcmp(path,target),continue;end
    rows(end+1,:)={path(numel(root)+2:end),files(k).bytes,sha256_file(path)}; %#ok<AGROW>
end
writetable(cell2table(rows,'VariableNames',{'file','bytes','sha256'}),target);
end
