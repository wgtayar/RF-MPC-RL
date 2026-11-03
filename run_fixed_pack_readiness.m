function report = run_fixed_pack_readiness(root)
%RUN_FIXED_PACK_READINESS Captured, non-learning qualification of the fixed pack.
% Run from a clean source checkout. Each invocation needs a new local-only root.
% Software success and mission qualification are deliberately separate outputs.
src = bootstrap_RF_MPC_RL();
assert(~isfolder(root) && ~isfile(root),'readiness:Exists','Use a new output directory.');
assert(~isfile(fullfile(src,'rlEnv_MPC_R.mat')),'readiness:SavedBundle', ...
    'Qualification requires an explicit fresh configuration, without a saved worker bundle.');
sha = gitValue(src,'rev-parse HEAD');
assert(isempty(gitValue(src,'status --porcelain')),'readiness:Dirty','Commit source before qualification.');
mkdir(root);
b = setup_RL_MPC();
save(fullfile(root,'configuration.mat'),'b','-v7.3');
meta = struct('schema','fixed_pack_rl_readiness_v1','source_sha',sha, ...
    'source_branch',gitValue(src,'branch --show-current'),'source_dirty',false, ...
    'monitor_sha',gitValue(fullfile(fileparts(src),'RL-MPC-Monitor'),'rev-parse HEAD'), ...
    'timestamp_utc',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
    'matlab',version,'seed',b.cfg.RNG_SEED,'training',false,'episodes_trained',0, ...
    'learning_rates_used',[],'exploration_noise_used',[], ...
    'reward_version','reward_phase3_legacy_bridge_v1', ...
    'configuration_sha256',sha256_file(fullfile(root,'configuration.mat')), ...
    'workbook_sha256',sha256_file(b.cfg.BATTERY.model_options.parameter_source), ...
    'knee_csv_sha256',sha256_file(b.cfg.PROXY.kneeCsv), ...
    'scope','Fixed software regressions and captured deterministic policies; no hardware or learned-value claim.');
writeJson(root,'manifest',meta);
try
    names = {'testFixedPackEnergy','testFixedPackMath','testFixedPackFailureHandling', ...
        'testFixedPackTargetCrossing','testBatteryManuscriptConfig','testPhase3BatteryFeedback', ...
        'testMpcReplayTimeGrid','testReferencePositionContinuity','testPhase3TargetStop', ...
        'testPhase3TerminalSemantics','testFixedPackPolicyEvaluation','testFixedPackLegacyTiming'};
    files = cellfun(@(n)fullfile(src,'tests',[n '.m']),names,'UniformOutput',false);
    r = runtests(files);
    writetable(table(string({r.Name})',[r.Passed]',[r.Failed]',[r.Incomplete]', ...
        'VariableNames',{'name','passed','failed','incomplete'}),fullfile(root,'tests.csv'));
    assertSuccess(r);
    v = run_phase3_environment_validation(fullfile(root,'interface'),true);
    writeJson(root,'interface',struct('rows',v.qp_steps,'reset_equal',v.reset_equal, ...
        'dataset_verified',true,'toolbox_validation',v.toolbox_validation_executed));
    cases = cell(3,1);
    cases{1} = runCase(fullfile(root,'short_slow'),b,3,10,'slow');
    cases{2} = runCase(fullfile(root,'mission_average'),b,320,600,'mission_average');
    cases{3} = runCase(fullfile(root,'mission_fast'),b,320,600,'fast');
    % This pre-existing crosscheck includes the fixed 25% SOC / 0.50 m/s probe.
    crosscheck = run_fixed_pack_crosscheck(fullfile(root,'crosscheck'));
    missionFeasible = cases{2}.mission_complete || cases{3}.mission_complete;
    report = struct('schema',meta.schema,'source_sha',sha,'source_dirty',false, ...
        'tests_passed',nnz([r.Passed]),'tests_failed',nnz([r.Failed]), ...
        'tests_incomplete',nnz([r.Incomplete]),'interface_passed',true, ...
        'short_case_passed',cases{1}.mission_complete,'cases',{cases},'crosscheck',crosscheck, ...
        'mission_feasible_at_configured_initial_soc',missionFeasible, ...
        'training_soc_range_qualified',false,'reward_observation_promoted',false, ...
        'training_ready',false, ...
        'next_gate','Qualify the intended SOC range and mission, then approve observations/reward and baseline comparisons.');
    writeJson(root,'readiness',report);
catch exception
    writeJson(root,'failure',struct('source_sha',sha,'identifier',exception.identifier, ...
        'message',exception.message,'stack',{exception.stack}));
    rethrow(exception)
end
end

function result = runCase(root,b,target,deadline,policy)
b.cfg.MISSION.D_TARGET_M = target;
b.cfg.MISSION_DURATION = deadline;
b.cfg.EP_STEPS = ceil(deadline/(b.cfg.CHUNK_DURATION*b.cfg.APPLY_EVERY));
b.cfg.MISSION.WINDOW_TARGET_M = target/b.cfg.EP_STEPS;
[~,name] = fileparts(root);
src = fileparts(mfilename('fullpath'));
metadata = struct('run_id',name,'run_type','fixed_pack_readiness', ...
    'monitor_root',fullfile(fileparts(src),'RL-MPC-Monitor'), ...
    'seed',b.cfg.RNG_SEED,'source_policy',policy,'training',false, ...
    'learning_rates_used',[],'exploration_noise_used',[]);
options = struct('reference_mode','position_continuous_v1', ...
    'solver_strategy','default_one_shot_fallback_tight_primal_v1', ...
    'mission_end_mode','mpc_step_target_v1');
env = Phase3MpcEnvironment(b,root,metadata,options);
try
    reset(env);
    switch policy
        case 'slow', gamma = (.5-b.cfg.V_MIN)/(b.cfg.V_MAX-b.cfg.V_MIN);
        case 'mission_average', gamma = b.cfg.GAMMA_V_MISSION;
        case 'fast', gamma = b.cfg.GAMMA_V_MAX;
    end
    while ~env.Done
        [observation,reward,~,info] = step(env,[zeros(3,1);gamma;b.cfg.GAMMA_A_MIN]);
        assert(all(isfinite(observation)) && isfinite(reward),'readiness:RLTransition','Nonfinite RL transition.');
    end
    metrics = phase3_episode_metrics(root,env.Config,env.Episode);
    result = struct('policy',policy,'target_m',target,'deadline_s',deadline, ...
        'initial_soc',b.cfg.BATTERY.SOC_init,'terminal',info.terminal_reason, ...
        'mission_complete',strcmp(info.terminal_reason,'mission_complete'), ...
        'time_s',env.State.t,'distance_m',info.window.distance_end_m, ...
        'decisions',env.Decision,'battery',env.State.battery,'metrics',metrics);
    close(env,'readiness_case_complete');
    validate_exact_state_dataset(root);
    result.dataset_validated = true;
    result.drive_trace = exportDriveTrace(root);
    writeJson(root,'result',result);
catch exception
    if ~env.Writer.Closed, close(env,'readiness_case_exception'); end
    rethrow(exception)
end
end

function summary = exportDriveTrace(root)
index = readtable(fullfile(root,'mpc_steps','index.csv'),'TextType','string');
values = zeros(12*sum(index.row_count),12); n = 0;
for f = 1:height(index)
    rows = read_exact_state_rows(fullfile(root,index.file(f)));
    for k = 1:numel(rows)
        row = rows{k};
        if ~row.current_sample_committed, continue; end
        b = row.state_after.battery;
        for leg = 1:4
            for axis = 1:3
                n = n+1;
                values(n,:) = [row.row_id,row.time_before,row.time_after,leg,axis, ...
                    b.drive_joint_speed_rad_s(axis,leg),b.drive_joint_torque_Nm(axis,leg), ...
                    b.drive_motor_speed_rad_s(axis,leg),b.drive_phase_current_A(axis,leg), ...
                    b.drive_motor_voltage_V(axis,leg),b.pack_voltage_V, ...
                    b.pack_voltage_V-abs(b.drive_motor_voltage_V(axis,leg))];
            end
        end
    end
end
T = array2table(values(1:n,:),'VariableNames',{'row','time_before_s','time_after_s', ...
    'leg','axis','joint_speed_rad_s','joint_torque_Nm','motor_speed_rad_s','phase_current_A', ...
    'motor_voltage_V','pack_voltage_V','voltage_headroom_V'});
writetable(T,fullfile(root,'drive_trace.csv'));
summary = struct('file','drive_trace.csv','rows',n, ...
    'sha256',sha256_file(fullfile(root,'drive_trace.csv')), ...
    'speed_convention','signed joint displacement / MPC interval; not continuous-time peak speed', ...
    'minimum_voltage_headroom_V',min(T.voltage_headroom_V));
end

function value = gitValue(root,args)
[status,value] = system(sprintf('git -C "%s" %s',root,args));
assert(status==0,'readiness:Git','Git provenance failed.'); value = strtrim(value);
end
function writeJson(root,name,value)
f = fopen(fullfile(root,[name '.json']),'w'); assert(f>=0);
clean = onCleanup(@()fclose(f)); fprintf(f,'%s\n',jsonencode(value,'PrettyPrint',true));
end
