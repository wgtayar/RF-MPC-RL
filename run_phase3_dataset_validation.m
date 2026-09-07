function report = run_phase3_dataset_validation(outputRoot)
%run_phase3_dataset_validation Exercise real integration and exact branch restore.
% This is a short correctness experiment, not an MPC-health or safety claim.
    source = bootstrap_RF_MPC_RL();
    loaded = load(fullfile(source,'rlEnv_MPC_R.mat'),'cfg');
    cfg = loaded.cfg;
    monitor = fullfile(fileparts(source),'RL-MPC-Monitor');
    [~,runId] = fileparts(outputRoot);
    metadata = struct('run_id',runId,'run_type','exact_state_validation', ...
        'monitor_root',monitor,'seed',cfg.RNG_SEED, ...
        'observation_schema_version','observation_v1_legacy', ...
        'reward_version',cfg.REWARD.version,'solver_strategy','default', ...
        'source_policy','fixed_mission_gamma_fixed_R');
    writer = ExactStateDataset(outputRoot,cfg,metadata);
    state0 = initialize_mpc_replay_state(cfg,0);
    p = get_params(0);
    [control,~] = resolve_replay_action([0;0;0;cfg.GAMMA_V_MISSION;cfg.GAMMA_A_MIN], ...
        state0,cfg,0.95*state0.R,1.05*state0.R);
    options = struct('duration_s',5*p.simTimeStep,'capture_trace',true, ...
        'update_battery',true,'dataset_writer',writer, ...
        'dataset_context',struct('episode',1,'decision',1,'chunk',1));
    timer = tic;
    [branchState,prefix] = simulate_mpc_horizon(state0,control,cfg,options);
    assert(prefix.completed_horizon && prefix.qp_solve_count == 5, ...
        'run_phase3_dataset_validation:Prefix','Five-step prefix did not complete.');
    options.duration_s = 3*p.simTimeStep;
    options.dataset_context.chunk = 2;
    [stateA,outA] = simulate_mpc_horizon(branchState,control,cfg,options);
    options.dataset_context = struct('episode',2,'decision',1,'chunk',1);
    [stateB,outB] = simulate_mpc_horizon(branchState,control,cfg,options);
    options.duration_s = 8*p.simTimeStep;
    options.dataset_context.episode = 3;
    [stateC,outC] = simulate_mpc_horizon(state0,control,cfg,options);
    wallSeconds = toc(timer);
    assert(outA.completed_horizon && outB.completed_horizon && outC.completed_horizon, ...
        'run_phase3_dataset_validation:Continuation','A nonzero continuation failed.');
    branchError = max(abs(stateA.Xt-stateB.Xt));
    splitError = max(abs(stateA.Xt-stateC.Xt));
    assert(branchError <= 1e-12 && splitError <= 1e-10 && ...
        max(abs(stateA.Ut-stateB.Ut)) <= 1e-12 && ...
        max(abs(stateA.Ut-stateC.Ut)) <= 1e-10, ...
        'run_phase3_dataset_validation:Restore','Restored or segmented dynamics differ.');
    assert(isequaln(stateA.fsm_internal_state,stateB.fsm_internal_state) && ...
        isequaln(stateA.knee_proxy_state,stateB.knee_proxy_state) && ...
        isequaln(stateA.current_time,stateB.current_time) && ...
        isequaln(stateA.current_total,stateB.current_total) && ...
        isequaln(stateA.battery,stateB.battery), ...
        'run_phase3_dataset_validation:HiddenRestore','Hidden state/battery reconstruction differs.');
    writer.finish('short_restore_validation_complete');
    report = validate_exact_state_dataset(outputRoot);
    report.restore_equivalence_tested = true;
    report.branch_max_state_error = branchError;
    report.split_max_state_error = splitError;
    report.wall_seconds = wallSeconds;
    report.append_logging_seconds = writer.LoggingSeconds;
    report.scope = '19 MPC solves; not long-horizon health evidence';
    disp(report);
end
