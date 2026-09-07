function report = run_phase3_reference_adapter_probe(sourceDataset, outputRoot)
%run_phase3_reference_adapter_probe Isolate position continuity for one legal action.
    source = bootstrap_RF_MPC_RL();
    validate_exact_state_dataset(sourceDataset);
    complete = load(fullfile(sourceDataset,'manifest','completion.mat'),'completion');
    last = complete.completion.segments;
    before = load(fullfile(sourceDataset,'snapshots', ...
        sprintf('segment_%06d_before.mat',last)),'snapshot');
    after = load(fullfile(sourceDataset,'snapshots', ...
        sprintf('segment_%06d_after.mat',last)),'snapshot');
    config = load(fullfile(sourceDataset,'manifest','configuration.mat'),'cfg');
    cfg = config.cfg;
    initial = after.snapshot.state;
    prior = before.snapshot.control;
    initial.reference_state = struct('schema_version','reference_position_state_v1', ...
        'mode','legacy_absolute_time','v_cmd',prior.v_cmd,'a_cmd',prior.a_cmd,'position_offset_m',0);
    metadata = struct('run_id','reference_adapter_probe','run_type','controlled_reference_ablation', ...
        'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',cfg.RNG_SEED, ...
        'observation_schema_version','not_used_fixed_candidates','reward_version',cfg.REWARD.version, ...
        'solver_strategy','default','source_policy','identical_action_legacy_vs_continuous_reference', ...
        'source_dataset',sourceDataset);
    writer = ExactStateDataset(outputRoot,cfg,metadata);
    modes = ["legacy_absolute_time","position_continuous_v1"];
    outcomes = cell(2,1);
    rows = repmat(struct(),2,1);
    for branch = 1:2
        control = resolve_replay_action([zeros(3,1);cfg.GAMMA_V_MAX;cfg.GAMMA_A_MIN], ...
            initial,cfg,initial.R,initial.R);
        control.reference_mode = modes(branch);
        options = struct('duration_s',5,'capture_trace',true,'capture_first_problem',true, ...
            'update_battery',true,'dataset_writer',writer, ...
            'dataset_context',struct('episode',branch,'decision',2,'chunk',1));
        [state,outcomes{branch}] = simulate_mpc_horizon(initial,control,cfg,options);
        trace = outcomes{branch}.trace;
        rows(branch).mode = modes(branch);
        rows(branch).survived_s = outcomes{branch}.survived_duration_s;
        rows(branch).terminal = outcomes{branch}.terminal_reason;
        rows(branch).orientation_max_rad = max(trace.orientation_error_after_rad,[],'omitnan');
        rows(branch).angular_velocity_max = max(trace.angular_velocity_after,[],'omitnan');
        rows(branch).velocity_error_max = max(trace.linear_velocity_error_after,[],'omitnan');
        rows(branch).first_force_correction_norm = trace.force_correction_norm(1);
        rows(branch).end_position_x = state.Xt(1);
        rows(branch).final_soc_pct = state.battery.soc_pct;
    end
    writer.finish('reference_adapter_probe_complete');
    report = struct('source_sha',writer.Manifest.source_sha,'source_dirty',writer.Manifest.source_dirty, ...
        'source_dataset',sourceDataset,'branches',struct2table(rows), ...
        'first_reference_position_difference_m',outcomes{2}.first_problem.Xd(1,1)-outcomes{1}.first_problem.Xd(1,1), ...
        'first_reference_velocity_difference',norm(outcomes{2}.first_problem.Xd(4:6,:)-outcomes{1}.first_problem.Xd(4:6,:)), ...
        'first_qp_state_difference',norm(outcomes{2}.first_problem.Xt-outcomes{1}.first_problem.Xt), ...
        'validation',validate_exact_state_dataset(outputRoot));
    disp(report);
    disp(report.branches);
end
