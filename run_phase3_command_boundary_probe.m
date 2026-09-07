function report = run_phase3_command_boundary_probe(sourceDataset, outputRoot)
%run_phase3_command_boundary_probe Compare hold/increase from one exact state.
    source = bootstrap_RF_MPC_RL();
    validate_exact_state_dataset(sourceDataset);
    complete = load(fullfile(sourceDataset,'manifest','completion.mat'),'completion');
    filename = sprintf('segment_%06d_after.mat',complete.completion.segments);
    saved = load(fullfile(sourceDataset,'snapshots',filename),'snapshot');
    initial = saved.snapshot.state;
    config = load(fullfile(sourceDataset,'manifest','configuration.mat'),'cfg');
    cfg = config.cfg;
    metadata = struct('run_id','command_boundary_probe','run_type','controlled_command_ablation', ...
        'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',cfg.RNG_SEED, ...
        'observation_schema_version','not_used_fixed_candidates','reward_version',cfg.REWARD.version, ...
        'solver_strategy','default','source_policy','hold_vs_legal_gamma_increase', ...
        'source_dataset',sourceDataset);
    writer = ExactStateDataset(outputRoot,cfg,metadata);
    candidates = [zeros(3,2);initial.supervisory_state.prev_gamma_v,cfg.GAMMA_V_MAX; ...
        cfg.GAMMA_A_MIN,cfg.GAMMA_A_MIN];
    outcomes = cell(2,1);
    controls = cell(2,1);
    finalStates = cell(2,1);
    for branch = 1:2
        % Freeze R explicitly to isolate the legal speed/governor transition.
        control = resolve_replay_action(candidates(:,branch),initial,cfg,initial.R,initial.R);
        options = struct('duration_s',5,'solver_strategy','default','capture_trace',true, ...
            'capture_first_problem',true,'update_battery',true,'dataset_writer',writer, ...
            'dataset_context',struct('episode',branch,'decision',2,'chunk',1));
        [finalStates{branch},outcomes{branch}] = simulate_mpc_horizon(initial,control,cfg,options);
        controls{branch} = control;
    end
    writer.finish('command_boundary_probe_complete');
    hold = controls{1};
    increase = controls{2};
    predictedJump = initial.t*(increase.v_cmd-hold.v_cmd) - ...
        increase.v_cmd^2/(2*increase.a_cmd)+hold.v_cmd^2/(2*hold.a_cmd);
    measuredJump = outcomes{2}.first_problem.Xd(1,1)-outcomes{1}.first_problem.Xd(1,1);
    rows = repmat(struct(),2,1);
    names = ["hold","legal_increase"];
    for branch = 1:2
        trace = outcomes{branch}.trace;
        rows(branch).branch = names(branch);
        rows(branch).v_exec = controls{branch}.v_cmd;
        rows(branch).a_exec = controls{branch}.a_cmd;
        rows(branch).survived_s = outcomes{branch}.survived_duration_s;
        rows(branch).terminal = outcomes{branch}.terminal_reason;
        rows(branch).orientation_max_rad = max(trace.orientation_error_after_rad,[],'omitnan');
        rows(branch).angular_velocity_max = max(trace.angular_velocity_after,[],'omitnan');
        rows(branch).velocity_error_max = max(trace.linear_velocity_error_after,[],'omitnan');
        rows(branch).first_force_correction_norm = trace.force_correction_norm(1);
        rows(branch).force_correction_max = max(trace.force_correction_norm,[],'omitnan');
        rows(branch).end_position_x = finalStates{branch}.Xt(1);
        rows(branch).final_soc_pct = finalStates{branch}.battery.soc_pct;
    end
    report = struct('source_sha',writer.Manifest.source_sha,'source_dirty',writer.Manifest.source_dirty, ...
        'conditioning_time_s',initial.t,'source_dataset',sourceDataset, ...
        'predicted_reference_jump_m',predictedJump,'measured_reference_jump_m',measuredJump, ...
        'reference_jump_prediction_error_m',abs(predictedJump-measuredJump), ...
        'first_qp_state_difference_norm',norm(outcomes{2}.first_problem.Xt-outcomes{1}.first_problem.Xt), ...
        'branches',struct2table(rows),'validation',validate_exact_state_dataset(outputRoot));
    disp(report);
    disp(report.branches);
end
