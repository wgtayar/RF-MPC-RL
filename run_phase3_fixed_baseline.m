function report = run_phase3_fixed_baseline(policy, durationSeconds, outputRoot, strategy)
%run_phase3_fixed_baseline Collect a fixed-R policy through the captured MPC engine.
% Candidates change only on supervisory boundaries, not each logging segment.
    if nargin < 4
        strategy = 'default';
    end
    validateattributes(durationSeconds,{'numeric'},{'scalar','finite','positive'});
    source = bootstrap_RF_MPC_RL();
    timer = tic;
    loaded = load(fullfile(source,'rlEnv_MPC_R.mat'),'cfg','initial_R','lower_abs','upper_abs');
    cfg = loaded.cfg;
    p = get_params(0);
    numberSteps = durationSeconds/p.simTimeStep;
    assert(abs(numberSteps-round(numberSteps)) < 1e-8, ...
        'run_phase3_fixed_baseline:DurationGrid', ...
        'Duration must contain an integer number of MPC timesteps.');
    baseline_supervisory_action(policy,0,cfg);
    previousRng = rng;
    rngCleanup = onCleanup(@() rng(previousRng));
    rng(cfg.RNG_SEED);
    monitor = fullfile(fileparts(source),'RL-MPC-Monitor');
    [~,runId] = fileparts(outputRoot);
    metadata = struct('run_id',runId,'run_type','fixed_policy_baseline', ...
        'monitor_root',monitor,'seed',cfg.RNG_SEED, ...
        'observation_schema_version','not_used_fixed_policy', ...
        'reward_version',cfg.REWARD.version,'solver_strategy',strategy,'source_policy',policy);
    writer = ExactStateDataset(outputRoot,cfg,metadata);
    state = initialize_mpc_replay_state(cfg,0);
    state.R = loaded.initial_R(:);
    state.supervisory_state.last_R = state.R;
    initialPosition = state.Xt(1);
    traces = {};
    segment = 0;
    outcome = 'probe_horizon_complete';
    if durationSeconds >= cfg.MISSION_DURATION
        outcome = 'time_limit';
    end
    while state.t < durationSeconds-1e-9
        segment = segment+1;
        decision = floor((segment-1)/cfg.APPLY_EVERY)+1;
        chunk = mod(segment-1,cfg.APPLY_EVERY)+1;
        if chunk == 1
            candidate = baseline_supervisory_action(policy,state.t,cfg);
            control = resolve_replay_action(candidate,state,cfg,loaded.lower_abs,loaded.upper_abs);
        end
        options = struct('duration_s',min(cfg.CHUNK_DURATION,durationSeconds-state.t), ...
            'solver_strategy',strategy,'capture_trace',true,'update_battery',true, ...
            'dataset_writer',writer,'dataset_context', ...
            struct('episode',1,'decision',decision,'chunk',chunk));
        [state,out] = simulate_mpc_horizon(state,control,cfg,options);
        traces{end+1,1} = out.trace; %#ok<AGROW>
        if ~out.completed_horizon
            outcome = char(out.terminal_reason);
            break
        end
        if state.Xt(1)-initialPosition >= cfg.MISSION.D_TARGET_M
            outcome = 'mission_complete';
            break
        end
        if state.battery.margin_norm <= cfg.BATTERY.terminal_margin
            outcome = 'battery_terminal';
            break
        end
    end
    writer.finish(outcome);
    validation = validate_exact_state_dataset(outputRoot);
    trace = vertcat(traces{:});
    files = dir(fullfile(outputRoot,'**','*'));
    report = struct('policy',policy,'strategy',strategy,'source_sha',writer.Manifest.source_sha, ...
        'source_dirty',writer.Manifest.source_dirty,'requested_duration_s',durationSeconds, ...
        'survived_duration_s',state.t,'outcome',outcome,'distance_m',state.Xt(1)-initialPosition, ...
        'qp_solves',height(trace),'solver_failures',nnz(trace.solver_exitflag <= 0), ...
        'numerical_rescues',nnz(trace.solver_classification == "numerical_solver_failure_recovered"), ...
        'solver_deadline_exceedances',nnz(trace.solver_wall_time_s > p.simTimeStep), ...
        'max_orientation_rad',max(trace.orientation_error_after_rad,[],'omitnan'), ...
        'max_angular_velocity',max(trace.angular_velocity_after,[],'omitnan'), ...
        'max_velocity_error',max(trace.linear_velocity_error_after,[],'omitnan'), ...
        'solver_wall_time_median_s',median(trace.solver_wall_time_s,'omitnan'), ...
        'final_soc_pct',state.battery.soc_pct,'append_logging_seconds',writer.LoggingSeconds, ...
        'total_wall_seconds',toc(timer),'dataset_bytes',sum([files.bytes]),'validation',validation);
    disp(report);
end
