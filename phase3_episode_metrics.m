function metrics = phase3_episode_metrics(root, cfg, episode)
%phase3_episode_metrics Read flushed captured rows for a policy benchmark.
% External callers should validate the finalized dataset first. The evaluation
% runner calls this after its last flushed decision, before final checksums.
    validateattributes(episode,{'double'},{'scalar','positive','integer'});
    index = readtable(fullfile(root,'mpc_steps','index.csv'),'TextType','string');
    names = {'time_before','time_after','x_before','x_after','integrated','solver_success', ...
        'iterations','solver_wall_s','quadprog_wall_s','fallback_attempted','fallback_success', ...
        'fallback_wall_s','orientation_before','orientation_after','omega_before','omega_after', ...
        'velocity_error_before','velocity_error_after','Ut_before','Ut_after','constraint_violation'};
    values = nan(sum(index.row_count),numel(names));
    classifications = strings(size(values,1),1);
    samples = nan(size(values,1),2);
    count = 0;
    sampleCount = 0;
    initial = struct();
    lastSegment = 0;
    timestep = NaN;
    for file = 1:height(index)
        rows = read_exact_state_rows(fullfile(root,index.file(file)));
        for k = 1:numel(rows)
            row = rows{k};
            if row.context.episode ~= episode
                continue
            end
            if count == 0
                saved = load(fullfile(root,row.snapshot_reference.file),'snapshot');
                initial = saved.snapshot.state;
                timestep = saved.snapshot.parameters.simTimeStep;
            end
            count = count+1;
            a = localHealth(row.health_before);
            b = localHealth(row.health_after);
            s = row.solver;
            values(count,:) = [row.time_before,row.time_after,row.state_before.Xt(1), ...
                row.state_after.Xt(1),row.integrated,s.success,s.iterations,s.wall_time_s, ...
                s.quadprog_wall_time_s,s.fallback_attempted,s.fallback_success,s.fallback_wall_time_s, ...
                a(1),b(1),a(2),b(2),a(3),b(3),a(4),b(4),s.constraint_violation];
            classifications(count) = s.classification;
            if row.current_sample_committed
                sampleCount = sampleCount+1;
                samples(sampleCount,:) = [row.current_sample_time,row.current_sample_A];
            end
            lastSegment = row.segment;
        end
    end
    assert(count > 0,'phase3_episode_metrics:NoRows','Episode has no recorded MPC solves.');
    T = array2table(values(1:count,:),'VariableNames',names);
    classifications = classifications(1:count);
    current = struct('time',[initial.current_time(:);samples(1:sampleCount,1)], ...
        'total',[initial.current_total(:);samples(1:sampleCount,2)]);
    final = load(fullfile(root,'snapshots',sprintf('segment_%06d_after.mat',lastSegment)),'snapshot');
    finalState = final.snapshot.state;
    origin = initial.decision_bookkeeping.initial_position_x;
    steps = struct('integrated',logical(T.integrated),'time_before',T.time_before,'time_after',T.time_after, ...
        'distance_before',T.x_before-origin,'distance_after',T.x_after-origin);
    crossing = phase3_target_crossing(steps,cfg.MISSION.D_TARGET_M,current,cfg.BATTERY);
    duration = finalState.t-initial.t;
    numerical = classifications == "numerical_solver_failure" | ...
        classifications == "numerical_solver_failure_recovered" | ...
        classifications == "numerical_solver_failure_unrecovered";
    charge = localCharge(current.time,current.total);
    scopeCharge = charge-localCharge(initial.current_time,initial.current_total);
    currentEquivalent = NaN;
    if duration > 0
        currentEquivalent = scopeCharge/duration;
    end
    metrics = struct('schema_version','policy_benchmark_v1','episode',episode, ...
        'scope_start_s',initial.t,'scope_end_s',finalState.t,'scope_duration_s',duration, ...
        'distance_m',finalState.Xt(1)-origin,'target_crossing',crossing, ...
        'final_soc_pct',finalState.battery.soc_pct,'sampled_charge_total_As',charge, ...
        'sampled_charge_in_scope_As',scopeCharge,'Ieq_scope_A',currentEquivalent, ...
        'qp_solves',count,'failed_qps',nnz(~T.solver_success), ...
        'numerical_events',nnz(numerical),'numerical_events_per_million_qps',1e6*nnz(numerical)/count, ...
        'mathematical_infeasibilities',nnz(classifications == "mathematical_constraint_infeasible"), ...
        'unclassified_solver_failures',nnz(classifications == "unclassified_solver_failure"), ...
        'fallback_attempts',nnz(T.fallback_attempted),'fallback_successes',nnz(T.fallback_success), ...
        'mpc_timestep_s',timestep,'deadline_exceedances',nnz(T.solver_wall_s > timestep), ...
        'solver_wall_s',localDistribution(T.solver_wall_s), ...
        'quadprog_wall_s',localDistribution(T.quadprog_wall_s), ...
        'iterations',localDistribution(T.iterations), ...
        'fallback_wall_s',localDistribution(T.fallback_wall_s(logical(T.fallback_attempted))), ...
        'orientation_rad',localDistribution([T.orientation_before;T.orientation_after]), ...
        'angular_velocity',localDistribution([T.omega_before;T.omega_after]), ...
        'velocity_error',localDistribution([T.velocity_error_before;T.velocity_error_after]), ...
        'force_norm',localDistribution([T.Ut_before;T.Ut_after]), ...
        'current_A',localDistribution(abs(samples(1:sampleCount,2))), ...
        'time_outside_validated_envelope_s',NaN,'health_envelope_validated',false);
    late = samples(1:sampleCount,1) >= initial.t+0.8*duration;
    metrics.late_current_window_start_s = initial.t+0.8*duration;
    metrics.late_current_A = localDistribution(abs(samples(late,2)));
    decisions = dir(fullfile(root,'decisions',sprintf('episode_%06d_decision_*.mat',episode)));
    distances = zeros(numel(decisions),1);
    modified = false(numel(decisions),1);
    for k = 1:numel(decisions)
        saved = load(fullfile(decisions(k).folder,decisions(k).name),'event');
        action = saved.event.record.action_execution;
        distances(k) = norm(action.candidate_action-action.applied_action);
        modified(k) = action.gamma_rate_limited || action.R_saturated || ...
            any(action.candidate_action(1:3) ~= action.dR_clipped) || action.gamma_a_raw ~= action.gamma_a_applied;
    end
    metrics.decisions = numel(decisions);
    metrics.candidate_to_applied_distance = localDistribution(distances);
    metrics.rate_limit_or_saturation_fraction = mean(modified);
    metrics.viability_projection_enabled = false;
end

function values = localHealth(health)
    values = nan(1,4);
    if health.valid
        values = [health.components.orientation_error_rad,health.components.angular_velocity_norm, ...
            health.linear_velocity_error,health.Ut_norm];
    end
end

function result = localDistribution(values)
    valid = values(isfinite(values));
    result = struct('count',numel(values),'finite_count',numel(valid),'mean',NaN, ...
        'median',NaN,'p95',NaN,'maximum',NaN);
    if ~isempty(valid)
        result.mean = mean(valid);
        result.median = median(valid);
        result.p95 = prctile(valid,95);
        result.maximum = max(valid);
    end
end

function charge = localCharge(time,current)
    charge = 0;
    if numel(time) >= 2
        charge = trapz(time,abs(current));
    end
end
