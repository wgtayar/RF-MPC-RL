function reports = run_phase3_exact_solver_abc(sourceDataset, rowId, outputRoot, durationSeconds, tightRescue)
%run_phase3_exact_solver_abc Captured solver-only branches at an exact failed row.
% Frozen supervisory action; chunk battery timing is retained. This is a local
% continuation, not a full policy/mission evaluation or calibrated safety test.
    if nargin < 4
        durationSeconds = 10;
    end
    if nargin < 5
        tightRescue = false;
    end
    validateattributes(tightRescue,{'logical'},{'scalar'});
    validateattributes(durationSeconds,{'double'},{'scalar','real','finite','positive'});
    source = bootstrap_RF_MPC_RL();
    assert(~isfolder(outputRoot) && ~isfile(outputRoot), ...
        'solverABC:Exists','Refusing to overwrite experiment.');
    config = load(fullfile(sourceDataset,'manifest','configuration.mat'),'cfg');
    cfg = config.cfg;
    [initial,control,p] = restore_exact_dataset_state(sourceDataset,rowId);
    assert(abs(durationSeconds/p.simTimeStep-round(durationSeconds/p.simTimeStep))<1e-8, ...
        'solverABC:Duration','Duration must be an integer number of MPC timesteps.');
    index = readtable(fullfile(sourceDataset,'mpc_steps','index.csv'),TextType='string');
    fileIndex = find(index.first_row<=rowId & index.last_row>=rowId,1);
    rows = read_exact_state_rows(fullfile(sourceDataset,index.file(fileIndex)));
    target = rows{rowId-index.first_row(fileIndex)+1};
    assert(~target.integrated && ~target.solver.success && ...
        strcmp(target.solver.classification,'numerical_solver_failure'), ...
        'solverABC:Source','Source must be an archived unintegrated numerical failure.');
    savedQP = load(fullfile(sourceDataset,target.qp_reference.file),'exactQP');
    savedStart = load(fullfile(sourceDataset,target.snapshot_reference.file),'snapshot');
    firstBoundary = savedStart.snapshot.state.t+cfg.CHUNK_DURATION;
    assert(firstBoundary>initial.t,'solverABC:Boundary','Invalid original chunk boundary.');
    assert(initial.t+durationSeconds <= cfg.MISSION_DURATION, ...
        'solverABC:MissionBoundary','Continuation must not extend beyond the mission time limit.');
    strategies = {'default','default_one_shot_fallback','active_set_feasible_point'};
    if tightRescue
        strategies(2:3) = {'default_one_shot_fallback_tight_primal_v1','active_set_tight_primal_v1'};
    end
    branchNames = {'A_default','B_one_shot','C_active_set'};
    reports = cell(3,1);
    for branch = 1:3
        metadata = struct('run_id',branchNames{branch},'run_type','exact_solver_abc', ...
            'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',cfg.RNG_SEED, ...
            'observation_schema_version','archived_state_no_actor', ...
            'reward_version',cfg.REWARD.version,'solver_strategy',strategies{branch}, ...
            'source_policy','archived_action_frozen','source_dataset',sourceDataset, ...
            'source_row_id',rowId,'duration_seconds',durationSeconds, ...
            'scope','fixed_action_local_continuation_not_mission_outcome');
        writer = ExactStateDataset(fullfile(outputRoot,branchNames{branch}),cfg,metadata);
        try
            state = initial;
            branchControl = control;
            stopTime = initial.t+durationSeconds;
            boundary = firstBoundary;
            chunk = 0;
            outcomes = cell(0,1);
            timer = tic;
            while state.t < stopTime-p.simTimeStep/2
                chunk = chunk+1;
                options = struct('duration_s',min(boundary,stopTime)-state.t, ...
                    'solver_strategy',strategies{branch},'capture_trace',true, ...
                    'capture_first_problem',true,'update_battery',true,'dataset_writer',writer, ...
                    'dataset_context',struct('episode',1,'decision',1,'chunk',chunk));
                [state,out] = simulate_mpc_horizon(state,branchControl,cfg,options);
                outcomes{end+1,1} = out; %#ok<AGROW>
                if chunk == 1
                    comparison = compare_exact_qp_rebuild(out.first_problem,savedQP.exactQP,target);
                    save(fullfile(writer.Root,'qp_rebuild_comparison.mat'),'comparison');
                    assert(comparison.exact_match,'solverABC:Rebuild','Archived QP/state did not rebuild exactly.');
                    if branch == 1
                        assert(out.qp_solve_count==1 && ~out.completed_horizon && ...
                            strcmp(out.terminal_reason,'numerical_solver_failure') && ...
                            out.failure_solver.exitflag==target.solver.exitflag, ...
                            'solverABC:DefaultReplay','Default did not reproduce the archived failure.');
                    end
                end
                if ~out.completed_horizon
                    break
                end
                boundary = boundary+cfg.CHUNK_DURATION;
                if isfield(branchControl,'integration_time_origin_s')
                    branchControl = rmfield(branchControl,'integration_time_origin_s');
                end
            end
            report = struct('branch',branchNames{branch},'strategy',strategies{branch}, ...
                'source_sha',writer.Manifest.source_sha,'source_dirty',writer.Manifest.source_dirty, ...
                'source_dataset',sourceDataset,'source_row_id',rowId,'duration_seconds',durationSeconds, ...
                'survived_seconds',state.t-initial.t,'terminal_reason',out.terminal_reason, ...
                'rollout_wall_seconds',toc(timer),'qp_rebuild',comparison, ...
                'metrics',phase3_episode_metrics(writer.Root,cfg,1), ...
                'solver_sequence',localSolverSequence(writer.Root));
            save(fullfile(writer.Root,'solver_abc_report.mat'),'report','outcomes','-v7.3');
            writer.finish(char(out.terminal_reason));
            report.validation = validate_exact_state_dataset(writer.Root);
            reports{branch} = report;
            fprintf('[%s] survived=%.4fs terminal=%s fallback=%d/%d peak_orientation=%.4f\n', ...
                report.branch,report.survived_seconds,report.terminal_reason, ...
                report.metrics.fallback_successes,report.metrics.fallback_attempts, ...
                report.metrics.orientation_rad.maximum);
        catch exception
            if ~writer.Closed
                failure = struct('identifier',exception.identifier,'message',exception.message);
                save(fullfile(writer.Root,'experiment_exception.mat'),'failure');
                writer.finish('solver_abc_experiment_exception');
            end
            rethrow(exception)
        end
    end
    save(fullfile(outputRoot,'comparison.mat'),'reports','-v7.3');
end

function sequence = localSolverSequence(root)
    index = readtable(fullfile(root,'mpc_steps','index.csv'),TextType='string');
    data = zeros(sum(index.row_count),6);
    n = 0;
    for k = 1:height(index)
        rows = read_exact_state_rows(fullfile(root,index.file(k)));
        for j = 1:numel(rows)
            n = n+1;
            solver = rows{j}.solver;
            defaultAttempt = any(strcmp(solver.strategy, ...
                {'default','default_one_shot_fallback','default_one_shot_fallback_tight_primal_v1'}));
            defaultSuccess = defaultAttempt && solver.success && ~solver.fallback_attempted;
            data(n,:) = [rows{j}.time_before,defaultAttempt,defaultSuccess, ...
                solver.fallback_attempted,solver.fallback_success,solver.fallback_wall_time_s];
        end
    end
    sequence = array2table(data,VariableNames={'time_s','default_attempted','default_succeeded', ...
        'fallback_attempted','fallback_succeeded','fallback_wall_seconds'});
end
