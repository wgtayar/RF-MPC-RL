function report = run_phase3_disk_restore_validation(sourceDataset, outputRoot, rowId)
%run_phase3_disk_restore_validation Repeat archived MPC rows from disk state.
% Replay through the original segment endpoint, preserving battery-update timing.
    if nargin < 3
        rowId = 6;
    end
    source = bootstrap_RF_MPC_RL();
    config = load(fullfile(sourceDataset,'manifest','configuration.mat'),'cfg');
    cfg = config.cfg;
    [state,control,p] = restore_exact_dataset_state(sourceDataset,rowId);
    index = readtable(fullfile(sourceDataset,'mpc_steps','index.csv'),'TextType','string');
    fileIndex = find(index.first_row <= rowId & index.last_row >= rowId,1);
    rows = read_exact_state_rows(fullfile(sourceDataset,index.file(fileIndex)));
    target = rows{rowId-index.first_row(fileIndex)+1};
    saved = load(fullfile(sourceDataset,'snapshots', ...
        sprintf('segment_%06d_after.mat',target.segment)),'snapshot');
    expected = saved.snapshot.state;
    archivedRows = {};
    for k = fileIndex:height(index)
        rows = read_exact_state_rows(fullfile(sourceDataset,index.file(k)));
        for j = 1:numel(rows)
            if rows{j}.segment == target.segment && rows{j}.row_id >= rowId
                archivedRows{end+1,1} = rows{j}; %#ok<AGROW>
            end
        end
    end
    assert(all(cellfun(@(row) row.integrated && row.solver.success,archivedRows)), ...
        'run_phase3_disk_restore_validation:UnsupportedTerminal', ...
        'This equivalence probe requires a fully integrated segment remainder.');
    strategy = target.solver.strategy;
    assert(all(cellfun(@(row) strcmp(row.solver.strategy,strategy),archivedRows)), ...
        'run_phase3_disk_restore_validation:MixedStrategy', ...
        'A changing solver schedule requires an explicit replay schedule.');
    numberSteps = numel(archivedRows);
    metadata = struct('run_id','disk_restore_validation','run_type','exact_state_validation', ...
        'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',cfg.RNG_SEED, ...
        'observation_schema_version','observation_v1_legacy','reward_version',cfg.REWARD.version, ...
        'solver_strategy',strategy,'source_policy','archived_segment_remainder', ...
        'source_dataset',sourceDataset,'source_row_id',rowId,'source_segment',target.segment);
    writer = ExactStateDataset(outputRoot,cfg,metadata);
    options = struct('duration_s',numberSteps*p.simTimeStep,'capture_trace',true, ...
        'solver_strategy',strategy, ...
        'update_battery',true,'dataset_writer',writer, ...
        'dataset_context',struct('episode',1,'decision',1,'chunk',1));
    [actual,out] = simulate_mpc_horizon(state,control,cfg,options);
    fields = union(fieldnames(actual),fieldnames(expected));
    fields = setdiff(fields,{'knee_template'});
    mismatchFields = fields(~cellfun(@(name) isfield(actual,name) && ...
        isfield(expected,name) && isequaln(actual.(name),expected.(name)),fields));
    dynamicEqual = isempty(mismatchFields);
    templateEqual = actual.knee_template.betaMc == expected.knee_template.betaMc;
    for name = {'tauSingle','tauSt','tauSw'}
        a = functions(actual.knee_template.(name{1}));
        b = functions(expected.knee_template.(name{1}));
        % Loaded anonymous handles have new identities; compare code and data.
        templateEqual = templateEqual && strcmp(a.function,b.function) && ...
            isequaln(a.workspace,b.workspace);
    end
    status = 'disk_restore_validation_failed';
    complete = out.completed_horizon && out.qp_solve_count == numberSteps;
    [trajectoryEqual,firstMismatch] = localCompareRows(outputRoot,archivedRows);
    if dynamicEqual && templateEqual && complete && trajectoryEqual
        status = 'disk_restore_validation_complete';
    end
    report = struct('restore_equivalence_tested',true,'run_status',status);
    report.dynamic_state_exact_match = dynamicEqual;
    report.template_code_and_captured_data_exact_match = templateEqual;
    report.source_dataset = sourceDataset;
    report.source_row_id = rowId;
    report.source_segment = target.segment;
    report.replayed_steps = out.qp_solve_count;
    report.expected_steps = numberSteps;
    report.completed_segment_remainder = complete;
    report.per_step_physics_exact_match = trajectoryEqual;
    report.first_mismatched_source_row = firstMismatch;
    report.mismatched_state_fields = mismatchFields;
    report.reference_state_present_in_source = isfield(expected,'reference_state');
    report.max_abs_Xt_difference = max(abs(actual.Xt-expected.Xt));
    report.max_abs_Ut_difference = max(abs(actual.Ut-expected.Ut));
    report.final_time_difference_s = actual.t-expected.t;
    report.source_sha = writer.Manifest.source_sha;
    report.source_dirty = writer.Manifest.source_dirty;
    save(fullfile(outputRoot,'restore_comparison.mat'),'report','-v7.3');
    writer.finish(status);
    report.validation = validate_exact_state_dataset(outputRoot);
    disp(report);
    assert(dynamicEqual && templateEqual && complete && trajectoryEqual, ...
        'run_phase3_disk_restore_validation:Mismatch', ...
        'Restored state differs from archive; inspect restore_comparison.mat.');
end

function [equal, firstMismatch] = localCompareRows(root, expected)
    index = readtable(fullfile(root,'mpc_steps','index.csv'),'TextType','string');
    % Wall-clock solver timings and new dataset identifiers are not physics.
    fields = {'time_before','time_after','state_before','state_after', ...
        'Xt_qp','Ut_qp','Xd','Ud','FSM','fsm_after','integrated', ...
        'current_sample_A','current_sample_time','current_sample_committed','action_execution'};
    count = 0;
    equal = true;
    firstMismatch = NaN;
    for k = 1:height(index)
        actual = read_exact_state_rows(fullfile(root,index.file(k)));
        for j = 1:numel(actual)
            count = count+1;
            if count > numel(expected)
                equal = false;
                return
            end
            a = actual{j};
            b = expected{count};
            if ~all(cellfun(@(name) isequaln(a.(name),b.(name)),fields)) || ...
                    ~isequaln(a.solver.z,b.solver.z)
                equal = false;
                firstMismatch = b.row_id;
                return
            end
        end
    end
    equal = count == numel(expected);
end
