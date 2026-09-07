function report = run_phase3_disk_restore_validation(sourceDataset, outputRoot)
%run_phase3_disk_restore_validation Repeat archived MPC rows from disk state.
    source = bootstrap_RF_MPC_RL();
    config = load(fullfile(sourceDataset,'manifest','configuration.mat'),'cfg');
    cfg = config.cfg;
    [state,control,p] = restore_exact_dataset_state(sourceDataset,6);
    metadata = struct('run_id','disk_restore_validation','run_type','exact_state_validation', ...
        'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',cfg.RNG_SEED, ...
        'observation_schema_version','observation_v1_legacy','reward_version',cfg.REWARD.version, ...
        'solver_strategy','default','source_policy','archived_rows_6_to_8');
    writer = ExactStateDataset(outputRoot,cfg,metadata);
    options = struct('duration_s',3*p.simTimeStep,'capture_trace',true, ...
        'update_battery',true,'dataset_writer',writer, ...
        'dataset_context',struct('episode',1,'decision',1,'chunk',1));
    [actual,out] = simulate_mpc_horizon(state,control,cfg,options);
    saved = load(fullfile(sourceDataset,'snapshots','segment_000002_after.mat'),'snapshot');
    expected = saved.snapshot.state;
    assert(out.completed_horizon && out.qp_solve_count == 3, ...
        'run_phase3_disk_restore_validation:Continuation','Disk continuation did not complete.');
    dynamicEqual = isequaln(rmfield(actual,'knee_template'),rmfield(expected,'knee_template'));
    templateEqual = actual.knee_template.betaMc == expected.knee_template.betaMc;
    for name = {'tauSingle','tauSt','tauSw'}
        a = functions(actual.knee_template.(name{1}));
        b = functions(expected.knee_template.(name{1}));
        % Loaded anonymous handles have new identities; compare code and data.
        templateEqual = templateEqual && strcmp(a.function,b.function) && ...
            isequaln(a.workspace,b.workspace);
    end
    status = 'disk_restore_validation_failed';
    if dynamicEqual && templateEqual
        status = 'disk_restore_validation_complete';
    end
    writer.finish(status);
    assert(dynamicEqual && templateEqual, ...
        'run_phase3_disk_restore_validation:Mismatch', ...
        'Restored dynamic state or captured template code/data differ from archive.');
    report = validate_exact_state_dataset(outputRoot);
    report.restore_equivalence_tested = true;
    report.dynamic_state_exact_match = dynamicEqual;
    report.template_code_and_captured_data_exact_match = templateEqual;
    report.source_dataset = sourceDataset;
    disp(report);
end
