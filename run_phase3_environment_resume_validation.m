function report = run_phase3_environment_resume_validation(sourceDataset,outputRoot)
%run_phase3_environment_resume_validation Reproduce decision 2 after disk resume.
    source = bootstrap_RF_MPC_RL();
    saved = load(fullfile(sourceDataset,'manifest','configuration.mat'),'cfg');
    cfg = saved.cfg;
    bundle = struct('cfg',cfg,'initial_R',cfg.PHASE3.initial_R, ...
        'lower_abs',cfg.PHASE3.lower_R,'upper_abs',cfg.PHASE3.upper_R);
    expected = load(fullfile(sourceDataset,'decisions','episode_000001_decision_000002.mat'),'event');
    [~,runId] = fileparts(outputRoot);
    metadata = struct('run_id',runId,'run_type','environment_disk_resume_validation', ...
        'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'),'seed',cfg.RNG_SEED, ...
        'source_policy','archived_decision_2_candidate','source_dataset',sourceDataset);
    env = Phase3MpcEnvironment(bundle,outputRoot,metadata,cfg.PHASE3.options);
    initialObservation = resumeFromDecision(env,sourceDataset,1,1);
    [observation,reward,done,actual] = step(env,expected.event.record.action_execution.candidate_action);
    reference = expected.event.record;
    dynamicEqual = isequaln(rmfield(actual.state,'knee_template'),rmfield(reference.state,'knee_template'));
    templateEqual = actual.state.knee_template.betaMc == reference.state.knee_template.betaMc;
    for name = {'tauSingle','tauSt','tauSw'}
        a = functions(actual.state.knee_template.(name{1}));
        b = functions(reference.state.knee_template.(name{1}));
        templateEqual = templateEqual && strcmp(a.function,b.function) && isequaln(a.workspace,b.workspace);
    end
    report = struct('source_sha',env.Writer.Manifest.source_sha,'source_dirty',env.Writer.Manifest.source_dirty, ...
        'source_dataset',sourceDataset,'initial_observation',initialObservation, ...
        'dynamic_state_exact',dynamicEqual,'template_code_and_data_exact',templateEqual, ...
        'observation_exact',isequaln(observation,reference.next_observation), ...
        'reward_exact',isequaln(reward,reference.reward),'terminal_exact',isequaln(done,reference.is_done), ...
        'action_mapping_exact',isequaln(actual.action_execution,reference.action_execution), ...
        'window_exact',isequaln(actual.window,reference.window),'qp_steps',env.Writer.RowCount);
    passed = all([report.dynamic_state_exact,report.template_code_and_data_exact, ...
        report.observation_exact,report.reward_exact,report.terminal_exact, ...
        report.action_mapping_exact,report.window_exact]);
    report.passed = passed;
    save(fullfile(outputRoot,'environment_resume_comparison.mat'),'report','-v7.3');
    status = 'environment_resume_failed';
    if passed
        status = 'environment_resume_complete';
    end
    close(env,status);
    report.validation = validate_exact_state_dataset(outputRoot);
    disp(report);
    assert(passed,'run_phase3_environment_resume_validation:Mismatch', ...
        'Disk-resumed environment differs; inspect preserved comparison report.');
end
