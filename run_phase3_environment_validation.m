function report = run_phase3_environment_validation(outputRoot, shortProbe, observationSchema)
%run_phase3_environment_validation Exercise two captured RL decisions and reset.
% Default uses genuine 50-second decisions. shortProbe=true includes two
% 10-step decisions plus MATLAB validateEnvironment's captured 10-step probe.
    if nargin < 2
        shortProbe = false;
    end
    if nargin < 3
        observationSchema = 'observation_v1_legacy';
    end
    source = bootstrap_RF_MPC_RL();
    bundle = load(fullfile(source,'rlEnv_MPC_R.mat'),'cfg','initial_R','lower_abs','upper_abs');
    previousRng = rng;
    rngCleanup = onCleanup(@() rng(previousRng));
    rng(bundle.cfg.RNG_SEED,'twister');
    if shortProbe
        bundle.cfg.CHUNK_DURATION = 0.05;
        bundle.cfg.APPLY_EVERY = 2;
        bundle.cfg.MISSION_DURATION = 0.2;
        bundle.cfg.EP_STEPS = 2;
        bundle.cfg.MISSION.WINDOW_TARGET_M = bundle.cfg.MISSION.D_TARGET_M/2;
    end
    protected = {'rlEnv_MPC_R.mat','LastR.mat','SimSnapshot_RL.mat'};
    beforeHashes = localHashes(source,protected);
    [~,runId] = fileparts(outputRoot);
    metadata = struct('run_id',runId,'run_type','captured_environment_validation', ...
        'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'), ...
        'seed',bundle.cfg.RNG_SEED,'source_policy','mission_average_then_legal_speed_increase');
    options = struct('reference_mode','position_continuous_v1','solver_strategy','default');
    expectedDimension = 19;
    if ~strcmp(observationSchema,'observation_v1_legacy')
        options.observation_schema = observationSchema;
        schema = phase3_observation_v2_schema(bundle.cfg);
        expectedDimension = schema.dimension;
    end
    env = Phase3MpcEnvironment(bundle,outputRoot,metadata,options);
    initialObservation = reset(env);
    initialState = env.State;
    candidates = {[zeros(3,1);bundle.cfg.GAMMA_V_MISSION;bundle.cfg.GAMMA_A_MIN], ...
        [zeros(3,1);bundle.cfg.GAMMA_V_MAX;bundle.cfg.GAMMA_A_MIN]};
    transitions = cell(2,1);
    for decision = 1:2
        previous = env.State.supervisory_state;
        expected = resolve_supervisory_action(candidates{decision},previous, ...
            bundle.cfg,bundle.lower_abs,bundle.upper_abs);
        [observation,reward,done,info] = step(env,candidates{decision});
        assert(isequaln(expected,info.action_execution), ...
            'run_phase3_environment_validation:Action','Environment action transform differs.');
        assert(isequal(size(observation),[expectedDimension,1]) && all(isfinite(observation)) && isfinite(reward), ...
            'run_phase3_environment_validation:Observation','Transition does not satisfy RL interface.');
        transitions{decision} = info;
        fprintf('Captured decision %d: t=%.2f, distance=%.5f, QPs=%d, terminal=%s\n', ...
            decision,env.State.t,info.window.distance_end_m,info.window.qp_solve_count,info.terminal_reason);
        if done && decision < 2
            error('run_phase3_environment_validation:EarlyTerminal','First decision terminated unexpectedly.');
        end
    end
    finalState = env.State;
    resetObservation = reset(env);
    assert(isequaln(initialObservation,resetObservation) && ...
        isequaln(initialState.Xt,env.State.Xt) && isempty(env.State.current_time) && ...
        isequal(env.State.R,bundle.initial_R(:)) && env.Episode == 2 && env.Decision == 0, ...
        'run_phase3_environment_validation:Reset','Episode state or observation leaked across reset.');
    if shortProbe
        validateEnvironment(env);
    end
    afterHashes = localHashes(source,protected);
    assert(isequal(beforeHashes,afterHashes), ...
        'run_phase3_environment_validation:SharedFiles','Legacy shared runtime files changed.');
    report = struct('source_sha',env.Writer.Manifest.source_sha, ...
        'source_dirty',env.Writer.Manifest.source_dirty,'short_probe',shortProbe,'observation_schema',observationSchema, ...
        'initial_observation',initialObservation,'transitions',{transitions}, ...
        'final_state',finalState,'reset_equal',true,'shared_file_hashes',{afterHashes}, ...
        'legacy_shared_files_unchanged',true,'action_mapping_exact',true, ...
        'qp_steps',env.Writer.RowCount,'toolbox_validation_executed',shortProbe,'training_promoted',false);
    save(fullfile(outputRoot,'environment_validation.mat'),'report','-v7.3');
    close(env,'environment_validation_complete');
    report.validation = validate_exact_state_dataset(outputRoot);
    disp(rmfield(report,{'transitions','final_state','initial_observation','shared_file_hashes'}));
end

function hashes = localHashes(root,names)
    hashes = cell(size(names));
    for k = 1:numel(names)
        path = fullfile(root,names{k});
        hashes{k} = 'absent';
        if isfile(path)
            hashes{k} = sha256_file(path);
        end
    end
end
