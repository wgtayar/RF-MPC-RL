function result = run_reference_continuity_ablation(runRoot,referenceMode,scenario,decisionLimit)
%RUN_REFERENCE_CONTINUITY_ABLATION Run one scripted, captured energy mission.
%   runRoot is a new directory below the local artifact store. Scenarios are
%   staircase95, staircase45, and constant95. A smaller decisionLimit is
%   useful for a smoke check and is recorded separately from a terminal run.
%   longwalk95 and longwalk60 (monitor #43, WGT 21 Nov 2026): one continuous
%   mission of 1280 m in 2400 s (four standard missions without stopping),
%   the battery draining normally, with the speed command switched at every
%   320 m boundary (1.0, 0.85, 1.0, 0.85 m/s), as a charge-budgeting plan
%   would. Pass a decisionLimit of at least 480 for the full walk.
    arguments
        runRoot (1,:) char
        referenceMode (1,:) char {mustBeMember(referenceMode, ...
            {'legacy_absolute_time','velocity_continuous_v2'})}
        scenario (1,:) char {mustBeMember(scenario,{'staircase95','staircase45','constant95','longwalk95','longwalk60'})}
        decisionLimit (1,1) double {mustBeInteger,mustBePositive} = 120
    end

    source = bootstrap_RF_MPC_RL();
    sourceSha = cleanSourceSha(source);
    workspace = fileparts(source);
    artifactRoot = fullfile(workspace,'BeyondTorque-Local-Artifacts');
    runRoot = char(java.io.File(runRoot).getCanonicalPath());
    assert(isfile(fullfile(artifactRoot,'LOCAL_ONLY_ARTIFACT_STORE.json')) && ...
        startsWith(runRoot,[char(java.io.File(artifactRoot).getCanonicalPath()) filesep]) && ...
        ~isfolder(runRoot) && ~isfile(runRoot), ...
        'referenceAblation:RunRoot','Use a new run directory in the local artifact store.');

    initialSoc = .95;
    speeds = [.533 .733 .933 .733 .533];
    if strcmp(scenario,'staircase45')
        initialSoc = .45;
    elseif strcmp(scenario,'constant95')
        speeds = .8;
    end
    holdSeconds = 60;
    setup = struct('initial_soc',initialSoc);
    segmentMetres = Inf;
    if startsWith(scenario,'longwalk')
        initialSoc = str2double(scenario(end-1:end))/100;
        speeds = [1.0 .85 1.0 .85];
        segmentMetres = 320;
        setup = struct('initial_soc',initialSoc,'target_distance_m',numel(speeds)*segmentMetres, ...
            'mission_duration_s',numel(speeds)*600);
    end
    bundle = setup_energy_mission_rl(setup);
    cfg = bundle.cfg;
    gamma = (speeds-cfg.V_MIN)/(cfg.V_MAX-cfg.V_MIN);
    % Initialize the governor at the first target. In the constant control
    % this prevents an unintended second target during launch.
    bundle.cfg.GAMMA_V_MISSION = gamma(1);
    [~,runId] = fileparts(runRoot);
    metadata = struct('run_id',runId,'run_type','scripted_reference_ablation', ...
        'monitor_root',fullfile(workspace,'RL-MPC-Monitor'), ...
        'source_policy',scenario,'reference_mode',referenceMode, ...
        'initial_soc',initialSoc,'requested_speed_schedule_m_s',speeds, ...
        'hold_seconds',holdSeconds,'segment_metres',segmentMetres,'decision_limit',decisionLimit, ...
        'training',false,'learning_rates_used',[],'exploration_noise_used',[]);
    raw = fullfile(runRoot,'raw');
    timer = tic;
    env = Phase3MpcEnvironment(bundle,raw,metadata,struct('reference_mode',referenceMode));
    peakTilt = NaN;
    peakBodyRate = NaN;
    tiltExceedanceSteps = 0;
    recovered = 0;
    appliedSpeeds = zeros(0,1);
    endReason = 'probe_decision_limit';
    decisionSeconds = cfg.CHUNK_DURATION*cfg.APPLY_EVERY;

    distance = 0;
    segmentLog = zeros(0,3);
    try
        reset(env);
        while ~env.Done && env.Decision < decisionLimit
            if isfinite(segmentMetres)
                % Speed command switches at each segment boundary crossed.
                index = min(1+floor(distance/segmentMetres),numel(speeds));
            else
                index = 1+mod(floor(env.Decision*decisionSeconds/holdSeconds),numel(speeds));
            end
            [~,~,isDone,info] = step(env,[zeros(3,1);gamma(index);cfg.GAMMA_A_MIN]);
            assert(abs(info.window.v_exec-speeds(index)) < 1e-9, ...
                'referenceAblation:Speed','The governor did not apply the scripted speed.');
            appliedSpeeds(end+1,1) = info.window.v_exec; %#ok<AGROW>
            distance = info.window.distance_end_m;
            segmentLog(end+1,:) = [env.State.t distance env.State.energy.soc]; %#ok<AGROW>
            recovered = recovered+info.window.recovered_solver_events;
            attitude = info.window.attitude;
            if attitude.available
                peakTilt = max(peakTilt,attitude.max_tilt_rad);
                peakBodyRate = max(peakBodyRate,attitude.max_body_rate_rad_s);
                tiltExceedanceSteps = tiltExceedanceSteps+ ...
                    round(attitude.tilt_exceedance_fraction*attitude.integrated_steps);
            end
            if isDone
                endReason = info.terminal_reason;
            end
            fprintf('decision=%d t=%.2f distance=%.4f speed=%.3f terminal=%s\n', ...
                env.Decision,env.State.t,info.window.distance_end_m,info.window.v_exec,info.terminal_reason);
        end
        close(env,endReason);
    catch exception
        if ~env.Writer.Closed
            close(env,'reference_ablation_exception');
        end
        rethrow(exception);
    end

    validation = validate_exact_state_dataset(raw);
    assert(validation.valid && strcmp(cleanSourceSha(source),sourceSha), ...
        'referenceAblation:Validation','Dataset invalid or source changed during the run.');
    metrics = phase3_episode_metrics(raw,env.Config,env.Episode);
    charge = metrics.sampled_charge_in_scope_As;
    chargePerMetre = NaN;
    if metrics.distance_m > 0
        chargePerMetre = charge/metrics.distance_m;
    end
    manifest = env.Writer.Manifest;
    result = struct('run_id',runId,'scenario',scenario,'reference_mode',referenceMode, ...
        'source_sha',sourceSha,'monitor_sha',manifest.monitor_sha, ...
        'configuration_sha256',manifest.configuration_sha256,'matlab_version',version, ...
        'seed',manifest.seed,'reward_version',manifest.reward_version, ...
        'solver_strategy',manifest.solver_strategy,'initial_soc',initialSoc, ...
        'end_reason',endReason,'time_s',metrics.scope_duration_s, ...
        'distance_m',metrics.distance_m,'qp_solves',metrics.qp_solves, ...
        'failed_qps',metrics.failed_qps,'mathematical_infeasibilities',metrics.mathematical_infeasibilities, ...
        'numerical_events',metrics.numerical_events,'recovered_solver_events',recovered, ...
        'peak_tilt_rad',peakTilt,'time_above_025_rad_s',tiltExceedanceSteps*metrics.mpc_timestep_s, ...
        'peak_body_rate_rad_s',peakBodyRate,'charge_As',charge,'charge_per_m_As',chargePerMetre, ...
        'applied_speed_m_s',appliedSpeeds,'decision_limit',decisionLimit, ...
        'segment_metres',segmentMetres,'segments',localSegments(segmentLog,segmentMetres), ...
        'final_true_soc',env.State.energy.soc,'final_estimated_soc',env.State.energy.estimate(1), ...
        'dataset_validated',true,'wall_seconds',toc(timer));
    analysisDir = fullfile(runRoot,'analysis');
    mkdir(analysisDir);
    file = fopen(fullfile(analysisDir,'metrics.json'),'w');
    assert(file >= 0,'referenceAblation:MetricsFile','Could not open the metrics file.');
    fileCleanup = onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(result,'PrettyPrint',true));
end

function sha = cleanSourceSha(source)
    [gitStatus,dirty] = git_output(source,'status --porcelain');
    [shaStatus,sha] = git_output(source,'rev-parse HEAD');
    assert(gitStatus == 0 && shaStatus == 0 && isempty(strtrim(dirty)), ...
        'referenceAblation:Dirty','Run from a clean Git commit.');
    sha = strtrim(sha);
end

function segments = localSegments(log,segmentMetres)
% Time and SOC at which each segment boundary was crossed (long walks only).
    segments = struct('boundary_m',{},'time_s',{},'true_soc',{});
    if ~isfinite(segmentMetres) || isempty(log), return, end
    for k = 1:floor(log(end,2)/segmentMetres)
        row = find(log(:,2) >= k*segmentMetres,1);
        segments(end+1) = struct('boundary_m',k*segmentMetres,'time_s',log(row,1), ...
            'true_soc',log(row,3)); %#ok<AGROW>
    end
end
