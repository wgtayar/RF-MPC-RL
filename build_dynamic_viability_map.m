function result = build_dynamic_viability_map(baseStates, baseMetadata, samples, cfg, outputFolder, horizonSeconds, datasetWriter, actionBounds)
%build_dynamic_viability_map Evaluate conditional supervisory-action slices.

    if nargin < 8 || ~isa(datasetWriter,'ExactStateDataset') || ...
            ~isstruct(actionBounds) || ~all(isfield(actionBounds,{'lower','upper'}))
        error('build_dynamic_viability_map:ExactCaptureRequired', ...
            'New maps require an ExactStateDataset writer and explicit lower/upper R bounds.');
    end

    if nargin < 3 || isempty(samples)
        samples = generate_phase2_action_samples(cfg);
    end
    if nargin < 5 || strlength(string(outputFolder)) == 0
        outputFolder = fullfile(datasetWriter.Root,'analysis');
    end
    if nargin < 6 || isempty(horizonSeconds)
        horizonSeconds = 0.05;
    end
    if ~iscell(baseStates)
        error('build_dynamic_viability_map:BaseStateType', ...
            'baseStates must be a cell array of replay-state structs.');
    end
    if numel(baseStates) ~= numel(baseMetadata)
        error('build_dynamic_viability_map:MetadataCount', ...
            'Provide one metadata struct per base state.');
    end
    outputFolder = string(outputFolder);
    if isfolder(outputFolder)
        error('build_dynamic_viability_map:ExistingOutput', ...
            'Refusing to overwrite an existing map output directory.');
    end
    figureFolder = fullfile(outputFolder, 'figures');
    if ~isfolder(figureFolder)
        mkdir(figureFolder);
    end

    thresholds = localThresholds();
    numberRows = numel(baseStates)*height(samples);
    rows = repmat(localEmptyRow(), numberRows, 1);
    executions = cell(numberRows, 1);
    assessments = cell(numberRows, 1);
    contexts = repmat(localEmptyContext(), numel(baseStates), 1);
    rowIndex = 0;
    lowerR = actionBounds.lower;
    upperR = actionBounds.upper;
    actionRange = [2*cfg.DR_MAX, 2*cfg.DR_MAX, 2*cfg.DR_MAX, ...
        cfg.GAMMA_V_MAX-cfg.GAMMA_V_MIN, ...
        cfg.GAMMA_A_MAX-cfg.GAMMA_A_MIN];
    defaultReference = [-cfg.DR_MAX, -cfg.DR_MAX, cfg.DR_MAX, ...
        cfg.GAMMA_V_MAX, cfg.GAMMA_A_MIN];

    for baseIndex = 1:numel(baseStates)
        baseState = baseStates{baseIndex};
        metadata = baseMetadata(baseIndex);
        context = localContext(baseState, metadata, baseIndex);
        contexts(baseIndex) = context;
        initialMetrics = localInitialMetrics(baseState);
        referenceAction = baseState.previous_action(:).';
        if numel(referenceAction) ~= 5 || any(~isfinite(referenceAction))
            referenceAction = defaultReference;
        end
        fprintf('[viability map] base %d/%d: %s, %d actions\n', ...
            baseIndex, numel(baseStates), context.base_label, height(samples));

        for sampleIndex = 1:height(samples)
            action = [samples.dR1(sampleIndex), samples.dR2(sampleIndex), ...
                samples.dR3(sampleIndex), samples.gamma_v(sampleIndex), ...
                samples.gamma_a(sampleIndex)];
            [control, execution] = resolve_replay_action( ...
                action(:), baseState, cfg, lowerR, upperR);
            candidateR = execution.R_applied;
            vCommand = execution.v_exec;
            aCommand = execution.a_exec;
            options = struct('duration_s', horizonSeconds, ...
                'solver_strategy', "default", 'capture_trace', true, ...
                'classify_failure', true, 'update_proxy', true, ...
                'update_battery', true,'dataset_writer',datasetWriter, ...
                'dataset_context',struct('episode',(baseIndex-1)*height(samples)+sampleIndex, ...
                'decision',1,'chunk',1));
            [~, out] = simulate_mpc_horizon(baseState, control, cfg, options);
            [class,assessment] = classify_dynamic_viability_v2(initialMetrics, out, thresholds);
            rowIndex = rowIndex + 1;
            rows(rowIndex) = localResultRow(context, samples(sampleIndex, :), ...
                action, referenceAction, actionRange, candidateR, ...
                vCommand, aCommand, out, class, initialMetrics, thresholds);
            executions{rowIndex} = execution;
            assessments{rowIndex} = assessment;
        end
    end

    map = struct2table(rows);
    map = [map, supervisory_action_table(executions)];
    contextTable = struct2table(contexts);
    map.safe_action = cellfun(@(a) a.safe_for_policy,assessments);
    map.recovery_candidate = cellfun(@(a) a.recovery_candidate,assessments);
    map.within_envelope_at_horizon = cellfun(@(a) a.within_envelope_at_horizon,assessments);
    map.minimum_health_margin = cellfun(@(a) a.minimum_health_margin,assessments);
    map.horizon_seconds = repmat(horizonSeconds,height(map),1);
    map.thresholds_validated = false(height(map),1);
    volume = groupsummary(map, {'base_id','base_label','sample_type','profile'}, ...
        'mean', 'safe_action');
    volume.Properties.VariableNames{end} = 'safe_action_fraction';
    provenance = struct('source_git_sha', string(datasetWriter.Manifest.source_sha), ...
        'matlab_version', string(version), 'created_at', ...
        string(datetime('now', 'TimeZone', 'local')), ...
        'rng_seed', cfg.RNG_SEED, 'horizon_seconds', horizonSeconds, ...
        'reward_version', string(cfg.REWARD.version));
    provenance.action_schema_version = "action_v2_rate_limited";
    provenance.viability_schema_version = "viability_v2_whole_horizon";
    provenance.thresholds_validated = false;
    writetable(map, fullfile(outputFolder, 'dynamic_viability_samples.csv'));
    writetable(contextTable, fullfile(outputFolder, 'conditioning_states.csv'));
    writetable(volume, fullfile(outputFolder, 'safe_action_volume.csv'));
    save(fullfile(outputFolder, 'dynamic_viability_map.mat'), ...
        'map', 'contextTable', 'volume', 'baseStates', 'baseMetadata', ...
        'samples', 'thresholds', 'horizonSeconds', 'provenance', 'executions', 'assessments', '-v7.3');
    localCreateFigures(map, contextTable, figureFolder);
    result = struct('map', map, 'conditioning_states', contextTable, ...
        'safe_action_volume', volume, 'thresholds', thresholds, ...
        'horizon_seconds', horizonSeconds, 'provenance', provenance, ...
        'output_folder', outputFolder);
end

function row = localResultRow(context, sample, action, referenceAction, ...
        actionRange, R, vCommand, aCommand, out, class, initial, thresholds)
    row = localEmptyRow();
    row.base_id = context.base_id;
    row.base_label = context.base_label;
    row.policy_source = context.policy_source;
    row.conditioning_time_s = context.time_s;
    row.conditioning_soc_pct = context.soc_pct;
    row.conditioning_orientation_error_rad = context.orientation_error_rad;
    row.conditioning_angular_velocity = context.angular_velocity;
    row.conditioning_position_invariant_norm = ...
        context.position_invariant_norm;
    row.conditioning_Ut_norm = context.Ut_norm;
    row.conditioning_FSM = context.FSM;
    row.action_id = sample.action_id;
    row.sample_type = sample.sample_type;
    row.profile = sample.profile;
    row.dR1 = action(1);
    row.dR2 = action(2);
    row.dR3 = action(3);
    row.gamma_v = action(4);
    row.gamma_a = action(5);
    row.R1 = R(1);
    row.R2 = R(2);
    row.R3 = R(3);
    row.v_cmd = vCommand;
    row.a_cmd = aCommand;
    row.normalized_action_distance = norm((action-referenceAction)./actionRange);
    row.survived_horizon_s = out.survived_duration_s;
    row.solver_failure_count = out.qp_failed_count;
    row.terminal_class = out.terminal_reason;
    row.viability_class = class;
    row.solver_iterations_max = max(out.trace.solver_iterations, [], 'omitnan');
    row.solver_wall_time_max_s = max(out.trace.solver_wall_time_s, [], 'omitnan');
    row.inequality_margin_min = min(out.trace.inequality_margin_min, [], 'omitnan');
    row.equality_residual_max = max( ...
        out.trace.equality_residual_max_abs, [], 'omitnan');
    row.kkt_stationarity_max = max( ...
        out.trace.kkt_stationarity_inf, [], 'omitnan');
    valid = find(isfinite(out.trace.orientation_error_after_rad), 1, 'last');
    if ~isempty(valid)
        last = out.trace(valid, :);
        row.final_orientation_error_rad = last.orientation_error_after_rad;
        row.final_angular_velocity = last.angular_velocity_after;
        row.final_com_velocity = last.com_velocity_after;
        row.final_position_invariant_norm = ...
            last.position_invariant_norm_after;
        row.final_Ut_norm = last.Ut_norm_after;
    end
    before = out.trace{:,{'orientation_error_before_rad','angular_velocity_before', ...
        'position_invariant_norm_before','Ut_norm_before'}};
    after = out.trace{:,{'orientation_error_after_rad','angular_velocity_after', ...
        'position_invariant_norm_after','Ut_norm_after'}};
    health = [initial.orientation,initial.angular_velocity,initial.position_invariant,initial.Ut; ...
        before;after];
    if all(isfinite(health(:)))
        normalized = health./[thresholds.orientation,thresholds.angular_velocity, ...
            thresholds.position_invariant,thresholds.Ut];
        row.risk_score = max([normalized(:); ...
            row.solver_iterations_max/thresholds.solver_iterations; ...
            row.solver_wall_time_max_s/out.mpc_timestep_s]);
    end
end

function metrics = localInitialMetrics(state)
    components = decompose_srb_state(state.Xt);
    metrics = struct('orientation', components.orientation_error_rad, ...
        'angular_velocity', components.angular_velocity_norm, ...
        'position_invariant', components.position_invariant_state_norm, ...
        'Ut', norm(state.Ut));
end

function context = localContext(state, metadata, baseIndex)
    components = decompose_srb_state(state.Xt);
    context = localEmptyContext();
    context.base_id = baseIndex;
    context.base_label = string(metadata.base_label);
    context.policy_source = string(metadata.policy_source);
    context.decision = localMetadata(metadata, 'decision', NaN);
    context.time_s = state.t;
    context.soc_pct = state.battery.soc_pct;
    context.orientation_error_rad = components.orientation_error_rad;
    context.angular_velocity = components.angular_velocity_norm;
    context.com_velocity = components.linear_velocity_norm;
    context.position_invariant_norm = ...
        components.position_invariant_state_norm;
    context.Ut_norm = norm(state.Ut);
    if isfield(metadata, 'FSM')
        fsm = metadata.FSM;
    elseif isfield(state.fsm_internal_state, 'FSM')
        fsm = state.fsm_internal_state.FSM;
    else
        fsm = nan(4, 1);
    end
    context.FSM = join(string(fsm(:).'), '');
    if isfield(state.fsm_internal_state, 'Ta') && ...
            isfield(state.fsm_internal_state, 'Tb')
        phase = (state.t-state.fsm_internal_state.Ta(:))./ ...
            (state.fsm_internal_state.Tb(:)-state.fsm_internal_state.Ta(:));
        context.phase = join(compose('%.4f', phase), ',');
    end
end

function value = localMetadata(metadata, field, defaultValue)
    if isfield(metadata, field)
        value = metadata.(field);
    else
        value = defaultValue;
    end
end

function thresholds = localThresholds()
    thresholds = struct('orientation', 1, 'angular_velocity', 10, ...
        'position_invariant', 10, 'Ut', 100, ...
        'solver_iterations', 100, 'equality_residual', 1e-6, ...
        'inequality_margin', -1e-6, 'kkt_stationarity', 1e-4);
end

function localCreateFigures(map, contexts, figureFolder)
    for baseId = contexts.base_id.'
        context = contexts(contexts.base_id == baseId, :);
        baseRows = map(map.base_id == baseId, :);
        gammaRows = baseRows(baseRows.sample_type == "gamma_heatmap", :);
        profiles = unique(gammaRows.profile, 'stable');
        if isempty(profiles)
            gammaRows = baseRows;
            gammaRows.profile(:) = "all_candidates";
            profiles = "all_candidates";
        end
        figureHandle = figure('Visible', 'off', 'Color', 'w', ...
            'Position', [100, 100, 1150, 380]);
        cleanupObj = onCleanup(@() close(figureHandle));
        tiledlayout(1, numel(profiles), 'TileSpacing', 'compact');
        for i = 1:numel(profiles)
            rows = gammaRows(gammaRows.profile == profiles(i), :);
            nexttile;
            scatter(rows.gamma_v, rows.gamma_a, 90, rows.risk_score, 'filled');
            colorbar;
            colorMaximum = max(rows.risk_score(isfinite(rows.risk_score)));
            if isempty(colorMaximum)
                colorMaximum = 2;
            end
            clim([0,max(2,colorMaximum)]);
            xlabel('raw candidate \gamma_v');
            ylabel('raw candidate \gamma_a');
            title(strrep(profiles(i), '_', ' '));
        end
        titleText = sprintf('%s | t=%.3f SOC=%.2f%% omega=%.2f orient=%.2f Ut=%.2f FSM=%s | h=%g s n=%d | dynamic_state_v2 provisional', ...
            context.base_label, context.time_s, context.soc_pct, ...
            context.angular_velocity, context.orientation_error_rad, ...
            context.Ut_norm, context.FSM,baseRows.horizon_seconds(1),height(baseRows));
        sgtitle(titleText, 'Interpreter', 'none');
        exportgraphics(figureHandle, fullfile(figureFolder, ...
            sprintf('base_%02d_gamma_slices.png', baseId)), 'Resolution', 160);
        clear cleanupObj

        figureHandle = figure('Visible', 'off', 'Color', 'w');
        scatter(baseRows.normalized_action_distance, baseRows.risk_score, ...
            35, double(baseRows.safe_action), 'filled');
        grid on;
        xlabel('normalized distance from conditioning policy action');
        ylabel('provisional normalized stress (not probability)');
        title(titleText, 'Interpreter', 'none');
        exportgraphics(figureHandle, fullfile(figureFolder, ...
            sprintf('base_%02d_risk_distance.png', baseId)), 'Resolution', 160);
        close(figureHandle);
    end
end

function row = localEmptyRow()
    row = struct('base_id', NaN, 'base_label', "", 'policy_source', "", ...
        'conditioning_time_s', NaN, 'conditioning_soc_pct', NaN, ...
        'conditioning_orientation_error_rad', NaN, ...
        'conditioning_angular_velocity', NaN, ...
        'conditioning_position_invariant_norm', NaN, ...
        'conditioning_Ut_norm', NaN, 'conditioning_FSM', "", ...
        'action_id', NaN, 'sample_type', "", 'profile', "", ...
        'dR1', NaN, 'dR2', NaN, 'dR3', NaN, ...
        'gamma_v', NaN, 'gamma_a', NaN, ...
        'R1', NaN, 'R2', NaN, 'R3', NaN, 'v_cmd', NaN, 'a_cmd', NaN, ...
        'normalized_action_distance', NaN, 'survived_horizon_s', NaN, ...
        'solver_failure_count', NaN, 'terminal_class', "", ...
        'viability_class', "", 'solver_iterations_max', NaN, ...
        'solver_wall_time_max_s', NaN, 'inequality_margin_min', NaN, ...
        'equality_residual_max', NaN, 'kkt_stationarity_max', NaN, ...
        'final_orientation_error_rad', NaN, ...
        'final_angular_velocity', NaN, 'final_com_velocity', NaN, ...
        'final_position_invariant_norm', NaN, 'final_Ut_norm', NaN, ...
        'risk_score', NaN);
end

function context = localEmptyContext()
    context = struct('base_id', NaN, 'base_label', "", ...
        'policy_source', "", 'decision', NaN, 'time_s', NaN, ...
        'soc_pct', NaN, 'orientation_error_rad', NaN, ...
        'angular_velocity', NaN, 'com_velocity', NaN, ...
        'position_invariant_norm', NaN, 'Ut_norm', NaN, ...
        'FSM', "", 'phase', "");
end
