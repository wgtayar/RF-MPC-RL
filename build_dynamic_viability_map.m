function result = build_dynamic_viability_map(baseStates, baseMetadata, samples, cfg, outputFolder, horizonSeconds)
%build_dynamic_viability_map Evaluate conditional supervisory-action slices.

    if nargin < 3 || isempty(samples)
        samples = generate_phase2_action_samples(cfg);
    end
    if nargin < 5 || strlength(string(outputFolder)) == 0
        outputFolder = fullfile(fileparts(mfilename('fullpath')), ...
            'Phase 2 Outputs', 'dynamic_viability');
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
    figureFolder = fullfile(outputFolder, 'figures');
    if ~isfolder(figureFolder)
        mkdir(figureFolder);
    end

    thresholds = localThresholds();
    numberRows = numel(baseStates)*height(samples);
    rows = repmat(localEmptyRow(), numberRows, 1);
    contexts = repmat(localEmptyContext(), numel(baseStates), 1);
    rowIndex = 0;
    p = get_params(0);
    nominalR = diag(p.R);
    nominalR = nominalR(1:3);
    lowerR = 0.95*nominalR;
    upperR = 1.05*nominalR;
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
            candidateR = baseState.R(:).*(1 + action(1:3).');
            candidateR = min(max(candidateR, lowerR), upperR);
            [vCommand, aCommand] = apply_command_governor( ...
                cfg.V_REQ_FIXED, cfg.A_REQ_FIXED, action(4), action(5), cfg);
            control = struct('R', candidateR, 'v_cmd', vCommand, ...
                'a_cmd', aCommand, 'action', action(:));
            options = struct('duration_s', horizonSeconds, ...
                'solver_strategy', "default", 'capture_trace', true, ...
                'classify_failure', true, 'update_proxy', true, ...
                'update_battery', false);
            [~, out] = simulate_mpc_horizon(baseState, control, cfg, options);
            class = classify_dynamic_viability(initialMetrics, out, thresholds);
            rowIndex = rowIndex + 1;
            rows(rowIndex) = localResultRow(context, samples(sampleIndex, :), ...
                action, referenceAction, actionRange, candidateR, ...
                vCommand, aCommand, out, class, initialMetrics, thresholds);
        end
    end

    map = struct2table(rows);
    contextTable = struct2table(contexts);
    safeMask = ismember(map.viability_class, [ ...
        "healthy_robust_solver", "healthy_solver_stressed", ...
        "degraded_recoverable"]);
    map.safe_action = safeMask;
    volume = groupsummary(map, {'base_id','base_label','sample_type','profile'}, ...
        'mean', 'safe_action');
    volume.Properties.VariableNames{end} = 'safe_action_fraction';
    provenance = struct('source_git_sha', localGitSha(), ...
        'matlab_version', string(version), 'created_at', ...
        string(datetime('now', 'TimeZone', 'local')), ...
        'rng_seed', cfg.RNG_SEED, 'horizon_seconds', horizonSeconds, ...
        'reward_version', string(cfg.REWARD.version));
    writetable(map, fullfile(outputFolder, 'dynamic_viability_samples.csv'));
    writetable(contextTable, fullfile(outputFolder, 'conditioning_states.csv'));
    writetable(volume, fullfile(outputFolder, 'safe_action_volume.csv'));
    save(fullfile(outputFolder, 'dynamic_viability_map.mat'), ...
        'map', 'contextTable', 'volume', 'baseStates', 'baseMetadata', ...
        'samples', 'thresholds', 'horizonSeconds', 'provenance', '-v7.3');
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
        finalScore = max([ ...
            row.final_orientation_error_rad/thresholds.orientation, ...
            row.final_angular_velocity/thresholds.angular_velocity, ...
            row.final_position_invariant_norm/thresholds.position_invariant, ...
            row.final_Ut_norm/thresholds.Ut]);
    else
        finalScore = 5;
    end
    initialScore = max([initial.orientation/thresholds.orientation, ...
        initial.angular_velocity/thresholds.angular_velocity, ...
        initial.position_invariant/thresholds.position_invariant, ...
        initial.Ut/thresholds.Ut]);
    solverStress = max(row.solver_iterations_max/thresholds.solver_iterations, 0);
    row.risk_score = max([finalScore, 0.5*initialScore, solverStress]);
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
        figureHandle = figure('Visible', 'off', 'Color', 'w', ...
            'Position', [100, 100, 1150, 380]);
        cleanupObj = onCleanup(@() close(figureHandle));
        tiledlayout(1, numel(profiles), 'TileSpacing', 'compact');
        for i = 1:numel(profiles)
            rows = gammaRows(gammaRows.profile == profiles(i), :);
            nexttile;
            scatter(rows.gamma_v, rows.gamma_a, 90, rows.risk_score, 'filled');
            colorbar;
            clim([0, max(2, max(rows.risk_score, [], 'omitnan'))]);
            xlabel('\gamma_v');
            ylabel('\gamma_a');
            title(strrep(profiles(i), '_', ' '));
        end
        titleText = sprintf('%s | t=%.3f SOC=%.2f%% omega=%.2f orient=%.2f Ut=%.2f FSM=%s', ...
            context.base_label, context.time_s, context.soc_pct, ...
            context.angular_velocity, context.orientation_error_rad, ...
            context.Ut_norm, context.FSM);
        sgtitle(titleText, 'Interpreter', 'none');
        exportgraphics(figureHandle, fullfile(figureFolder, ...
            sprintf('base_%02d_gamma_slices.png', baseId)), 'Resolution', 160);
        clear cleanupObj

        figureHandle = figure('Visible', 'off', 'Color', 'w');
        scatter(baseRows.normalized_action_distance, baseRows.risk_score, ...
            35, double(baseRows.safe_action), 'filled');
        grid on;
        xlabel('normalized distance from conditioning policy action');
        ylabel('continuation risk score');
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

function sha = localGitSha()
    [status, text] = system('git rev-parse HEAD');
    if status == 0
        sha = string(strtrim(text));
    else
        sha = "unknown";
    end
end
