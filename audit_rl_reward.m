function audit = audit_rl_reward(runDir, outputFolder)
%audit_rl_reward Audit the active reward without modifying reward semantics.

    runDir = string(runDir);
    if nargin < 2 || strlength(string(outputFolder)) == 0
        outputFolder = fullfile(fileparts(mfilename('fullpath')), ...
            'Phase 2 Outputs', 'reward_audit');
    end
    if ~isfolder(outputFolder)
        mkdir(outputFolder);
    end
    cfgData = load(localSingleFile(runDir, 'cfg_snapshot_*.mat'), 'cfg');
    cfg = cfgData.cfg;
    decisions = readtable(localSingleFile(runDir, 'rl_decisions_*.csv'), ...
        TextType='string');
    numberRows = height(decisions);
    decomposition = repmat(localEmptyDecomposition(), numberRows, 1);
    counterfactual = repmat(localEmptyCounterfactual(), numberRows, 1);
    counterfactualCount = 0;

    for i = 1:numberRows
        window = localWindow(decisions(i, :));
        [recomputed, info] = compute_rl_reward(window, cfg);
        nonterminalWindow = window;
        nonterminalWindow.terminal_reason = '';
        nonterminalReward = compute_rl_reward(nonterminalWindow, cfg);
        decomposition(i) = localDecompositionRow( ...
            decisions(i, :), recomputed, nonterminalReward, info, cfg);
        if string(window.terminal_reason) == "infeasible"
            counterfactualCount = counterfactualCount + 1;
            row = localEmptyCounterfactual();
            row.episode_idx = decisions.episode_idx(i);
            row.decision_idx = decisions.decision_idx(i);
            row.failure_qp_id = decisions.failure_qp_id(i);
            row.logged_terminal_label = "infeasible";
            row.evidence_class = "numerical_solver_failure";
            row.logged_reward = decisions.reward(i);
            row.counterfactual_nonterminal_reward = nonterminalReward;
            row.label_penalty = nonterminalReward - recomputed;
            counterfactual(counterfactualCount) = row;
        end
    end
    decomposition = struct2table(decomposition);
    counterfactual = counterfactual(1:counterfactualCount);
    counterfactual = struct2table(counterfactual);

    qValues = linspace(0, 2, 201).';
    qCurve = localQCurve(qValues, cfg);
    gammaValues = linspace(cfg.GAMMA_V_MIN, cfg.GAMMA_V_MAX, 101).';
    gammaCurve = localGammaCurve(gammaValues, cfg);
    saturation = localSaturationAudit(cfg);

    writetable(decomposition, fullfile(outputFolder, ...
        'reward_decomposition.csv'));
    writetable(counterfactual, fullfile(outputFolder, ...
        'counterfactual_terminal_labels.csv'));
    writetable(qCurve, fullfile(outputFolder, 'pace_reward_curve.csv'));
    writetable(gammaCurve, fullfile(outputFolder, 'gamma_v_reward_curve.csv'));
    writetable(saturation, fullfile(outputFolder, 'action_saturation_audit.csv'));

    summary = struct();
    summary.source_git_sha = localGitSha();
    summary.reward_version = string(cfg.REWARD.version);
    summary.actual_row_count = numberRows;
    summary.max_reward_reconstruction_error = max( ...
        abs(decomposition.logged_reward-decomposition.recomputed_reward));
    summary.numerical_solver_terminal_count = height(counterfactual);
    summary.total_numerical_label_penalty = sum(counterfactual.label_penalty);
    summary.mean_numerical_label_penalty = mean(counterfactual.label_penalty);
    summary.pace_reward_at_q1 = interp1(qCurve.q_pace, ...
        qCurve.pace_reward, 1);
    summary.pace_reward_at_q2 = qCurve.pace_reward(end);
    summary.reward_maximizing_q_on_test_grid = ...
        qCurve.q_pace(find(qCurve.reward == max(qCurve.reward), 1, 'last'));
    summary.gamma_v_maximizing_test_reward = gammaCurve.gamma_v( ...
        find(gammaCurve.reward == max(gammaCurve.reward), 1, 'last'));
    summary.dR2_direct_risk_is_inactive = all(saturation.risk_r2 == 0);
    summary.acceleration_direct_risk_is_inactive = ...
        all(saturation.risk_a == 0);
    summary.reward_changed = false;
    save(fullfile(outputFolder, 'reward_audit.mat'), ...
        'summary', 'decomposition', 'counterfactual', 'qCurve', ...
        'gammaCurve', 'saturation', 'cfg', '-v7.3');
    localWriteJson(fullfile(outputFolder, 'reward_audit_summary.json'), summary);
    localPlot(qCurve, gammaCurve, outputFolder);
    audit = struct('summary', summary, 'decomposition', decomposition, ...
        'counterfactual_labels', counterfactual, 'pace_curve', qCurve, ...
        'gamma_curve', gammaCurve, 'saturation', saturation, ...
        'output_folder', string(outputFolder));
end

function window = localWindow(row)
    window = struct();
    window.tracking_error_mean = row.tracking_error_mean;
    window.control_effort_mean = row.control_effort_mean;
    window.Ieq_window = row.Ieq_window_A;
    window.soc_start_pct = row.soc_start_pct;
    window.soc_end_pct = row.soc_end_pct;
    window.lag_frac = row.lag_frac;
    window.time_frac = row.time_frac;
    window.progress_frac = row.progress_frac;
    window.distance_start_m = row.distance_start_m;
    window.window_distance_m = row.window_distance_m;
    window.v_exec = row.v_exec;
    window.a_exec = row.a_exec;
    window.delta_v_exec = row.delta_v_exec;
    window.delta_gamma_v = row.delta_gamma_v;
    window.dR2 = row.dR2;
    window.state_norm_proxy = row.state_norm_proxy;
    window.com_speed_mag = row.com_speed_mag;
    terminalReason = row.terminal_reason;
    if ismissing(terminalReason)
        terminalReason = "";
    end
    window.terminal_reason = char(terminalReason);
    window.battery = struct('margin_norm', row.soc_end_pct/100);
end

function row = localDecompositionRow(source, reward, nonterminalReward, info, cfg)
    row = localEmptyDecomposition();
    row.episode_idx = source.episode_idx;
    row.decision_idx = source.decision_idx;
    row.terminal_reason = source.terminal_reason;
    row.q_pace = info.q_pace;
    row.logged_reward = source.reward;
    row.recomputed_reward = reward;
    row.pace = info.pace_reward;
    row.lag = -info.lag_penalty;
    row.risk = -cfg.REWARD.w_risk*info.risk_score;
    row.battery = -info.battery_penalty;
    row.tracking = -cfg.REWARD.w_track*info.nt;
    row.effort = -cfg.REWARD.w_effort*info.nu;
    row.slow_speed = -cfg.REWARD.w_slow*info.slow_pen;
    row.mission_guard = -info.mission_guard_penalty;
    row.adaptive = info.adaptive_efficiency_reward - ...
        info.adaptive_conservation_penalty;
    row.terminal = reward - nonterminalReward;
end

function curve = localQCurve(qValues, cfg)
    reward = nan(size(qValues));
    paceReward = nan(size(qValues));
    for i = 1:numel(qValues)
        window = localCanonicalWindow(cfg);
        window.window_distance_m = qValues(i)*cfg.MISSION.WINDOW_TARGET_M;
        [reward(i), info] = compute_rl_reward(window, cfg);
        paceReward(i) = info.pace_reward;
    end
    curve = table(qValues, paceReward, reward, ...
        'VariableNames', {'q_pace','pace_reward','reward'});
end

function curve = localGammaCurve(gammaValues, cfg)
    reward = nan(size(gammaValues));
    qPace = nan(size(gammaValues));
    vCommand = nan(size(gammaValues));
    for i = 1:numel(gammaValues)
        window = localCanonicalWindow(cfg);
        [vCommand(i), window.a_exec] = apply_command_governor( ...
            cfg.V_REQ_FIXED, cfg.A_REQ_FIXED, gammaValues(i), ...
            cfg.GAMMA_A_MIN, cfg);
        window.v_exec = vCommand(i);
        window.window_distance_m = vCommand(i)*cfg.CHUNK_DURATION*cfg.APPLY_EVERY;
        [reward(i), info] = compute_rl_reward(window, cfg);
        qPace(i) = info.q_pace;
    end
    curve = table(gammaValues, vCommand, qPace, reward, ...
        'VariableNames', {'gamma_v','v_command','q_pace','reward'});
end

function window = localCanonicalWindow(cfg)
    window = struct('tracking_error_mean', cfg.TRACK_REF, ...
        'control_effort_mean', cfg.EFFORT_REF, 'Ieq_window', 55, ...
        'soc_start_pct', 75.5, 'soc_end_pct', 75, 'lag_frac', 0, ...
        'time_frac', 0.5, 'progress_frac', 0.5, ...
        'distance_start_m', 160, ...
        'window_distance_m', cfg.MISSION.WINDOW_TARGET_M, ...
        'v_exec', cfg.V_REQ_FIXED, 'a_exec', 0.5, ...
        'delta_v_exec', 0, 'delta_gamma_v', 0, 'dR2', 0, ...
        'state_norm_proxy', 0, 'com_speed_mag', cfg.V_REQ_FIXED, ...
        'terminal_reason', '', ...
        'battery', struct('margin_norm', 0.75));
end

function saturation = localSaturationAudit(cfg)
    dR2 = [-cfg.DR_MAX; 0; cfg.DR_MAX];
    aExec = [cfg.A_MIN; 0.5; min(cfg.A_MAX, cfg.A_REQ_FIXED*cfg.GAMMA_A_MAX)];
    riskR2 = max(((-dR2)-cfg.REWARD.risk_r2_thr) / ...
        cfg.REWARD.risk_r2_scale, 0);
    riskA = max((aExec-cfg.REWARD.risk_a_thr) / ...
        cfg.REWARD.risk_a_scale, 0);
    saturation = table(dR2, aExec, riskR2, riskA, ...
        'VariableNames', {'dR2','a_exec','risk_r2','risk_a'});
end

function row = localEmptyDecomposition()
    row = struct('episode_idx', NaN, 'decision_idx', NaN, ...
        'terminal_reason', "", 'q_pace', NaN, 'logged_reward', NaN, ...
        'recomputed_reward', NaN, 'pace', NaN, 'lag', NaN, ...
        'risk', NaN, 'battery', NaN, 'tracking', NaN, 'effort', NaN, ...
        'slow_speed', NaN, 'mission_guard', NaN, 'adaptive', NaN, ...
        'terminal', NaN);
end

function row = localEmptyCounterfactual()
    row = struct('episode_idx', NaN, 'decision_idx', NaN, ...
        'failure_qp_id', "", 'logged_terminal_label', "", ...
        'evidence_class', "", 'logged_reward', NaN, ...
        'counterfactual_nonterminal_reward', NaN, 'label_penalty', NaN);
end

function localPlot(qCurve, gammaCurve, outputFolder)
    figureHandle = figure('Visible', 'off', 'Color', 'w', ...
        'Position', [100, 100, 950, 380]);
    cleanupObj = onCleanup(@() close(figureHandle));
    tiledlayout(1, 2, 'TileSpacing', 'compact');
    nexttile;
    plot(qCurve.q_pace, qCurve.reward, 'LineWidth', 1.5);
    xline(1, '--');
    grid on;
    xlabel('q pace');
    ylabel('canonical one-window reward');
    nexttile;
    plot(gammaCurve.gamma_v, gammaCurve.reward, 'LineWidth', 1.5);
    grid on;
    xlabel('\gamma_v');
    ylabel('canonical one-window reward');
    exportgraphics(figureHandle, fullfile(outputFolder, ...
        'reward_preference_curves.png'), 'Resolution', 160);
end

function file = localSingleFile(folder, pattern)
    files = dir(fullfile(folder, '**', pattern));
    if numel(files) ~= 1
        error('audit_rl_reward:FileCount', ...
            'Expected one %s under %s; found %d.', ...
            pattern, folder, numel(files));
    end
    file = string(fullfile(files.folder, files.name));
end

function localWriteJson(file, value)
    fid = fopen(file, 'w');
    if fid < 0
        error('audit_rl_reward:OpenFailed', 'Could not write %s.', file);
    end
    cleanupObj = onCleanup(@() fclose(fid));
    fprintf(fid, '%s\n', jsonencode(value, PrettyPrint=true));
end

function sha = localGitSha()
    [status, text] = system('git rev-parse HEAD');
    if status == 0
        sha = string(strtrim(text));
    else
        sha = "unknown";
    end
end
