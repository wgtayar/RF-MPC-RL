function evaluate_saved_agents_deterministic(agentNumbers, numEpisodes, agentRoot)

    if nargin < 2 || isempty(numEpisodes)
        numEpisodes = 5;
    end

    rootDir = bootstrap_RF_MPC_RL();
    rehash;

    if nargin < 3 || isempty(agentRoot)
        agentRoot = fullfile(rootDir, 'RL Midtraining Logs', ...
            'Successful Agents', 'run_2026-07-03_05-38-23');
    end

    if nargin < 1 || isempty(agentNumbers)
        agentNumbers = localDiscoverAgentNumbers(agentRoot);
        if isempty(agentNumbers)
            error('evaluate_saved_agents_deterministic:NoAgents', ...
                'No Agent<number>.mat files were found under: %s', agentRoot);
        end
        fprintf('[AGENT DISCOVERY] Found available agents: %s\n', mat2str(agentNumbers));
    end

    if ~exist(agentRoot, 'dir')
        error('Agent directory does not exist: %s', agentRoot);
    end

    cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');

    fprintf('\n[DETERMINISTIC EVAL] Preparing baseline environment via setup_RL_MPC...\n');
    setup_RL_MPC;

    Sbase = load(cfgPath, 'env', 'agent', 'cfg', 'lower_abs', 'upper_abs', 'initial_R');
    env = Sbase.env;
    agentBase = Sbase.agent;
    cfgBase = Sbase.cfg;
    lower_abs = Sbase.lower_abs;
    upper_abs = Sbase.upper_abs;
    initial_R = Sbase.initial_R;

    logsRoot = fullfile(rootDir, 'RL Midtraining Logs');
    if ~exist(logsRoot, 'dir')
        mkdir(logsRoot);
    end

    evalStamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    evalRoot = fullfile(logsRoot, ['eval_' evalStamp '_deterministic_agents']);
    if ~exist(evalRoot, 'dir')
        mkdir(evalRoot);
    end

    masterManifest = fullfile(evalRoot, ['eval_manifest_' evalStamp '.txt']);
    masterRunInfo = struct('run_id', ['eval_' evalStamp], ...
        'training_mode', 'deterministic_evaluation_batch', ...
        'max_episodes', numEpisodes, 'random_seed', cfgBase.RNG_SEED, ...
        'noise_variance', 0, 'noise_decay', 0, ...
        'source_agent', agentRoot);
    write_run_manifest(masterManifest, cfgBase, masterRunInfo);

    summaryRows = {};
    summaryNames = { ...
        'agent_label', ...
        'agent_file', ...
        'requested_eval_episodes', ...
        'observed_episodes', ...
        'mission_complete_count', ...
        'infeasible_count', ...
        'battery_terminal_count', ...
        'time_limit_count', ...
        'other_terminal_count', ...
        'completion_rate', ...
        'mean_final_distance_m', ...
        'std_final_distance_m', ...
        'mean_final_soc_pct', ...
        'std_final_soc_pct', ...
        'mean_target_cross_time_s', ...
        'mean_target_cross_soc_pct', ...
        'mean_total_reward', ...
        'mean_terminal_reward', ...
        'mean_late_current_below50_A', ...
        'mean_late_velocity_below50_mps', ...
        'mode_terminal_reason', ...
        'mode_failure_decision', ...
        'decision_csv', ...
        'chunk_csv', ...
        'failure_csv', ...
        'run_dir'};

    cleanupObj = onCleanup(@() localCleanupEval(cfgPath));

    for iAgent = 1:numel(agentNumbers)
        agentNum = agentNumbers(iAgent);
        agentLabel = sprintf('Agent%d', agentNum);

        fprintf('\n============================================================\n');
        fprintf('[EVAL AGENT] %s\n', agentLabel);
        fprintf('============================================================\n');

        agentFile = localResolveAgentFile(agentRoot, agentNum);

        if isempty(agentFile)
            warning('Could not find %s under %s. Skipping.', agentLabel, agentRoot);
            summaryRows(end+1, :) = localMissingSummaryRow( ...
                agentLabel, agentRoot, numEpisodes, summaryNames); %#ok<AGROW>
            continue
        end

        agentRunDir = fullfile(evalRoot, agentLabel);
        if ~exist(agentRunDir, 'dir')
            mkdir(agentRunDir);
        end

        agent = localLoadSavedAgent(agentFile);
        agent = localForceZeroNoise(agent);

        cfg = cfgBase;
        cfg.LOG.enable = true;
        cfg.LOG.print_chunk = true;
        cfg.LOG.print_decision = true;
        cfg.LOG.print_episode = true;

        cfg.RUN.enabled = true;
        cfg.RUN.root_dir = evalRoot;
        cfg.RUN.run_dir = agentRunDir;
        cfg.RUN.run_stamp = [evalStamp '_' agentLabel];
        cfg.RUN.log_file = fullfile(agentRunDir, ['eval_log_' evalStamp '_' agentLabel '.txt']);
        cfg.RUN.checkpoint_file = fullfile(agentRunDir, ['rl_checkpoints_' evalStamp '_' agentLabel '.mat']);
        cfg.RUN.chunk_csv = fullfile(agentRunDir, ['rl_chunks_' evalStamp '_' agentLabel '.csv']);
        cfg.RUN.decision_csv = fullfile(agentRunDir, ['rl_decisions_' evalStamp '_' agentLabel '.csv']);
        cfg.RUN.failure_csv = fullfile(agentRunDir, ['rl_failures_' evalStamp '_' agentLabel '.csv']);
        cfg.RUN.qp_failure_dir = fullfile(agentRunDir, 'qp_failures');
        if ~exist(cfg.RUN.qp_failure_dir, 'dir')
            mkdir(cfg.RUN.qp_failure_dir);
        end
        cfg.RUN.saved_agents_dir = '';
        cfg.RUN.eval_mode = 'deterministic_actor';
        cfg.RUN.source_agent = agentFile;
        cfg.RUN.num_eval_episodes = numEpisodes;
        cfg.RUN.exploration_noise = 0;
        cfg.RUN.training_updates = false;

        if isfield(cfg, 'EXPERIMENT') && isfield(cfg.EXPERIMENT, 'id')
            cfg.EXPERIMENT.id = sprintf('%s_eval_%s_deterministic', cfg.EXPERIMENT.id, agentLabel);
        else
            cfg.EXPERIMENT.id = sprintf('eval_%s_deterministic', agentLabel);
        end

        if isfield(cfg, 'EXPERIMENT') && isfield(cfg.EXPERIMENT, 'description')
            cfg.EXPERIMENT.description = sprintf('%s Deterministic actor evaluation for %s. No exploration noise. No training updates.', ...
                cfg.EXPERIMENT.description, agentLabel);
        else
            cfg.EXPERIMENT.description = sprintf('Deterministic actor evaluation for %s. No exploration noise. No training updates.', agentLabel);
        end

        save(cfgPath, 'env', 'agent', 'lower_abs', 'upper_abs', 'initial_R', 'cfg');
        save(fullfile(agentRunDir, ['cfg_snapshot_' evalStamp '_' agentLabel '.mat']), 'cfg', 'agentFile');

        agentRunInfo = struct('run_id', cfg.RUN.run_stamp, ...
            'training_mode', 'deterministic_evaluation', ...
            'max_episodes', numEpisodes, 'random_seed', cfg.RNG_SEED, ...
            'noise_variance', 0, 'noise_decay', 0, ...
            'source_agent', agentFile, 'run_label', agentLabel);
        write_run_manifest( ...
            fullfile(agentRunDir, ['eval_manifest_' evalStamp '_' agentLabel '.txt']), ...
            cfg, agentRunInfo);

        clear rlResetFunction rlStepFunction;

        diary off;
        diary(cfg.RUN.log_file);
        diary on;

        fprintf('[EVAL START] mode=deterministic_actor, agent=%s\n', agentLabel);
        fprintf('[SOURCE AGENT] %s\n', agentFile);
        fprintf('[RUN DIR] %s\n', agentRunDir);
        fprintf('[EPISODES] %d\n', numEpisodes);
        fprintf('[NOISE] zero / sim-mode\n');
        fprintf('[TRAINING UPDATES] false\n');
        fprintf('[MISSION] D=%.2f m, T=%.2f s, decisions=%d, chunks/decision=%d\n', ...
            cfg.MISSION.D_TARGET_M, cfg.MISSION_DURATION, cfg.EP_STEPS, cfg.APPLY_EVERY);
        fprintf('[ACTION LIMITS] gamma_v=[%.3f %.3f], v_cap=%.3f m/s, DR_MAX=%.6f\n', ...
            cfg.GAMMA_V_MIN, cfg.GAMMA_V_MAX, ...
            cfg.V_MIN + cfg.GAMMA_V_MAX * (cfg.V_MAX - cfg.V_MIN), cfg.DR_MAX);

        simOpts = rlSimulationOptions('MaxSteps', cfg.EP_STEPS);

        for ep = 1:numEpisodes
            fprintf('\n[EVAL EPISODE %d/%d | %s]\n', ep, numEpisodes, agentLabel);
            try
                sim(env, agent, simOpts);
            catch ME
                diary off;
                localCleanupEval(cfgPath);
                rethrow(ME);
            end
        end

        fprintf('\n[EVAL END] agent=%s\n', agentLabel);
        diary off;

        agentSummaryRow = localSummarizeDecisionCsv( ...
            agentLabel, agentFile, numEpisodes, cfg.RUN.decision_csv, ...
            cfg.RUN.chunk_csv, cfg.RUN.failure_csv, agentRunDir, summaryNames);

        summaryRows(end+1, :) = agentSummaryRow; %#ok<AGROW>

        agentSummaryTable = cell2table(agentSummaryRow, 'VariableNames', summaryNames);
        writetable(agentSummaryTable, fullfile(agentRunDir, ['eval_summary_' evalStamp '_' agentLabel '.csv']));

        fprintf('[SUMMARY] %s: complete=%d/%d, finalSOC=%.2f%%, finalDist=%.2f m\n', ...
            agentLabel, agentSummaryRow{5}, agentSummaryRow{4}, ...
            agentSummaryRow{13}, agentSummaryRow{11});
    end

    summaryTable = cell2table(summaryRows, 'VariableNames', summaryNames);
    summaryFile = fullfile(evalRoot, ['eval_summary_' evalStamp '.csv']);
    writetable(summaryTable, summaryFile);

    fprintf('\n============================================================\n');
    fprintf('[DETERMINISTIC EVAL COMPLETE]\n');
    fprintf('[EVAL ROOT] %s\n', evalRoot);
    fprintf('[SUMMARY CSV] %s\n', summaryFile);
    fprintf('============================================================\n');

    cfg = cfgBase;
    if isfield(cfg, 'LOG')
        cfg.LOG.enable = false;
    end
    if isfield(cfg, 'RUN')
        cfg.RUN.enabled = false;
    end
    agent = agentBase;
    save(cfgPath, 'env', 'agent', 'lower_abs', 'upper_abs', 'initial_R', 'cfg');
end

function agentNumbers = localDiscoverAgentNumbers(agentRoot)
    files = localListMatFilesRecursive(agentRoot);
    agentNumbers = [];
    for i = 1:numel(files)
        [~, baseName] = fileparts(files{i});
        token = regexp(baseName, '^Agent(\d+)$', 'tokens', 'once');
        if ~isempty(token)
            agentNumbers(end+1) = str2double(token{1}); %#ok<AGROW>
        end
    end
    agentNumbers = unique(agentNumbers, 'sorted');
end

function agentFile = localResolveAgentFile(agentRoot, agentNum)
    agentFile = '';

    exactNames = { ...
        sprintf('Agent%d.mat', agentNum), ...
        sprintf('agent%d.mat', agentNum), ...
        sprintf('Agent_%d.mat', agentNum), ...
        sprintf('agent_%d.mat', agentNum)};

    for i = 1:numel(exactNames)
        candidate = fullfile(agentRoot, exactNames{i});
        if exist(candidate, 'file')
            agentFile = candidate;
            return
        end
    end

    files = localListMatFilesRecursive(agentRoot);
    if isempty(files)
        return
    end

    targetPattern = sprintf('agent[^0-9]*0*%d([^0-9]|$)', agentNum);

    for i = 1:numel(files)
        [~, name, ext] = fileparts(files{i});
        fullName = [name ext];
        if ~isempty(regexpi(fullName, targetPattern, 'once'))
            agentFile = files{i};
            return
        end
    end

    fallbackPattern = sprintf('%d', agentNum);
    for i = 1:numel(files)
        [~, name, ~] = fileparts(files{i});
        if contains(name, fallbackPattern)
            agentFile = files{i};
            return
        end
    end
end

function files = localListMatFilesRecursive(rootDir)
    d = dir(fullfile(rootDir, '**', '*.mat'));
    if ~isempty(d)
        files = arrayfun(@(x) fullfile(x.folder, x.name), d, 'UniformOutput', false);
        return
    end

    files = {};
    stack = {rootDir};

    while ~isempty(stack)
        current = stack{end};
        stack(end) = [];

        mats = dir(fullfile(current, '*.mat'));
        for i = 1:numel(mats)
            files{end+1} = fullfile(mats(i).folder, mats(i).name); %#ok<AGROW>
        end

        dirs = dir(current);
        dirs = dirs([dirs.isdir]);
        for i = 1:numel(dirs)
            nm = dirs(i).name;
            if strcmp(nm, '.') || strcmp(nm, '..')
                continue
            end
            stack{end+1} = fullfile(dirs(i).folder, nm); %#ok<AGROW>
        end
    end
end

function agent = localLoadSavedAgent(savedAgentPath)
    vars = whos('-file', savedAgentPath);
    names = {vars.name};
    preferredNames = {'agent', 'saved_agent', 'trainedAgent', 'Agent'};

    for i = 1:numel(preferredNames)
        if any(strcmp(names, preferredNames{i}))
            Sagent = load(savedAgentPath, preferredNames{i});
            agent = Sagent.(preferredNames{i});
            fprintf('[LOAD AGENT] Loaded variable "%s" from %s\n', ...
                preferredNames{i}, savedAgentPath);
            return
        end
    end

    Sagent = load(savedAgentPath);
    names = fieldnames(Sagent);

    for i = 1:numel(names)
        candidate = Sagent.(names{i});
        if isa(candidate, 'rl.agent.AbstractAgent') || contains(class(candidate), 'rl')
            agent = candidate;
            fprintf('[LOAD AGENT] Loaded variable "%s" of class "%s" from %s\n', ...
                names{i}, class(candidate), savedAgentPath);
            return
        end
    end

    error('No RL agent object found in saved file: %s', savedAgentPath);
end

function agent = localForceZeroNoise(agent)
    try
        if isprop(agent, 'AgentOptions') && isprop(agent.AgentOptions, 'NoiseOptions')
            agent.AgentOptions.NoiseOptions.Variance = 0;
            agent.AgentOptions.NoiseOptions.VarianceDecayRate = 0;
        end
    catch ME
        warning('evaluate_saved_agents_deterministic:NoiseOverrideFailed', ...
            ['Could not explicitly zero agent noise. sim-mode should still ', ...
            'be deterministic. Reason: %s'], ME.message);
    end
end

function row = localSummarizeDecisionCsv(agentLabel, agentFile, requestedEpisodes, ...
        decisionCsv, chunkCsv, failureCsv, runDir, summaryNames)

    if ~exist(decisionCsv, 'file')
        row = localMissingSummaryRow(agentLabel, agentFile, requestedEpisodes, summaryNames);
        row{end} = runDir;
        return
    end

    T = readtable(decisionCsv, 'PreserveVariableNames', true);

    if isempty(T) || ~ismember('episode_idx', T.Properties.VariableNames)
        row = localMissingSummaryRow(agentLabel, agentFile, requestedEpisodes, summaryNames);
        row{end} = runDir;
        return
    end

    episodeIds = unique(T.episode_idx);
    lastIdx = zeros(numel(episodeIds), 1);

    for i = 1:numel(episodeIds)
        rows = find(T.episode_idx == episodeIds(i));
        lastIdx(i) = rows(end);
    end

    E = T(lastIdx, :);
    observedEpisodes = height(E);

    terminalReason = strings(observedEpisodes, 1);
    if ismember('terminal_reason', E.Properties.VariableNames)
        terminalReason = string(E.terminal_reason);
    end

    completeMask = terminalReason == "mission_complete";
    infeasibleMask = terminalReason == "infeasible";
    batteryMask = terminalReason == "battery_terminal";
    timeMask = terminalReason == "time_limit";
    otherMask = ~(completeMask | infeasibleMask | batteryMask | timeMask);

    completeCount = sum(completeMask);
    infeasibleCount = sum(infeasibleMask);
    batteryCount = sum(batteryMask);
    timeCount = sum(timeMask);
    otherCount = sum(otherMask);

    completionRate = completeCount / max(observedEpisodes, 1);

    meanFinalDistance = localMeanVar(E, 'distance_end_m');
    stdFinalDistance = localStdVar(E, 'distance_end_m');
    meanFinalSoc = localMeanVar(E, 'soc_end_pct');
    stdFinalSoc = localStdVar(E, 'soc_end_pct');
    meanTotalReward = localSumByEpisodeMean(T, 'reward');
    meanTerminalReward = localMeanVar(E, 'reward');

    targetCrossTime = NaN;
    targetCrossSoc = NaN;
    if ismember('target_crossed_this_window', T.Properties.VariableNames)
        crossMask = logical(T.target_crossed_this_window);
        if any(crossMask)
            targetCrossTime = localMeanVector(T.target_cross_time_s(crossMask));
            targetCrossSoc = localMeanVector(T.target_cross_soc_pct(crossMask));
        end
    end

    lateCurrent = NaN;
    lateVelocity = NaN;
    if ismember('soc_end_pct', T.Properties.VariableNames) && ismember('Ieq_window_A', T.Properties.VariableNames)
        lateMask = T.soc_end_pct < 50 & T.feasible == 1;
        if any(lateMask)
            lateCurrent = localMeanVector(T.Ieq_window_A(lateMask));
            if ismember('v_exec', T.Properties.VariableNames)
                lateVelocity = localMeanVector(T.v_exec(lateMask));
            end
        end
    end

    modeTerminalReason = localModeString(terminalReason);

    modeFailureDecision = NaN;
    if exist(failureCsv, 'file')
        F = readtable(failureCsv, 'PreserveVariableNames', true);
        if ~isempty(F) && ismember('decision_idx', F.Properties.VariableNames)
            modeFailureDecision = localModeNumeric(F.decision_idx);
        end
    end

    row = { ...
        agentLabel, ...
        agentFile, ...
        requestedEpisodes, ...
        observedEpisodes, ...
        completeCount, ...
        infeasibleCount, ...
        batteryCount, ...
        timeCount, ...
        otherCount, ...
        completionRate, ...
        meanFinalDistance, ...
        stdFinalDistance, ...
        meanFinalSoc, ...
        stdFinalSoc, ...
        targetCrossTime, ...
        targetCrossSoc, ...
        meanTotalReward, ...
        meanTerminalReward, ...
        lateCurrent, ...
        lateVelocity, ...
        modeTerminalReason, ...
        modeFailureDecision, ...
        decisionCsv, ...
        chunkCsv, ...
        failureCsv, ...
        runDir};

    if numel(row) ~= numel(summaryNames)
        error('Internal summary row length mismatch.');
    end
end

function row = localMissingSummaryRow(agentLabel, agentFile, requestedEpisodes, summaryNames)
    row = { ...
        agentLabel, agentFile, requestedEpisodes, 0, 0, 0, 0, 0, 0, NaN, ...
        NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, ...
        "missing", NaN, "", "", "", ""};

    if numel(row) ~= numel(summaryNames)
        error('Internal missing summary row length mismatch.');
    end
end

function val = localMeanVar(T, varName)
    val = NaN;
    if ismember(varName, T.Properties.VariableNames)
        val = localMeanVector(T.(varName));
    end
end

function val = localStdVar(T, varName)
    val = NaN;
    if ismember(varName, T.Properties.VariableNames)
        x = T.(varName);
        if iscell(x) || isstring(x)
            x = str2double(string(x));
        end
        x = x(isfinite(x));
        if ~isempty(x)
            val = std(x);
        end
    end
end

function val = localMeanVector(x)
    if iscell(x) || isstring(x)
        x = str2double(string(x));
    end
    x = x(isfinite(x));
    if isempty(x)
        val = NaN;
    else
        val = mean(x);
    end
end

function val = localSumByEpisodeMean(T, varName)
    val = NaN;
    if ~ismember(varName, T.Properties.VariableNames) || ~ismember('episode_idx', T.Properties.VariableNames)
        return
    end

    episodeIds = unique(T.episode_idx);
    sums = NaN(numel(episodeIds), 1);

    for i = 1:numel(episodeIds)
        rows = T.episode_idx == episodeIds(i);
        x = T.(varName)(rows);
        if iscell(x) || isstring(x)
            x = str2double(string(x));
        end
        x = x(isfinite(x));
        if ~isempty(x)
            sums(i) = sum(x);
        end
    end

    sums = sums(isfinite(sums));
    if ~isempty(sums)
        val = mean(sums);
    end
end

function out = localModeString(x)
    x = string(x);
    x = x(~ismissing(x) & x ~= "");
    if isempty(x)
        out = "";
        return
    end

    u = unique(x);
    counts = zeros(numel(u), 1);

    for i = 1:numel(u)
        counts(i) = sum(x == u(i));
    end

    [~, idx] = max(counts);
    out = u(idx);
end

function out = localModeNumeric(x)
    x = x(isfinite(x));
    if isempty(x)
        out = NaN;
        return
    end
    u = unique(x);
    counts = zeros(numel(u), 1);
    for i = 1:numel(u)
        counts(i) = sum(x == u(i));
    end
    [~, idx] = max(counts);
    out = u(idx);
end

function localCleanupEval(cfgPath)
    try
        diary off;
    catch
    end

    try
        if exist(cfgPath, 'file')
            S = load(cfgPath, 'cfg');
            cfg = S.cfg;
            if isfield(cfg, 'LOG')
                cfg.LOG.enable = false;
            end
            if isfield(cfg, 'RUN')
                cfg.RUN.enabled = false;
            end
            save(cfgPath, 'cfg', '-append');
        end
    catch
    end
end
