function Results = transition_sweep(cfg)
    rootDir = fileparts(mfilename('fullpath'));

    add_if_exists(fullfile(rootDir, 'fcns'));
    add_if_exists(fullfile(rootDir, 'fcns_MPC'));

    if nargin < 1
        cfg = struct();
    end

    cfg = fill_transition_defaults(cfg, rootDir);

    logsRoot = fullfile(rootDir, 'MPC Boundaries Exploration');
    if ~exist(logsRoot, 'dir')
        mkdir(logsRoot);
    end

    runStamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    runDir = fullfile(logsRoot, ['transition_run_' runStamp]);
    if ~exist(runDir, 'dir')
        mkdir(runDir);
    end

    txtFile = fullfile(runDir, ['transition_sweep_log_' runStamp '.txt']);
    fid = fopen(txtFile, 'w');
    if fid < 0
        error('Could not create transition log file: %s', txtFile);
    end
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

    log_msg(fid, cfg, 'TRANSITION SWEEP START');
    log_msg(fid, cfg, ['Run directory: ' runDir]);
    log_msg(fid, cfg, sprintf('chunk_duration = %.3f s', cfg.chunk_duration));
    log_msg(fid, cfg, sprintf('run_pairwise = %d', cfg.run_pairwise));
    log_msg(fid, cfg, sprintf('run_frontier = %d', cfg.run_frontier));
    log_msg(fid, cfg, sprintf('run_adaptive = %d', cfg.run_adaptive));
    log_msg(fid, cfg, sprintf('usable_tracking_threshold = %.4f', cfg.usable_tracking_threshold));
    log_msg(fid, cfg, sprintf('usable_effort_threshold = %.4f', cfg.usable_effort_threshold));
    log_msg(fid, cfg, sprintf('save_iteration_logs = %d', cfg.save_iteration_logs));
    log_msg(fid, cfg, sprintf('save_full_qp_on_failure = %d', cfg.save_full_qp_on_failure));
    log_msg(fid, cfg, sprintf('autosave_every = %d', cfg.autosave_every));
    log_msg(fid, cfg, 'Stage definitions: S = static seed, T = one-step reachable, M = multi-step reachable, U = unreachable, P = usable-feasible, Q = solver-feasible but poor-quality, I = infeasible.');

    staticInfo = load_static_case_table(cfg);
    seedTable = build_seed_table(staticInfo.caseTable, cfg);

    log_msg(fid, cfg, sprintf('Loaded static case table from: %s', staticInfo.source_file));
    log_msg(fid, cfg, sprintf('Static cases available = %d', height(staticInfo.caseTable)));
    log_msg(fid, cfg, sprintf('Seed nodes found = %d', height(seedTable)));

    seedCsv = fullfile(runDir, ['transition_seed_nodes_' runStamp '.csv']);
    writetable(seedTable, seedCsv);

    nodeRegistry = init_node_registry(seedTable);
    nodeRegistryCsv = fullfile(runDir, ['transition_node_registry_initial_' runStamp '.csv']);
    writetable(node_registry_to_table(nodeRegistry), nodeRegistryCsv);

    Results = struct();
    Results.run_stamp = runStamp;
    Results.run_dir = runDir;
    Results.created_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    Results.static_source = staticInfo.source_file;
    Results.seed_table = seedTable;

    if cfg.run_pairwise
        log_msg(fid, cfg, 'PAIRWISE STAGE START');
        pairwiseOut = run_pairwise_stage(seedTable, staticInfo.caseTable, nodeRegistry, cfg, fid, runDir, runStamp);
        nodeRegistry = pairwiseOut.nodeRegistry;
        Results.pairwise = pairwiseOut.summary;
        log_msg(fid, cfg, 'PAIRWISE STAGE END');
    end

    if cfg.run_frontier
        log_msg(fid, cfg, 'FRONTIER STAGE START');
        frontierOut = run_frontier_stage(nodeRegistry, staticInfo.caseTable, cfg, fid, runDir, runStamp);
        nodeRegistry = frontierOut.nodeRegistry;
        Results.frontier = frontierOut.summary;
        log_msg(fid, cfg, 'FRONTIER STAGE END');
    end

    if cfg.run_adaptive
        log_msg(fid, cfg, 'ADAPTIVE STAGE START');
        adaptiveOut = run_adaptive_stage(nodeRegistry, staticInfo.caseTable, cfg, fid, runDir, runStamp);
        nodeRegistry = adaptiveOut.nodeRegistry;
        Results.adaptive = adaptiveOut.summary;
        log_msg(fid, cfg, 'ADAPTIVE STAGE END');
    end

    finalNodeTable = node_registry_to_table(nodeRegistry);
    finalNodeCsv = fullfile(runDir, ['transition_node_registry_final_' runStamp '.csv']);
    finalNodeMat = fullfile(runDir, ['transition_node_registry_final_' runStamp '.mat']);
    writetable(finalNodeTable, finalNodeCsv);
    save(finalNodeMat, 'finalNodeTable', 'nodeRegistry');

    Results.node_table = finalNodeTable;
    Results.node_registry = nodeRegistry;

    save(fullfile(runDir, ['transition_sweep_results_' runStamp '.mat']), 'Results', '-v7.3');

    log_msg(fid, cfg, sprintf('Final node registry CSV: %s', finalNodeCsv));
    log_msg(fid, cfg, sprintf('Final node registry MAT: %s', finalNodeMat));
    log_msg(fid, cfg, 'TRANSITION SWEEP END');

    fprintf('[TRANSITION SWEEP END] run_dir=%s\n', runDir);
    fprintf('[TRANSITION SWEEP END] seeds=%d, final_nodes=%d\n', height(seedTable), height(finalNodeTable));
end

function cfg = fill_transition_defaults(cfg, rootDir)
    if ~isfield(cfg, 'gait')
        cfg.gait = 0;
    end

    if ~isfield(cfg, 'chunk_duration')
        cfg.chunk_duration = 5;
    end

    if ~isfield(cfg, 'run_pairwise')
        cfg.run_pairwise = true;
    end

    if ~isfield(cfg, 'run_frontier')
        cfg.run_frontier = false;
    end

    if ~isfield(cfg, 'run_adaptive')
        cfg.run_adaptive = false;
    end

    if ~isfield(cfg, 'static_case_csv')
        cfg.static_case_csv = '';
    end

    if ~isfield(cfg, 'usable_tracking_threshold')
        cfg.usable_tracking_threshold = 1.0;
    end

    if ~isfield(cfg, 'usable_effort_threshold')
        cfg.usable_effort_threshold = inf;
    end

    if ~isfield(cfg, 'compute_current_battery')
        cfg.compute_current_battery = true;
    end

    if ~isfield(cfg, 'save_iteration_logs')
        cfg.save_iteration_logs = false;
    end

    if ~isfield(cfg, 'save_full_qp_on_failure')
        cfg.save_full_qp_on_failure = true;
    end

    if ~isfield(cfg, 'save_fail_state_vectors')
        cfg.save_fail_state_vectors = true;
    end

    if ~isfield(cfg, 'save_success_snapshots')
        cfg.save_success_snapshots = true;
    end

    if ~isfield(cfg, 'autosave_every')
        cfg.autosave_every = 25;
    end

    if ~isfield(cfg, 'print_to_cli')
        cfg.print_to_cli = true;
    end

    if ~isfield(cfg, 'print_case_start')
        cfg.print_case_start = true;
    end

    if ~isfield(cfg, 'print_failure_snapshot')
        cfg.print_failure_snapshot = true;
    end

    if ~isfield(cfg, 'cfgPath')
        cfg.cfgPath = fullfile(rootDir, 'rlEnv_MPC_R.mat');
    end

    if ~isfield(cfg, 'frontier_drho')
        cfg.frontier_drho = [-0.05 0 0.05];
    end

    if ~isfield(cfg, 'frontier_dv')
        cfg.frontier_dv = [-0.1 0 0.1];
    end

    if ~isfield(cfg, 'frontier_da')
        cfg.frontier_da = [-0.2 0 0.2];
    end

    if ~isfield(cfg, 'frontier_rho_bounds')
        cfg.frontier_rho_bounds = [0.70 1.30];
    end

    if ~isfield(cfg, 'frontier_v_bounds')
        cfg.frontier_v_bounds = [0.3 1.5];
    end

    if ~isfield(cfg, 'frontier_a_bounds')
        cfg.frontier_a_bounds = [0.2 4.0];
    end

    if ~isfield(cfg, 'frontier_use_feasible_va_mask')
        cfg.frontier_use_feasible_va_mask = true;
    end

    if ~isfield(cfg, 'tacc_min')
        cfg.tacc_min = 0.2;
    end

    if ~isfield(cfg, 'tacc_max')
        cfg.tacc_max = 2.0;
    end

    if ~isfield(cfg, 'adaptive_max_depth')
        cfg.adaptive_max_depth = 3;
    end

    if ~isfield(cfg, 'adaptive_source_quality')
        cfg.adaptive_source_quality = 'solver';
    end

    if ~isfield(cfg, 'adaptive_max_new_nodes_per_depth')
        cfg.adaptive_max_new_nodes_per_depth = 100;
    end

    if ~isfield(cfg, 'target_grid_rho_step')
        cfg.target_grid_rho_step = 0.05;
    end

    if ~isfield(cfg, 'target_grid_v_step')
        cfg.target_grid_v_step = 0.1;
    end

    if ~isfield(cfg, 'target_grid_a_step')
        cfg.target_grid_a_step = 0.2;
    end

    if exist(cfg.cfgPath, 'file')
        S = load(cfg.cfgPath, 'cfg');
        if isfield(S, 'cfg')
            cfg.rlCfg = S.cfg;
        else
            cfg.rlCfg = build_fallback_rl_cfg(rootDir);
        end
    else
        cfg.rlCfg = build_fallback_rl_cfg(rootDir);
    end
end

function staticInfo = load_static_case_table(cfg)
    if ~isempty(cfg.static_case_csv) && exist(cfg.static_case_csv, 'file')
        staticFile = cfg.static_case_csv;
    else
        staticFile = find_latest_static_csv();
        if isempty(staticFile)
            error('Could not find a static_sweep_cases_*.csv file. Set cfg.static_case_csv explicitly.');
        end
    end

    caseTable = readtable(staticFile);

    if ~all(ismember({'rhoR','v_cmd','a_cmd','feasible'}, caseTable.Properties.VariableNames))
        error('Static CSV is missing required columns.');
    end

    [~, ia] = unique(caseTable(:, {'rhoR','v_cmd','a_cmd'}), 'rows', 'last');
    caseTable = caseTable(sort(ia), :);

    staticInfo = struct();
    staticInfo.source_file = staticFile;
    staticInfo.caseTable = caseTable;
end

function seedTable = build_seed_table(caseTable, cfg)
    seedMask = caseTable.feasible == 1;
    seedTable = caseTable(seedMask, :);
    seedTable = sortrows(seedTable, {'v_cmd','a_cmd','rhoR'});

    n = height(seedTable);
    node_id = (1:n).';
    node_key = strings(n,1);
    node_class = strings(n,1);
    quality_class = strings(n,1);
    static_solver_feasible = true(n,1);
    static_usable = false(n,1);

    for i = 1:n
        node_key(i) = make_node_key(seedTable.rhoR(i), seedTable.v_cmd(i), seedTable.a_cmd(i));
        node_class(i) = "S";
        static_usable(i) = is_usable_row(seedTable(i,:), cfg);
        if static_usable(i)
            quality_class(i) = "P";
        else
            quality_class(i) = "Q";
        end
    end

    seedTable.node_id = node_id;
    seedTable.node_key = node_key;
    seedTable.node_class = node_class;
    seedTable.quality_class = quality_class;
    seedTable.static_solver_feasible = static_solver_feasible;
    seedTable.static_usable = static_usable;
end

function nodeRegistry = init_node_registry(seedTable)
    n = height(seedTable);
    nodeRegistry(n,1) = empty_node_entry();

    for i = 1:n
        nodeRegistry(i).node_id = seedTable.node_id(i);
        nodeRegistry(i).node_key = seedTable.node_key(i);
        nodeRegistry(i).rhoR = seedTable.rhoR(i);
        nodeRegistry(i).v_cmd = seedTable.v_cmd(i);
        nodeRegistry(i).a_cmd = seedTable.a_cmd(i);
        nodeRegistry(i).is_static_seed = true;
        nodeRegistry(i).static_solver_feasible = true;
        nodeRegistry(i).static_usable = seedTable.static_usable(i);
        nodeRegistry(i).min_depth = 0;
        nodeRegistry(i).node_class = 'S';
        if seedTable.static_usable(i)
            nodeRegistry(i).quality_class = 'P';
            nodeRegistry(i).ever_usable_feasible = true;
        else
            nodeRegistry(i).quality_class = 'Q';
            nodeRegistry(i).ever_solver_feasible = true;
        end
    end
end

function out = run_pairwise_stage(seedTable, staticTable, nodeRegistry, cfg, fid, runDir, runStamp)
    nSeeds = height(seedTable);

    log_msg(fid, cfg, sprintf('Pairwise stage seeds = %d', nSeeds));

    sourceResults(nSeeds,1) = empty_source_rollout();
    sourceCases(nSeeds,1) = empty_transition_record();

    for i = 1:nSeeds
        srcPoint = row_to_point(seedTable(i,:));

        if cfg.print_case_start
            log_msg(fid, cfg, sprintf('PAIRWISE SOURCE %d/%d | rhoR=%.3f | v=%.3f | a=%.3f', ...
                i, nSeeds, srcPoint.rhoR, srcPoint.v_cmd, srcPoint.a_cmd));
        end

        srcRun = run_mpc_chunk(srcPoint, cfg, []);
        sourceResults(i) = source_rollout_from_chunk(seedTable(i,:), srcRun);

        sourceCases(i).stage = "pairwise_source";
        sourceCases(i).source_node_id = seedTable.node_id(i);
        sourceCases(i).source_key = seedTable.node_key(i);
        sourceCases(i).target_node_id = seedTable.node_id(i);
        sourceCases(i).target_key = seedTable.node_key(i);
        sourceCases(i).solver_feasible = srcRun.solver_feasible;
        sourceCases(i).usable_feasible = srcRun.usable_feasible;
        sourceCases(i).success_level = classify_success_level(srcRun.solver_feasible, srcRun.usable_feasible);
        sourceCases(i).transition_relation = "source_reset_rollout";
        sourceCases(i).fail_reason = string(srcRun.fail_reason);
        sourceCases(i).fail_time_s = srcRun.fail_time_s;
        sourceCases(i).fail_iter = srcRun.fail_iter;
        sourceCases(i).quadprog_exitflag = srcRun.quadprog_exitflag;
        sourceCases(i).tracking_error_mean = srcRun.tracking_error_mean;
        sourceCases(i).control_effort_mean = srcRun.control_effort_mean;
        sourceCases(i).Ieq_A = srcRun.Ieq_A;
        sourceCases(i).soc_end_pct = srcRun.soc_end_pct;

        if ~srcRun.solver_feasible
            log_msg(fid, cfg, sprintf('PAIRWISE SOURCE FAIL | node=%d | reason=%s | fail_t=%.3f', ...
                seedTable.node_id(i), srcRun.fail_reason, srcRun.fail_time_s));
        end
    end

    sourceCsv = fullfile(runDir, ['transition_pairwise_sources_' runStamp '.csv']);
    sourceMat = fullfile(runDir, ['transition_pairwise_sources_' runStamp '.mat']);
    writetable(source_rollout_to_table(sourceResults), sourceCsv);
    save(sourceMat, 'sourceResults');

    nEdges = nSeeds * nSeeds;
    edgeResults(nEdges,1) = empty_transition_record();

    solverMat = zeros(nSeeds, nSeeds);
    usableMat = zeros(nSeeds, nSeeds);

    edgeIdx = 0;
    for i = 1:nSeeds
        src = sourceResults(i);

        for j = 1:nSeeds
            edgeIdx = edgeIdx + 1;

            dstPoint = row_to_point(seedTable(j,:));
            rec = empty_transition_record();
            rec.stage = "pairwise";
            rec.source_node_id = seedTable.node_id(i);
            rec.target_node_id = seedTable.node_id(j);
            rec.source_key = seedTable.node_key(i);
            rec.target_key = seedTable.node_key(j);
            rec.source_rhoR = seedTable.rhoR(i);
            rec.source_v = seedTable.v_cmd(i);
            rec.source_a = seedTable.a_cmd(i);
            rec.target_rhoR = seedTable.rhoR(j);
            rec.target_v = seedTable.v_cmd(j);
            rec.target_a = seedTable.a_cmd(j);
            rec.delta_rhoR = rec.target_rhoR - rec.source_rhoR;
            rec.delta_v = rec.target_v - rec.source_v;
            rec.delta_a = rec.target_a - rec.source_a;

            dstStatic = lookup_static_status(staticTable, dstPoint, cfg);
            rec.target_static_solver = dstStatic.static_solver;
            rec.target_static_usable = dstStatic.static_usable;

            if ~src.source_solver_feasible
                rec.solver_feasible = false;
                rec.usable_feasible = false;
                rec.success_level = "I";
                rec.transition_relation = "source_seed_failed_from_reset";
                rec.fail_reason = "source_seed_failed";
                edgeResults(edgeIdx) = rec;
                continue
            end

            if cfg.print_case_start
                log_msg(fid, cfg, sprintf('PAIRWISE EDGE %d/%d | src=%d -> dst=%d', ...
                    edgeIdx, nEdges, rec.source_node_id, rec.target_node_id));
            end

            dstRun = run_mpc_chunk(dstPoint, cfg, src.terminal_ctx);

            rec.solver_feasible = dstRun.solver_feasible;
            rec.usable_feasible = dstRun.usable_feasible;
            rec.success_level = classify_success_level(dstRun.solver_feasible, dstRun.usable_feasible);
            rec.transition_relation = classify_transition_relation(rec.target_static_solver, rec.solver_feasible);
            rec.fail_reason = string(dstRun.fail_reason);
            rec.fail_time_s = dstRun.fail_time_s;
            rec.fail_iter = dstRun.fail_iter;
            rec.quadprog_exitflag = dstRun.quadprog_exitflag;
            rec.tracking_error_mean = dstRun.tracking_error_mean;
            rec.control_effort_mean = dstRun.control_effort_mean;
            rec.Ieq_A = dstRun.Ieq_A;
            rec.soc_end_pct = dstRun.soc_end_pct;
            rec.com_speed_end = dstRun.com_speed_end;
            rec.fail_state_norm = dstRun.fail_state_norm;
            rec.fail_input_norm = dstRun.fail_input_norm;
            rec.fail_com_speed = dstRun.fail_com_speed;

            solverMat(i,j) = double(rec.solver_feasible);
            usableMat(i,j) = double(rec.usable_feasible);

            if dstRun.solver_feasible
                nodeRegistry = update_node_registry(nodeRegistry, dstPoint, 1, rec.usable_feasible);
            end

            edgeResults(edgeIdx) = rec;

            if ~dstRun.solver_feasible && cfg.print_failure_snapshot
                log_msg(fid, cfg, sprintf('PAIRWISE FAIL | src=%d -> dst=%d | reason=%s | fail_t=%.3f | exitflag=%g', ...
                    rec.source_node_id, rec.target_node_id, dstRun.fail_reason, dstRun.fail_time_s, dstRun.quadprog_exitflag));
            end

            if mod(edgeIdx, cfg.autosave_every) == 0 || edgeIdx == nEdges
                save(fullfile(runDir, ['transition_pairwise_edges_partial_' runStamp '.mat']), 'edgeResults', 'solverMat', 'usableMat', 'nodeRegistry');
            end
        end
    end

    edgeCsv = fullfile(runDir, ['transition_pairwise_edges_' runStamp '.csv']);
    edgeMat = fullfile(runDir, ['transition_pairwise_edges_' runStamp '.mat']);
    solverCsv = fullfile(runDir, ['transition_pairwise_solver_matrix_' runStamp '.csv']);
    usableCsv = fullfile(runDir, ['transition_pairwise_usable_matrix_' runStamp '.csv']);
    nodeCsv = fullfile(runDir, ['transition_pairwise_node_registry_' runStamp '.csv']);

    edgeTable = transition_records_to_table(edgeResults);
    writetable(edgeTable, edgeCsv);
    save(edgeMat, 'edgeResults', 'solverMat', 'usableMat', 'nodeRegistry');

    writematrix(solverMat, solverCsv);
    writematrix(usableMat, usableCsv);
    writetable(node_registry_to_table(nodeRegistry), nodeCsv);

    summary = struct();
    summary.n_seeds = nSeeds;
    summary.n_edges = nEdges;
    summary.source_table = source_rollout_to_table(sourceResults);
    summary.edge_table = edgeTable;
    summary.solver_matrix = solverMat;
    summary.usable_matrix = usableMat;
    summary.n_solver_feasible_edges = sum(edgeTable.solver_feasible);
    summary.n_usable_feasible_edges = sum(edgeTable.usable_feasible);

    log_msg(fid, cfg, sprintf('PAIRWISE SUMMARY | solver-feasible edges = %d | usable-feasible edges = %d', ...
        summary.n_solver_feasible_edges, summary.n_usable_feasible_edges));

    out = struct();
    out.nodeRegistry = nodeRegistry;
    out.summary = summary;
end

function out = run_frontier_stage(nodeRegistry, staticTable, cfg, fid, runDir, runStamp)
    sourceIdx = find(arrayfun(@(n) ~strcmp(n.quality_class, 'I'), nodeRegistry));
    sourceNodes = nodeRegistry(sourceIdx);

    candidates = build_candidate_targets(sourceNodes, cfg);
    candidates = remove_existing_nodes(candidates, nodeRegistry);

    log_msg(fid, cfg, sprintf('FRONTIER SUMMARY START | sources = %d | candidates = %d', numel(sourceNodes), numel(candidates)));

    nEdges = numel(sourceNodes) * numel(candidates);
    edgeResults(nEdges,1) = empty_transition_record();
    edgeIdx = 0;

    newNodeRegistry = nodeRegistry;

    for i = 1:numel(sourceNodes)
        srcPoint = node_to_point(sourceNodes(i));
        srcRun = run_mpc_chunk(srcPoint, cfg, []);

        if ~srcRun.solver_feasible
            continue
        end

        for j = 1:numel(candidates)
            edgeIdx = edgeIdx + 1;

            dstPoint = candidates(j);
            rec = empty_transition_record();
            rec.stage = "frontier";
            rec.source_node_id = sourceNodes(i).node_id;
            rec.source_key = string(sourceNodes(i).node_key);
            rec.target_node_id = NaN;
            rec.target_key = string(dstPoint.node_key);
            rec.source_rhoR = srcPoint.rhoR;
            rec.source_v = srcPoint.v_cmd;
            rec.source_a = srcPoint.a_cmd;
            rec.target_rhoR = dstPoint.rhoR;
            rec.target_v = dstPoint.v_cmd;
            rec.target_a = dstPoint.a_cmd;
            rec.delta_rhoR = rec.target_rhoR - rec.source_rhoR;
            rec.delta_v = rec.target_v - rec.source_v;
            rec.delta_a = rec.target_a - rec.source_a;

            dstStatic = lookup_static_status(staticTable, dstPoint, cfg);
            rec.target_static_solver = dstStatic.static_solver;
            rec.target_static_usable = dstStatic.static_usable;

            dstRun = run_mpc_chunk(dstPoint, cfg, srcRun.terminal_ctx);

            rec.solver_feasible = dstRun.solver_feasible;
            rec.usable_feasible = dstRun.usable_feasible;
            rec.success_level = classify_success_level(dstRun.solver_feasible, dstRun.usable_feasible);
            rec.transition_relation = classify_transition_relation(rec.target_static_solver, rec.solver_feasible);
            rec.fail_reason = string(dstRun.fail_reason);
            rec.fail_time_s = dstRun.fail_time_s;
            rec.fail_iter = dstRun.fail_iter;
            rec.quadprog_exitflag = dstRun.quadprog_exitflag;
            rec.tracking_error_mean = dstRun.tracking_error_mean;
            rec.control_effort_mean = dstRun.control_effort_mean;
            rec.Ieq_A = dstRun.Ieq_A;
            rec.soc_end_pct = dstRun.soc_end_pct;
            rec.com_speed_end = dstRun.com_speed_end;
            rec.fail_state_norm = dstRun.fail_state_norm;
            rec.fail_input_norm = dstRun.fail_input_norm;
            rec.fail_com_speed = dstRun.fail_com_speed;

            if dstRun.solver_feasible
                newNodeRegistry = update_node_registry(newNodeRegistry, dstPoint, 1, dstRun.usable_feasible);
                rec.target_node_id = find_node_id(newNodeRegistry, dstPoint.node_key);
            end

            edgeResults(edgeIdx) = rec;

            if mod(edgeIdx, cfg.autosave_every) == 0 || edgeIdx == nEdges
                save(fullfile(runDir, ['transition_frontier_partial_' runStamp '.mat']), 'edgeResults', 'newNodeRegistry');
            end
        end
    end

    edgeResults = edgeResults(1:edgeIdx);
    edgeTable = transition_records_to_table(edgeResults);

    edgeCsv = fullfile(runDir, ['transition_frontier_edges_' runStamp '.csv']);
    edgeMat = fullfile(runDir, ['transition_frontier_edges_' runStamp '.mat']);
    nodeCsv = fullfile(runDir, ['transition_frontier_node_registry_' runStamp '.csv']);

    writetable(edgeTable, edgeCsv);
    save(edgeMat, 'edgeResults', 'newNodeRegistry');
    writetable(node_registry_to_table(newNodeRegistry), nodeCsv);

    summary = struct();
    summary.edge_table = edgeTable;
    summary.n_edges = height(edgeTable);
    summary.n_solver_feasible_edges = sum(edgeTable.solver_feasible);
    summary.n_usable_feasible_edges = sum(edgeTable.usable_feasible);

    log_msg(fid, cfg, sprintf('FRONTIER SUMMARY END | tested edges = %d | solver-feasible = %d | usable-feasible = %d', ...
        summary.n_edges, summary.n_solver_feasible_edges, summary.n_usable_feasible_edges));

    out = struct();
    out.nodeRegistry = newNodeRegistry;
    out.summary = summary;
end

function out = run_adaptive_stage(nodeRegistry, staticTable, cfg, fid, runDir, runStamp)
    newNodeRegistry = nodeRegistry;
    allEdgeResults = empty_transition_record();
    allEdgeResults = allEdgeResults([]);

    currentDepth = 1;
    frontierKeys = get_frontier_keys(newNodeRegistry, currentDepth);

    while currentDepth <= cfg.adaptive_max_depth && ~isempty(frontierKeys)
        log_msg(fid, cfg, sprintf('ADAPTIVE DEPTH %d START | frontier nodes = %d', currentDepth, numel(frontierKeys)));

        srcMask = arrayfun(@(n) any(strcmp(string(n.node_key), frontierKeys)), newNodeRegistry);
        sourceNodes = newNodeRegistry(srcMask);

        edgeResults = empty_transition_record();
        edgeResults = edgeResults([]);

        newKeysThisDepth = strings(0,1);

        for i = 1:numel(sourceNodes)
            srcPoint = node_to_point(sourceNodes(i));
            srcRun = run_mpc_chunk(srcPoint, cfg, []);

            if ~srcRun.solver_feasible
                continue
            end

            candidates = build_candidate_targets(sourceNodes(i), cfg);
            candidates = remove_existing_nodes(candidates, newNodeRegistry);

            if numel(candidates) > cfg.adaptive_max_new_nodes_per_depth
                candidates = candidates(1:cfg.adaptive_max_new_nodes_per_depth);
            end

            for j = 1:numel(candidates)
                dstPoint = candidates(j);

                rec = empty_transition_record();
                rec.stage = "adaptive";
                rec.depth = currentDepth + 1;
                rec.source_node_id = sourceNodes(i).node_id;
                rec.source_key = string(sourceNodes(i).node_key);
                rec.target_key = string(dstPoint.node_key);
                rec.source_rhoR = srcPoint.rhoR;
                rec.source_v = srcPoint.v_cmd;
                rec.source_a = srcPoint.a_cmd;
                rec.target_rhoR = dstPoint.rhoR;
                rec.target_v = dstPoint.v_cmd;
                rec.target_a = dstPoint.a_cmd;
                rec.delta_rhoR = rec.target_rhoR - rec.source_rhoR;
                rec.delta_v = rec.target_v - rec.source_v;
                rec.delta_a = rec.target_a - rec.source_a;

                dstStatic = lookup_static_status(staticTable, dstPoint, cfg);
                rec.target_static_solver = dstStatic.static_solver;
                rec.target_static_usable = dstStatic.static_usable;

                dstRun = run_mpc_chunk(dstPoint, cfg, srcRun.terminal_ctx);

                rec.solver_feasible = dstRun.solver_feasible;
                rec.usable_feasible = dstRun.usable_feasible;
                rec.success_level = classify_success_level(dstRun.solver_feasible, dstRun.usable_feasible);
                rec.transition_relation = classify_transition_relation(rec.target_static_solver, rec.solver_feasible);
                rec.fail_reason = string(dstRun.fail_reason);
                rec.fail_time_s = dstRun.fail_time_s;
                rec.fail_iter = dstRun.fail_iter;
                rec.quadprog_exitflag = dstRun.quadprog_exitflag;
                rec.tracking_error_mean = dstRun.tracking_error_mean;
                rec.control_effort_mean = dstRun.control_effort_mean;
                rec.Ieq_A = dstRun.Ieq_A;
                rec.soc_end_pct = dstRun.soc_end_pct;
                rec.com_speed_end = dstRun.com_speed_end;
                rec.fail_state_norm = dstRun.fail_state_norm;
                rec.fail_input_norm = dstRun.fail_input_norm;
                rec.fail_com_speed = dstRun.fail_com_speed;

                if dstRun.solver_feasible
                    newNodeRegistry = update_node_registry(newNodeRegistry, dstPoint, currentDepth + 1, dstRun.usable_feasible);
                    rec.target_node_id = find_node_id(newNodeRegistry, dstPoint.node_key);
                    newKeysThisDepth(end+1,1) = string(dstPoint.node_key); %#ok<AGROW>
                end

                edgeResults(end+1,1) = rec; %#ok<AGROW>
            end
        end

        levelCsv = fullfile(runDir, sprintf('transition_adaptive_depth_%02d_%s.csv', currentDepth + 1, runStamp));
        levelMat = fullfile(runDir, sprintf('transition_adaptive_depth_%02d_%s.mat', currentDepth + 1, runStamp));

        edgeTable = transition_records_to_table(edgeResults);
        writetable(edgeTable, levelCsv);
        save(levelMat, 'edgeResults', 'newNodeRegistry');

        if isempty(allEdgeResults)
            allEdgeResults = edgeResults;
        else
            allEdgeResults = [allEdgeResults; edgeResults]; %#ok<AGROW>
        end

        log_msg(fid, cfg, sprintf('ADAPTIVE DEPTH %d END | tested edges = %d | new solver-reachable keys = %d', ...
            currentDepth + 1, height(edgeTable), numel(unique(newKeysThisDepth))));

        frontierKeys = unique(newKeysThisDepth);
        currentDepth = currentDepth + 1;
    end

    allEdgeCsv = fullfile(runDir, ['transition_adaptive_all_edges_' runStamp '.csv']);
    allEdgeMat = fullfile(runDir, ['transition_adaptive_all_edges_' runStamp '.mat']);
    nodeCsv = fullfile(runDir, ['transition_adaptive_node_registry_' runStamp '.csv']);

    allEdgeTable = transition_records_to_table(allEdgeResults);
    writetable(allEdgeTable, allEdgeCsv);
    save(allEdgeMat, 'allEdgeResults', 'newNodeRegistry');
    writetable(node_registry_to_table(newNodeRegistry), nodeCsv);

    summary = struct();
    summary.edge_table = allEdgeTable;
    summary.n_edges = height(allEdgeTable);
    summary.n_solver_feasible_edges = sum(allEdgeTable.solver_feasible);
    summary.n_usable_feasible_edges = sum(allEdgeTable.usable_feasible);

    out = struct();
    out.nodeRegistry = newNodeRegistry;
    out.summary = summary;
end

function run = run_mpc_chunk(point, cfg, initCtx)
    gait = cfg.gait;
    rlCfg = cfg.rlCfg;

    p = get_params(gait);
    p.R = point.rhoR * p.R;
    p.vel_d = [point.v_cmd; 0];
    p.acc_d = point.a_cmd;
    p.yaw_d = 0;

    dt_sim = p.simTimeStep;
    maxIter = floor(cfg.chunk_duration / dt_sim);
    qp_options = optimoptions('quadprog', 'Display', 'off');

    if isempty(initCtx)
        if gait == 1
            [p, Xt, Ut] = fcn_bound_ref_traj(p);
        else
            [Xt, Ut] = fcn_gen_XdUd(0, [], [1;1;1;1], p);
        end
        tstart = 0;
        batteryInit = make_battery_init(rlCfg);
        if cfg.compute_current_battery
            kneeTpl = load_knee_template(rlCfg.PROXY.kneeCsv, rlCfg.PROXY.betaMc);
            Dmc = readmatrix(rlCfg.PROXY.kneeCsv);
            Tmc = Dmc(end,1) - Dmc(1,1);
            kneeParams = make_knee_params(rlCfg, Tmc);
            kneeState = init_knee_proxy_state();
            Ihip4 = joint_torque_to_current(rlCfg.PROXY.tauHip4_joint, rlCfg.PROXY.Nhip, rlCfg.PROXY.eta, rlCfg.PROXY.Kt);
        else
            kneeTpl = [];
            kneeParams = struct();
            kneeState = [];
            Ihip4 = NaN;
        end
    else
        Xt = initCtx.Xt;
        Ut = initCtx.Ut;
        tstart = initCtx.t_abs_end;
        batteryInit = initCtx.battery;
        if cfg.compute_current_battery
            kneeTpl = initCtx.kneeTpl;
            kneeParams = initCtx.kneeParams;
            kneeState = initCtx.kneeState;
            Ihip4 = initCtx.Ihip4;
        else
            kneeTpl = [];
            kneeParams = struct();
            kneeState = [];
            Ihip4 = NaN;
        end
    end

    tracking_error = nan(maxIter, 1);
    control_effort = nan(maxIter, 1);
    com_speed = nan(maxIter, 1);
    state_norm = nan(maxIter, 1);
    input_norm = nan(maxIter, 1);
    qp_exitflag = nan(maxIter, 1);

    if cfg.compute_current_battery
        knee_t = nan(maxIter, 1);
        knee_I4 = nan(maxIter, 1);
        knee_tau4 = nan(maxIter, 1);
        Tst_log = nan(maxIter, 1);
        Tsw_log = nan(maxIter, 1);
    else
        knee_t = [];
        knee_I4 = [];
        knee_tau4 = [];
        Tst_log = [];
        Tsw_log = [];
    end

    failSnapshot = empty_fail_snapshot();
    lastSuccess = empty_success_snapshot();

    solver_feasible = true;
    fail_reason = 'none';
    fail_iter = NaN;
    fail_time_s = NaN;
    quadprog_exitflag_last = NaN;

    t0 = tic;
    tend = tstart + dt_sim;

    try
        for ii = 1:maxIter
            Xt_before = Xt;
            Ut_before = Ut;

            t_hor = tstart + p.Tmpc * (0:p.predHorizon-1);

            if gait == 1
                [FSM, Xd, Ud, Xt] = fcn_FSM_bound(t_hor, Xt, p);
            else
                [FSM, Xd, Ud, Xt] = fcn_FSM(t_hor, Xt, p);
            end

            if cfg.compute_current_battery
                [kneeState, kneeOut] = knee_proxy_step(tstart, FSM(1), kneeState, kneeTpl, kneeParams);
                knee_t(ii) = kneeOut.t;
                knee_tau4(ii) = kneeOut.tau4;
                knee_I4(ii) = kneeOut.I4;
                if isfinite(kneeState.Tst)
                    Tst_log(ii) = kneeState.Tst;
                end
                if isfinite(kneeState.Tsw)
                    Tsw_log(ii) = kneeState.Tsw;
                end
            end

            [H, g, Aineq, bineq, Aeq, beq] = fcn_get_QP_form_eta(Xt, Ut, Xd, Ud, p);
            H = (H + H') / 2;

            qpDiag = compute_qp_diag(H, g, Aineq, bineq, Aeq, beq);

            [zval, ~, exitflag] = quadprog(H, g, Aineq, bineq, Aeq, beq, [], [], [], qp_options);
            qp_exitflag(ii) = exitflag;
            quadprog_exitflag_last = exitflag;

            if exitflag <= 0 || isempty(zval)
                solver_feasible = false;
                fail_reason = 'quadprog';
                fail_iter = ii;
                fail_time_s = tstart;
                failSnapshot = build_fail_snapshot(ii, tstart, Xt_before, Ut_before, Xd, Ud, FSM, ...
                    H, g, Aineq, bineq, Aeq, beq, qpDiag, Tst_log, Tsw_log, ii, cfg, 'quadprog', '');
                break
            end

            ineq_margin_min = NaN;
            eq_residual_norm = NaN;
            if ~isempty(Aineq)
                ineq_margin_min = min(bineq - Aineq * zval);
            end
            if ~isempty(Aeq)
                eq_residual_norm = norm(Aeq * zval - beq);
            end

            Ut = Ut + zval(1:12);

            [u_ext, p_ext] = fcn_get_disturbance(tstart, p);
            p.p_ext = p_ext;
            u_ext = 0 * u_ext;

            Xdot_before = dynamics_SRB(tstart, Xt, Ut, Xd, u_ext, p);
            [~, X_chunk] = ode45(@(t, X) dynamics_SRB(t, X, Ut, Xd, u_ext, p), [tstart, tend], Xt);
            Xt_after = X_chunk(end,:).';

            if any(isnan(Xt_after)) || any(isinf(Xt_after))
                solver_feasible = false;
                fail_reason = 'state_invalid';
                fail_iter = ii;
                fail_time_s = tend;
                failSnapshot = build_fail_snapshot(ii, tend, Xt, Ut, Xd, Ud, FSM, ...
                    H, g, Aineq, bineq, Aeq, beq, qpDiag, Tst_log, Tsw_log, ii, cfg, 'state_invalid', '');
                failSnapshot.Xt_after_invalid = Xt_after;
                failSnapshot.Xdot_before = Xdot_before;
                break
            end

            Xt = Xt_after;

            tracking_error(ii) = sum((Xt - Xd(:,1)).^2);
            control_effort(ii) = sum(Ut.^2);
            state_norm(ii) = norm(Xt);
            input_norm(ii) = norm(Ut);
            com_speed(ii) = norm(Xt(4:5));

            lastSuccess = build_success_snapshot(ii, tstart, Xt, Ut, Xd, Ud, FSM, qpDiag, zval, ...
                ineq_margin_min, eq_residual_norm, cfg, Tst_log, Tsw_log);

            tstart = tend;
            tend = tstart + dt_sim;
        end
    catch ME
        solver_feasible = false;
        fail_reason = 'exception';
        fail_iter = ii_if_defined();
        fail_time_s = tstart;
        failSnapshot = build_fail_snapshot(ii_if_defined(), tstart, Xt, Ut, nan_safe_col(Xt), nan_safe_col(Ut), NaN, ...
            [], [], [], [], [], [], empty_qp_diag(), Tst_log, Tsw_log, ii_if_defined(), cfg, 'exception', ME.message);
    end

    validDyn = isfinite(tracking_error);
    if any(validDyn)
        tracking_error_mean = mean(tracking_error(validDyn));
        control_effort_mean = mean(control_effort(validDyn));
        com_speed_end = com_speed(find(validDyn, 1, 'last'));
    else
        tracking_error_mean = NaN;
        control_effort_mean = NaN;
        com_speed_end = norm(Xt(4:5));
    end

    if cfg.compute_current_battery
        [Ieq_A, soc_end_pct, batteryOut, total_I_peak] = postprocess_current_battery(knee_t, knee_I4, knee_tau4, Ihip4, Tst_log, Tsw_log, rlCfg, batteryInit);
    else
        Ieq_A = NaN;
        soc_end_pct = NaN;
        batteryOut = batteryInit;
        total_I_peak = NaN;
    end

    usable_feasible = solver_feasible ...
        && isfinite(Ieq_A) ...
        && isfinite(soc_end_pct) ...
        && isfinite(tracking_error_mean) ...
        && tracking_error_mean <= cfg.usable_tracking_threshold ...
        && control_effort_mean <= cfg.usable_effort_threshold;

    terminal_ctx = struct();
    terminal_ctx.Xt = Xt;
    terminal_ctx.Ut = Ut;
    terminal_ctx.t_abs_end = tstart;
    terminal_ctx.battery = batteryOut;
    terminal_ctx.kneeState = kneeState;
    terminal_ctx.kneeTpl = kneeTpl;
    terminal_ctx.kneeParams = kneeParams;
    terminal_ctx.Ihip4 = Ihip4;

    run = struct();
    run.solver_feasible = solver_feasible;
    run.usable_feasible = usable_feasible;
    run.fail_reason = fail_reason;
    run.fail_iter = fail_iter;
    run.fail_time_s = fail_time_s;
    run.quadprog_exitflag = quadprog_exitflag_last;
    run.tracking_error_mean = tracking_error_mean;
    run.control_effort_mean = control_effort_mean;
    run.Ieq_A = Ieq_A;
    run.soc_end_pct = soc_end_pct;
    run.total_I_peak = total_I_peak;
    run.com_speed_end = com_speed_end;
    run.fail_state_norm = failSnapshot.state_norm;
    run.fail_input_norm = failSnapshot.input_norm;
    run.fail_com_speed = failSnapshot.com_speed;
    run.last_success_snapshot = lastSuccess;
    run.fail_snapshot = failSnapshot;
    run.terminal_ctx = terminal_ctx;
    run.elapsed_sec = toc(t0);

    function out = ii_if_defined()
        try
            out = ii;
        catch
            out = NaN;
        end
    end
end

function [Ieq_A, soc_end_pct, batteryOut, total_I_peak] = postprocess_current_battery(knee_t, knee_I4, knee_tau4, Ihip4, Tst_log, Tsw_log, rlCfg, batteryInit)
    Ieq_A = NaN;
    soc_end_pct = NaN;
    total_I_peak = NaN;
    batteryOut = batteryInit;

    validCurrent = isfinite(knee_t) & isfinite(knee_I4);
    tCurrent = knee_t(validCurrent);
    I_knee = knee_I4(validCurrent);
    tau_knee = knee_tau4(validCurrent); %#ok<NASGU>

    if numel(tCurrent) < 2
        return
    end

    [tCurrent, ord] = sort(tCurrent);
    I_knee = I_knee(ord);

    [tCurrent, uniqIdx] = unique(tCurrent);
    I_knee = I_knee(uniqIdx);

    I_total = I_knee + Ihip4;
    total_I_peak = max(abs(I_total));
    current_duration_s = tCurrent(end) - tCurrent(1);

    if current_duration_s > 0
        charge_total_As = trapz(tCurrent, abs(I_total));
        Ieq_A = charge_total_As / current_duration_s;
    end

    batteryOut = evaluate_battery_feedback(tCurrent, I_total, rlCfg.BATTERY, batteryInit);

    if isfield(batteryOut, 'soc_pct')
        soc_end_pct = batteryOut.soc_pct;
    end
end

function batteryInit = make_battery_init(rlCfg)
    batteryInit = struct();
    batteryInit.metric_type = rlCfg.BATTERY.metric_type;
    batteryInit.metric_value = rlCfg.BATTERY.metric_init;
    batteryInit.margin_norm = rlCfg.BATTERY.SOC_init;
    batteryInit.soc_pct = 100 * rlCfg.BATTERY.SOC_init;
    batteryInit.n_series = rlCfg.BATTERY.n_series;
    batteryInit.n_parallel = rlCfg.BATTERY.n_parallel;
end

function kneeParams = make_knee_params(rlCfg, Tmc)
    kneeParams = struct();
    kneeParams.Tmc = Tmc;
    kneeParams.alpha = rlCfg.PROXY.alpha;
    kneeParams.beta = rlCfg.PROXY.beta;
    kneeParams.Kt = rlCfg.PROXY.Kt;
    kneeParams.eta = rlCfg.PROXY.eta;
    kneeParams.Nknee = rlCfg.PROXY.Nknee;
end

function rec = empty_transition_record()
    rec = struct( ...
        'stage', "", ...
        'depth', NaN, ...
        'source_node_id', NaN, ...
        'target_node_id', NaN, ...
        'source_key', "", ...
        'target_key', "", ...
        'source_rhoR', NaN, ...
        'source_v', NaN, ...
        'source_a', NaN, ...
        'target_rhoR', NaN, ...
        'target_v', NaN, ...
        'target_a', NaN, ...
        'delta_rhoR', NaN, ...
        'delta_v', NaN, ...
        'delta_a', NaN, ...
        'target_static_solver', false, ...
        'target_static_usable', false, ...
        'solver_feasible', false, ...
        'usable_feasible', false, ...
        'success_level', "", ...
        'transition_relation', "", ...
        'fail_reason', "", ...
        'fail_time_s', NaN, ...
        'fail_iter', NaN, ...
        'quadprog_exitflag', NaN, ...
        'tracking_error_mean', NaN, ...
        'control_effort_mean', NaN, ...
        'Ieq_A', NaN, ...
        'soc_end_pct', NaN, ...
        'com_speed_end', NaN, ...
        'fail_state_norm', NaN, ...
        'fail_input_norm', NaN, ...
        'fail_com_speed', NaN);
end

function T = transition_records_to_table(recs)
    if isempty(recs)
        T = table();
        return
    end

    n = numel(recs);

    stage = strings(n,1);
    depth = nan(n,1);
    source_node_id = nan(n,1);
    target_node_id = nan(n,1);
    source_key = strings(n,1);
    target_key = strings(n,1);
    source_rhoR = nan(n,1);
    source_v = nan(n,1);
    source_a = nan(n,1);
    target_rhoR = nan(n,1);
    target_v = nan(n,1);
    target_a = nan(n,1);
    delta_rhoR = nan(n,1);
    delta_v = nan(n,1);
    delta_a = nan(n,1);
    target_static_solver = false(n,1);
    target_static_usable = false(n,1);
    solver_feasible = false(n,1);
    usable_feasible = false(n,1);
    success_level = strings(n,1);
    transition_relation = strings(n,1);
    fail_reason = strings(n,1);
    fail_time_s = nan(n,1);
    fail_iter = nan(n,1);
    quadprog_exitflag = nan(n,1);
    tracking_error_mean = nan(n,1);
    control_effort_mean = nan(n,1);
    Ieq_A = nan(n,1);
    soc_end_pct = nan(n,1);
    com_speed_end = nan(n,1);
    fail_state_norm = nan(n,1);
    fail_input_norm = nan(n,1);
    fail_com_speed = nan(n,1);

    for i = 1:n
        r = recs(i);
        stage(i) = string(r.stage);
        depth(i) = r.depth;
        source_node_id(i) = r.source_node_id;
        target_node_id(i) = r.target_node_id;
        source_key(i) = string(r.source_key);
        target_key(i) = string(r.target_key);
        source_rhoR(i) = r.source_rhoR;
        source_v(i) = r.source_v;
        source_a(i) = r.source_a;
        target_rhoR(i) = r.target_rhoR;
        target_v(i) = r.target_v;
        target_a(i) = r.target_a;
        delta_rhoR(i) = r.delta_rhoR;
        delta_v(i) = r.delta_v;
        delta_a(i) = r.delta_a;
        target_static_solver(i) = r.target_static_solver;
        target_static_usable(i) = r.target_static_usable;
        solver_feasible(i) = r.solver_feasible;
        usable_feasible(i) = r.usable_feasible;
        success_level(i) = string(r.success_level);
        transition_relation(i) = string(r.transition_relation);
        fail_reason(i) = string(r.fail_reason);
        fail_time_s(i) = r.fail_time_s;
        fail_iter(i) = r.fail_iter;
        quadprog_exitflag(i) = r.quadprog_exitflag;
        tracking_error_mean(i) = r.tracking_error_mean;
        control_effort_mean(i) = r.control_effort_mean;
        Ieq_A(i) = r.Ieq_A;
        soc_end_pct(i) = r.soc_end_pct;
        com_speed_end(i) = r.com_speed_end;
        fail_state_norm(i) = r.fail_state_norm;
        fail_input_norm(i) = r.fail_input_norm;
        fail_com_speed(i) = r.fail_com_speed;
    end

    T = table(stage, depth, source_node_id, target_node_id, source_key, target_key, ...
        source_rhoR, source_v, source_a, target_rhoR, target_v, target_a, ...
        delta_rhoR, delta_v, delta_a, target_static_solver, target_static_usable, ...
        solver_feasible, usable_feasible, success_level, transition_relation, fail_reason, ...
        fail_time_s, fail_iter, quadprog_exitflag, tracking_error_mean, control_effort_mean, ...
        Ieq_A, soc_end_pct, com_speed_end, fail_state_norm, fail_input_norm, fail_com_speed);
end

function s = empty_source_rollout()
    s = struct( ...
        'source_node_id', NaN, ...
        'source_key', "", ...
        'source_rhoR', NaN, ...
        'source_v', NaN, ...
        'source_a', NaN, ...
        'source_solver_feasible', false, ...
        'source_usable_feasible', false, ...
        'tracking_error_mean', NaN, ...
        'control_effort_mean', NaN, ...
        'Ieq_A', NaN, ...
        'soc_end_pct', NaN, ...
        'terminal_ctx', struct());
end

function s = source_rollout_from_chunk(seedRow, run)
    s = empty_source_rollout();
    s.source_node_id = seedRow.node_id;
    s.source_key = seedRow.node_key;
    s.source_rhoR = seedRow.rhoR;
    s.source_v = seedRow.v_cmd;
    s.source_a = seedRow.a_cmd;
    s.source_solver_feasible = run.solver_feasible;
    s.source_usable_feasible = run.usable_feasible;
    s.tracking_error_mean = run.tracking_error_mean;
    s.control_effort_mean = run.control_effort_mean;
    s.Ieq_A = run.Ieq_A;
    s.soc_end_pct = run.soc_end_pct;
    s.terminal_ctx = run.terminal_ctx;
end

function T = source_rollout_to_table(src)
    n = numel(src);
    source_node_id = nan(n,1);
    source_key = strings(n,1);
    source_rhoR = nan(n,1);
    source_v = nan(n,1);
    source_a = nan(n,1);
    source_solver_feasible = false(n,1);
    source_usable_feasible = false(n,1);
    tracking_error_mean = nan(n,1);
    control_effort_mean = nan(n,1);
    Ieq_A = nan(n,1);
    soc_end_pct = nan(n,1);

    for i = 1:n
        source_node_id(i) = src(i).source_node_id;
        source_key(i) = string(src(i).source_key);
        source_rhoR(i) = src(i).source_rhoR;
        source_v(i) = src(i).source_v;
        source_a(i) = src(i).source_a;
        source_solver_feasible(i) = src(i).source_solver_feasible;
        source_usable_feasible(i) = src(i).source_usable_feasible;
        tracking_error_mean(i) = src(i).tracking_error_mean;
        control_effort_mean(i) = src(i).control_effort_mean;
        Ieq_A(i) = src(i).Ieq_A;
        soc_end_pct(i) = src(i).soc_end_pct;
    end

    T = table(source_node_id, source_key, source_rhoR, source_v, source_a, ...
        source_solver_feasible, source_usable_feasible, tracking_error_mean, control_effort_mean, Ieq_A, soc_end_pct);
end

function nodeRegistry = update_node_registry(nodeRegistry, point, depth, usable_feasible)
    key = char(point.node_key);
    idx = find(strcmp({nodeRegistry.node_key}, key), 1);

    if isempty(idx)
        newEntry = empty_node_entry();
        newEntry.node_id = next_node_id(nodeRegistry);
        newEntry.node_key = key;
        newEntry.rhoR = point.rhoR;
        newEntry.v_cmd = point.v_cmd;
        newEntry.a_cmd = point.a_cmd;
        newEntry.min_depth = depth;
        if depth == 1
            newEntry.node_class = 'T';
        else
            newEntry.node_class = 'M';
        end
        newEntry.ever_solver_feasible = true;
        if usable_feasible
            newEntry.ever_usable_feasible = true;
            newEntry.quality_class = 'P';
        else
            newEntry.quality_class = 'Q';
        end
        nodeRegistry(end+1,1) = newEntry;
    else
        nodeRegistry(idx).ever_solver_feasible = true;
        nodeRegistry(idx).min_depth = min(nodeRegistry(idx).min_depth, depth);
        if ~nodeRegistry(idx).is_static_seed
            if nodeRegistry(idx).min_depth <= 1
                nodeRegistry(idx).node_class = 'T';
            else
                nodeRegistry(idx).node_class = 'M';
            end
        end
        if usable_feasible
            nodeRegistry(idx).ever_usable_feasible = true;
            nodeRegistry(idx).quality_class = 'P';
        elseif ~nodeRegistry(idx).ever_usable_feasible
            nodeRegistry(idx).quality_class = 'Q';
        end
    end
end

function id = find_node_id(nodeRegistry, nodeKey)
    idx = find(strcmp({nodeRegistry.node_key}, char(nodeKey)), 1);
    if isempty(idx)
        id = NaN;
    else
        id = nodeRegistry(idx).node_id;
    end
end

function id = next_node_id(nodeRegistry)
    ids = [nodeRegistry.node_id];
    if isempty(ids)
        id = 1;
    else
        id = max(ids) + 1;
    end
end

function T = node_registry_to_table(nodeRegistry)
    n = numel(nodeRegistry);
    node_id = nan(n,1);
    node_key = strings(n,1);
    rhoR = nan(n,1);
    v_cmd = nan(n,1);
    a_cmd = nan(n,1);
    is_static_seed = false(n,1);
    static_solver_feasible = false(n,1);
    static_usable = false(n,1);
    ever_solver_feasible = false(n,1);
    ever_usable_feasible = false(n,1);
    min_depth = nan(n,1);
    node_class = strings(n,1);
    quality_class = strings(n,1);

    for i = 1:n
        node_id(i) = nodeRegistry(i).node_id;
        node_key(i) = string(nodeRegistry(i).node_key);
        rhoR(i) = nodeRegistry(i).rhoR;
        v_cmd(i) = nodeRegistry(i).v_cmd;
        a_cmd(i) = nodeRegistry(i).a_cmd;
        is_static_seed(i) = nodeRegistry(i).is_static_seed;
        static_solver_feasible(i) = nodeRegistry(i).static_solver_feasible;
        static_usable(i) = nodeRegistry(i).static_usable;
        ever_solver_feasible(i) = nodeRegistry(i).ever_solver_feasible;
        ever_usable_feasible(i) = nodeRegistry(i).ever_usable_feasible;
        min_depth(i) = nodeRegistry(i).min_depth;
        node_class(i) = string(nodeRegistry(i).node_class);
        quality_class(i) = string(nodeRegistry(i).quality_class);
    end

    T = table(node_id, node_key, rhoR, v_cmd, a_cmd, is_static_seed, static_solver_feasible, static_usable, ...
        ever_solver_feasible, ever_usable_feasible, min_depth, node_class, quality_class);
    T = sortrows(T, {'min_depth','v_cmd','a_cmd','rhoR'});
end

function n = empty_node_entry()
    n = struct( ...
        'node_id', NaN, ...
        'node_key', '', ...
        'rhoR', NaN, ...
        'v_cmd', NaN, ...
        'a_cmd', NaN, ...
        'is_static_seed', false, ...
        'static_solver_feasible', false, ...
        'static_usable', false, ...
        'ever_solver_feasible', false, ...
        'ever_usable_feasible', false, ...
        'min_depth', inf, ...
        'node_class', 'U', ...
        'quality_class', 'I');
end

function status = lookup_static_status(caseTable, point, cfg)
    tolR = 1e-9;
    tolV = 1e-9;
    tolA = 1e-9;

    mask = abs(caseTable.rhoR - point.rhoR) < tolR ...
        & abs(caseTable.v_cmd - point.v_cmd) < tolV ...
        & abs(caseTable.a_cmd - point.a_cmd) < tolA;

    status = struct();
    status.static_solver = false;
    status.static_usable = false;

    if any(mask)
        row = caseTable(find(mask, 1, 'first'), :);
        status.static_solver = logical(row.feasible);
        status.static_usable = is_usable_row(row, cfg);
    end
end

function tf = is_usable_row(row, cfg)
    tf = logical(row.feasible) ...
        && ismember('Ieq_A', row.Properties.VariableNames) && isfinite(row.Ieq_A) ...
        && ismember('soc_end_pct', row.Properties.VariableNames) && isfinite(row.soc_end_pct) ...
        && ismember('tracking_error_mean', row.Properties.VariableNames) && isfinite(row.tracking_error_mean) ...
        && row.tracking_error_mean <= cfg.usable_tracking_threshold ...
        && (~ismember('control_effort_mean', row.Properties.VariableNames) || row.control_effort_mean <= cfg.usable_effort_threshold);
end

function level = classify_success_level(solver_feasible, usable_feasible)
    if usable_feasible
        level = "P";
    elseif solver_feasible
        level = "Q";
    else
        level = "I";
    end
end

function rel = classify_transition_relation(target_static_solver, transition_solver)
    if target_static_solver && transition_solver
        rel = "static_and_transition_ok";
    elseif target_static_solver && ~transition_solver
        rel = "static_ok_transition_fail";
    elseif ~target_static_solver && transition_solver
        rel = "dynamic_only_reachable";
    else
        rel = "both_fail";
    end
end

function point = row_to_point(row)
    point = struct();
    point.rhoR = row.rhoR;
    point.v_cmd = row.v_cmd;
    point.a_cmd = row.a_cmd;
    if ismember('node_key', row.Properties.VariableNames)
        point.node_key = char(row.node_key);
    else
        point.node_key = make_node_key(point.rhoR, point.v_cmd, point.a_cmd);
    end
end

function point = node_to_point(node)
    point = struct();
    point.rhoR = node.rhoR;
    point.v_cmd = node.v_cmd;
    point.a_cmd = node.a_cmd;
    point.node_key = node.node_key;
end

function keys = get_frontier_keys(nodeRegistry, currentDepth)
    mask = arrayfun(@(n) n.min_depth == currentDepth && n.ever_solver_feasible, nodeRegistry);
    keys = string({nodeRegistry(mask).node_key}).';
end

function candidates = build_candidate_targets(sourceNodes, cfg)
    if numel(sourceNodes) == 1
        srcList = sourceNodes;
    else
        srcList = sourceNodes(:);
    end

    candMap = containers.Map();

    for i = 1:numel(srcList)
        src = srcList(i);

        if isstruct(src) && isfield(src, 'node_key')
            rho0 = src.rhoR;
            v0 = src.v_cmd;
            a0 = src.a_cmd;
        else
            rho0 = src.rhoR;
            v0 = src.v_cmd;
            a0 = src.a_cmd;
        end

        for dr = cfg.frontier_drho
            for dv = cfg.frontier_dv
                for da = cfg.frontier_da
                    if dr == 0 && dv == 0 && da == 0
                        continue
                    end

                    rho = snap_to_grid(rho0 + dr, cfg.target_grid_rho_step, cfg.frontier_rho_bounds);
                    v = snap_to_grid(v0 + dv, cfg.target_grid_v_step, cfg.frontier_v_bounds);
                    a = snap_to_grid(a0 + da, cfg.target_grid_a_step, cfg.frontier_a_bounds);

                    if isnan(rho) || isnan(v) || isnan(a)
                        continue
                    end

                    if cfg.frontier_use_feasible_va_mask
                        if ~is_valid_va(v, a, cfg)
                            continue
                        end
                    end

                    key = make_node_key(rho, v, a);
                    if ~isKey(candMap, key)
                        p = struct();
                        p.rhoR = rho;
                        p.v_cmd = v;
                        p.a_cmd = a;
                        p.node_key = key;
                        candMap(key) = p;
                    end
                end
            end
        end
    end

    keys = candMap.keys;
    candidates = repmat(struct('rhoR', NaN, 'v_cmd', NaN, 'a_cmd', NaN, 'node_key', ''), numel(keys), 1);
    for i = 1:numel(keys)
        candidates(i) = candMap(keys{i});
    end

    [~, ord] = sort(arrayfun(@(c) c.v_cmd*1000 + c.a_cmd*100 + c.rhoR, candidates));
    candidates = candidates(ord);
end

function candidates = remove_existing_nodes(candidates, nodeRegistry)
    if isempty(candidates)
        return
    end

    existingKeys = string({nodeRegistry.node_key});
    keep = true(numel(candidates),1);

    for i = 1:numel(candidates)
        keep(i) = ~any(existingKeys == string(candidates(i).node_key));
    end

    candidates = candidates(keep);
end

function tf = is_valid_va(v, a, cfg)
    aLo = max(cfg.frontier_a_bounds(1), v / cfg.tacc_max);
    aHi = min(cfg.frontier_a_bounds(2), v / cfg.tacc_min);
    tf = a >= aLo && a <= aHi;
end

function x = snap_to_grid(x, step, bounds)
    x = round(x / step) * step;
    if x < bounds(1) - 1e-12 || x > bounds(2) + 1e-12
        x = NaN;
    end
end

function key = make_node_key(rhoR, v, a)
    key = sprintf('%.3f|%.3f|%.3f', rhoR, v, a);
end

function staticFile = find_latest_static_csv()
    files = dir(fullfile(pwd, '**', 'static_sweep_cases_*.csv'));
    if isempty(files)
        staticFile = '';
        return
    end

    [~, idx] = max([files.datenum]);
    staticFile = fullfile(files(idx).folder, files(idx).name);
end

function rlCfg = build_fallback_rl_cfg(rootDir)
    rlCfg = struct();

    rlCfg.BATTERY.metric_type = 'soc';
    rlCfg.BATTERY.SOC_init = 0.95;
    rlCfg.BATTERY.metric_init = 100 * rlCfg.BATTERY.SOC_init;
    rlCfg.BATTERY.metric_min = 20;
    rlCfg.BATTERY.metric_max = 100;
    rlCfg.BATTERY.terminal_margin = 0.20;
    rlCfg.BATTERY.C_nom_Ah = 2.0;
    rlCfg.BATTERY.pack_voltage = 12;
    rlCfg.BATTERY.DoD = 0.8;
    rlCfg.BATTERY.use_pack_sizing = false;
    rlCfg.BATTERY.n_series = 4;
    rlCfg.BATTERY.n_parallel = 4;
    rlCfg.BATTERY.decim = 10;
    rlCfg.BATTERY.make_plots = false;

    rlCfg.PROXY.betaMc = 0.5;
    rlCfg.PROXY.kneeCsv = fullfile(rootDir, 'Symmetric Knee Torques.csv');
    rlCfg.PROXY.alpha = 1;
    rlCfg.PROXY.beta = 2;
    rlCfg.PROXY.Kt = 0.0909;
    rlCfg.PROXY.eta = 0.90;
    rlCfg.PROXY.Nknee = 6 / 1.55;
    rlCfg.PROXY.tauHip4_joint = 6.114;
    rlCfg.PROXY.Nhip = 6;
end

function qpDiag = compute_qp_diag(H, g, Aineq, bineq, Aeq, beq)
    qpDiag = empty_qp_diag();

    qpDiag.h_rcond = safe_rcond(H);
    qpDiag.h_fro = norm(H, 'fro');
    if isempty(H)
        qpDiag.h_min_diag = NaN;
        qpDiag.h_max_diag = NaN;
    else
        d = diag(H);
        qpDiag.h_min_diag = min(d);
        qpDiag.h_max_diag = max(d);
    end

    qpDiag.g_norm = norm(g);
    qpDiag.Aineq_rows = size(Aineq, 1);
    qpDiag.Aineq_cols = size(Aineq, 2);
    qpDiag.Aeq_rows = size(Aeq, 1);
    qpDiag.Aeq_cols = size(Aeq, 2);

    if isempty(Aineq)
        qpDiag.Aineq_fro = 0;
        qpDiag.bineq_norm = 0;
    else
        qpDiag.Aineq_fro = norm(Aineq, 'fro');
        qpDiag.bineq_norm = norm(bineq);
    end

    if isempty(Aeq)
        qpDiag.Aeq_fro = 0;
        qpDiag.beq_norm = 0;
    else
        qpDiag.Aeq_fro = norm(Aeq, 'fro');
        qpDiag.beq_norm = norm(beq);
    end
end

function s = build_fail_snapshot(iter, tFail, Xt, Ut, Xd, Ud, FSM, H, g, Aineq, bineq, Aeq, beq, qpDiag, Tst_log, Tsw_log, idx, cfg, reason, detail)
    s = empty_fail_snapshot();

    s.iter = iter;
    s.t_fail = tFail;
    s.reason = reason;
    s.detail = detail;
    s.fsm_leg1 = scalar_or_nan(FSM, 1);
    s.com_speed = norm_safe_rows(Xt, 4, 5);
    s.state_norm = norm(Xt);
    s.input_norm = norm(Ut);
    s.Tst = safe_idx(Tst_log, idx);
    s.Tsw = safe_idx(Tsw_log, idx);

    s.h_rcond = qpDiag.h_rcond;
    s.h_fro = qpDiag.h_fro;
    s.h_min_diag = qpDiag.h_min_diag;
    s.h_max_diag = qpDiag.h_max_diag;
    s.g_norm = qpDiag.g_norm;
    s.Aineq_rows = qpDiag.Aineq_rows;
    s.Aineq_cols = qpDiag.Aineq_cols;
    s.Aeq_rows = qpDiag.Aeq_rows;
    s.Aeq_cols = qpDiag.Aeq_cols;
    s.Aineq_fro = qpDiag.Aineq_fro;
    s.Aeq_fro = qpDiag.Aeq_fro;
    s.bineq_norm = qpDiag.bineq_norm;
    s.beq_norm = qpDiag.beq_norm;

    if cfg.save_fail_state_vectors
        s.Xt = Xt;
        s.Ut = Ut;
        s.Xd1 = first_col_or_vector(Xd);
        s.Ud1 = first_col_or_vector(Ud);
        s.FSM = FSM;
    end

    if cfg.save_full_qp_on_failure
        s.H = H;
        s.g = g;
        s.Aineq = Aineq;
        s.bineq = bineq;
        s.Aeq = Aeq;
        s.beq = beq;
    end
end

function s = build_success_snapshot(iter, tNow, Xt, Ut, Xd, Ud, FSM, qpDiag, zval, ineq_margin_min, eq_residual_norm, cfg, Tst_log, Tsw_log)
    s = empty_success_snapshot();

    s.iter = iter;
    s.t = tNow;
    s.fsm_leg1 = scalar_or_nan(FSM, 1);
    s.com_speed = norm_safe_rows(Xt, 4, 5);
    s.state_norm = norm(Xt);
    s.input_norm = norm(Ut);
    s.z_step_norm = norm(zval(1:12));
    s.ineq_margin_min = ineq_margin_min;
    s.eq_residual_norm = eq_residual_norm;
    s.Tst = safe_idx(Tst_log, iter);
    s.Tsw = safe_idx(Tsw_log, iter);

    s.h_rcond = qpDiag.h_rcond;
    s.h_fro = qpDiag.h_fro;
    s.h_min_diag = qpDiag.h_min_diag;
    s.h_max_diag = qpDiag.h_max_diag;
    s.g_norm = qpDiag.g_norm;
    s.Aineq_rows = qpDiag.Aineq_rows;
    s.Aeq_rows = qpDiag.Aeq_rows;

    if cfg.save_success_snapshots
        s.Xt = Xt;
        s.Ut = Ut;
        s.Xd1 = first_col_or_vector(Xd);
        s.Ud1 = first_col_or_vector(Ud);
        s.FSM = FSM;
    end
end

function s = empty_fail_snapshot()
    s = struct( ...
        'iter', NaN, ...
        't_fail', NaN, ...
        'reason', '', ...
        'detail', '', ...
        'fsm_leg1', NaN, ...
        'com_speed', NaN, ...
        'state_norm', NaN, ...
        'input_norm', NaN, ...
        'Tst', NaN, ...
        'Tsw', NaN, ...
        'h_rcond', NaN, ...
        'h_fro', NaN, ...
        'h_min_diag', NaN, ...
        'h_max_diag', NaN, ...
        'g_norm', NaN, ...
        'Aineq_rows', NaN, ...
        'Aineq_cols', NaN, ...
        'Aeq_rows', NaN, ...
        'Aeq_cols', NaN, ...
        'Aineq_fro', NaN, ...
        'Aeq_fro', NaN, ...
        'bineq_norm', NaN, ...
        'beq_norm', NaN, ...
        'Xt', [], ...
        'Ut', [], ...
        'Xd1', [], ...
        'Ud1', [], ...
        'FSM', [], ...
        'H', [], ...
        'g', [], ...
        'Aineq', [], ...
        'bineq', [], ...
        'Aeq', [], ...
        'beq', [], ...
        'Xt_after_invalid', [], ...
        'Xdot_before', []);
end

function s = empty_success_snapshot()
    s = struct( ...
        'iter', NaN, ...
        't', NaN, ...
        'fsm_leg1', NaN, ...
        'com_speed', NaN, ...
        'state_norm', NaN, ...
        'input_norm', NaN, ...
        'z_step_norm', NaN, ...
        'ineq_margin_min', NaN, ...
        'eq_residual_norm', NaN, ...
        'Tst', NaN, ...
        'Tsw', NaN, ...
        'h_rcond', NaN, ...
        'h_fro', NaN, ...
        'h_min_diag', NaN, ...
        'h_max_diag', NaN, ...
        'g_norm', NaN, ...
        'Aineq_rows', NaN, ...
        'Aeq_rows', NaN, ...
        'Xt', [], ...
        'Ut', [], ...
        'Xd1', [], ...
        'Ud1', [], ...
        'FSM', []);
end

function q = empty_qp_diag()
    q = struct( ...
        'h_rcond', NaN, ...
        'h_fro', NaN, ...
        'h_min_diag', NaN, ...
        'h_max_diag', NaN, ...
        'g_norm', NaN, ...
        'Aineq_rows', NaN, ...
        'Aineq_cols', NaN, ...
        'Aeq_rows', NaN, ...
        'Aeq_cols', NaN, ...
        'Aineq_fro', NaN, ...
        'Aeq_fro', NaN, ...
        'bineq_norm', NaN, ...
        'beq_norm', NaN);
end

function add_if_exists(folderPath)
    if exist(folderPath, 'dir')
        addpath(folderPath);
    end
end

function log_msg(fid, cfg, msg)
    line = sprintf('[%s] %s', datestr(now, 'yyyy-mm-dd HH:MM:SS'), msg);
    fprintf(fid, '%s\n', line);
    if cfg.print_to_cli
        fprintf('%s\n', line);
    end
end

function y = safe_rcond(H)
    try
        y = rcond(H);
    catch
        y = NaN;
    end
end

function y = scalar_or_nan(x, idx)
    try
        y = x(idx);
    catch
        y = NaN;
    end
end

function y = norm_safe_rows(x, i1, i2)
    try
        y = norm(x(i1:i2));
    catch
        y = NaN;
    end
end

function y = safe_idx(x, idx)
    try
        y = x(idx);
    catch
        y = NaN;
    end
end

function y = first_col_or_vector(x)
    if isempty(x)
        y = [];
    elseif isvector(x)
        y = x(:);
    else
        y = x(:,1);
    end
end

function y = nan_safe_col(x)
    if isempty(x)
        y = [];
    else
        y = nan(size(x(:)));
    end
end