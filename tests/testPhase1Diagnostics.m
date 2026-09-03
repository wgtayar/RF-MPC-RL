function tests = testPhase1Diagnostics
%testPhase1Diagnostics Regression tests for Phase 1 diagnostic helpers.
    tests = functiontests(localfunctions);
end

function setupOnce(~)
    testFile = which('testPhase1Diagnostics');
    addpath(fileparts(fileparts(testFile)));
    bootstrap_RF_MPC_RL();
end

function setup(~)
    clear fcn_FSM fcn_FSM_bound fcn_gen_XdUd dynamics_SRB
end

function testInequalityRowMapping(testCase)
    mapping = map_qp_inequality_rows([1; 6; 7; 24; 25; 144]);
    verifyEqual(testCase, mapping.horizon_step, [1; 1; 1; 1; 2; 6]);
    verifyEqual(testCase, mapping.leg, [1; 1; 2; 4; 1; 4]);
    verifyEqual(testCase, mapping.constraint_index, [1; 6; 1; 6; 1; 6]);
end

function testSuccessfulQpDiagnostics(testCase)
    z = [0.5; 0.25];
    Aineq = [1, 0; -1, 0];
    bineq = [0.5; 1];
    Aeq = [0, 1];
    beq = 0.25;
    diagnostics = compute_qp_diagnostics( ...
        z, Aineq, bineq, Aeq, beq, 1e-10, 1e-6);

    verifyEqual(testCase, diagnostics.inequality_margin_min, 0, ...
        'AbsTol', 1e-14);
    verifyEqual(testCase, diagnostics.active_inequality_count, 1);
    verifyEqual(testCase, diagnostics.near_active_inequality_count, 1);
    verifyEqual(testCase, diagnostics.violated_inequality_count, 0);
    verifyEqual(testCase, diagnostics.equality_residual_norm, 0, ...
        'AbsTol', 1e-14);
end

function testStateDecompositionRemovesGlobalTranslation(testCase)
    p = get_params(0);
    [Xt, ~] = fcn_gen_XdUd(0, [], ones(4,1), p);
    shiftedXt = Xt;
    translation = [100; -20; 5];
    shiftedXt(1:3) = shiftedXt(1:3) + translation;
    feet = reshape(shiftedXt(19:30), 3, 4) + translation;
    shiftedXt(19:30) = feet(:);

    original = decompose_srb_state(Xt);
    shifted = decompose_srb_state(shiftedXt);
    verifyGreaterThan(testCase, shifted.full_state_norm, original.full_state_norm);
    verifyEqual(testCase, shifted.position_invariant_state_norm, ...
        original.position_invariant_state_norm, 'AbsTol', 1e-12);
end

function testPhaseOneSeparatesFeasibleAndInfeasible(testCase)
    feasibleQp = struct('H', eye(2), 'Aineq', [1, 0], 'bineq', 1, ...
        'Aeq', [0, 1], 'beq', 0, 'exitflag', -2);
    feasibleResult = analyze_qp_feasibility(feasibleQp);
    verifyEqual(testCase, feasibleResult.classification, ...
        'solver_difficulty_or_objective_numerics');
    verifyLessThanOrEqual(testCase, ...
        feasibleResult.minimum_scaled_inequality_relaxation_l1, 1e-8);

    infeasibleQp = struct('H', 1, 'Aineq', 1, 'bineq', 0, ...
        'Aeq', 1, 'beq', 1, 'exitflag', -2);
    infeasibleResult = analyze_qp_feasibility(infeasibleQp);
    verifyEqual(testCase, infeasibleResult.classification, ...
        'mathematically_infeasible_linear_constraints');
    verifyGreaterThan(testCase, ...
        infeasibleResult.minimum_scaled_inequality_relaxation_l1, 0.9);
    verifyEqual(testCase, infeasibleResult.offending_inequality_rows.row, 1);
end

function testNominalMpcQpSolves(testCase)
    p = get_params(0);
    [Xt, Ut] = fcn_gen_XdUd(0, [], ones(4,1), p);
    timeHorizon = p.Tmpc * (0:p.predHorizon-1);
    [FSM, Xd, Ud, Xt] = fcn_FSM(timeHorizon, Xt, p);
    [H, g, Aineq, bineq, Aeq, beq] = ...
        fcn_get_QP_form_eta(Xt, Ut, Xd, Ud, p);
    [z, ~, exitflag] = quadprog(H, g, Aineq, bineq, Aeq, beq, ...
        [], [], [], optimoptions('quadprog', 'Display', 'off'));

    verifyGreaterThan(testCase, exitflag, 0);
    verifySize(testCase, z, [144, 1]);
    verifySize(testCase, FSM, [4, 1]);
    diagnostics = compute_qp_diagnostics(z, Aineq, bineq, Aeq, beq);
    verifyGreaterThanOrEqual(testCase, diagnostics.inequality_margin_min, -1e-6);
    verifyLessThanOrEqual(testCase, diagnostics.equality_residual_max_abs, 1e-6);
end

function testFailureSnapshotRoundTrip(testCase)
    outputFolder = tempname;
    mkdir(outputFolder);
    cleanupObj = onCleanup(@() localRemoveFolder(outputFolder)); %#ok<NASGU>
    cfg.DIAGNOSTICS.enable = true;
    cfg.DIAGNOSTICS.save_full_qp_on_failure = true;
    cfg.RUN.enabled = true;
    cfg.RUN.qp_failure_dir = outputFolder;
    context = struct('episode_idx', 2, 'decision_idx', 3, ...
        'chunk_in_decision', 4);
    failureQP = struct('fail_iter', 5, 'H', eye(2));

    [failureId, failureFile] = ...
        save_qp_failure_snapshot(cfg, context, failureQP);
    verifyEqual(testCase, failureId, "ep0002_dec03_chunk04_iter0005");
    verifyTrue(testCase, isfile(failureFile));
    saved = load(failureFile, 'failureQP');
    verifyEqual(testCase, saved.failureQP.schema_version, ...
        'phase1_qp_failure_v1');
    verifyEqual(testCase, saved.failureQP.context, context);
end

function testManifestContainsRequiredReproducibilityFields(testCase)
    rootDir = bootstrap_RF_MPC_RL();
    active = load(fullfile(rootDir, 'rlEnv_MPC_R.mat'), 'cfg');
    manifestFile = [tempname '.txt'];
    cleanupObj = onCleanup(@() localDeleteFile(manifestFile)); %#ok<NASGU>
    runInfo = struct('run_id', 'unit_test', 'training_mode', 'test', ...
        'max_episodes', 50, 'random_seed', active.cfg.RNG_SEED, ...
        'noise_variance', active.cfg.TRAIN.noise_variance, ...
        'noise_decay', active.cfg.TRAIN.noise_decay);
    write_run_manifest(manifestFile, active.cfg, runInfo);
    manifest = fileread(manifestFile);
    requiredKeys = {'run_id:', 'branch:', 'git_sha:', 'git_dirty_status:', ...
        'matlab_version:', 'actor_learning_rate:', 'noise_standard_deviation:', ...
        'mission_target_m:', 'action_lower_bound:', 'battery_metric_type:', ...
        'reward_version:'};
    for i = 1:numel(requiredKeys)
        verifyTrue(testCase, contains(manifest, requiredKeys{i}), ...
            sprintf('Manifest is missing key %s', requiredKeys{i}));
    end
end

function localRemoveFolder(folder)
    if isfolder(folder)
        rmdir(folder, 's');
    end
end

function localDeleteFile(file)
    if isfile(file)
        delete(file);
    end
end
