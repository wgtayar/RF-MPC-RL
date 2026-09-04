function samples = generate_phase2_action_samples(cfg)
%generate_phase2_action_samples Build interpretable slices of the 5-D action space.

    gammaV = linspace(cfg.GAMMA_V_MIN, cfg.GAMMA_V_MAX, 5);
    gammaA = linspace(cfg.GAMMA_A_MIN, cfg.GAMMA_A_MAX, 5);
    profiles = [ ...
        0, 0, 0
        -cfg.DR_MAX, -cfg.DR_MAX, cfg.DR_MAX
        cfg.DR_MAX, 0, -cfg.DR_MAX];
    profileNames = ["nominal_dR", "Agent74_75_initial_dR", ...
        "opposing_boundary_dR"];
    rows = cell(75 + 15 + 40, 8);
    actionId = 0;
    for profile = 1:size(profiles, 1)
        for i = 1:numel(gammaV)
            for j = 1:numel(gammaA)
                actionId = actionId + 1;
                rows(actionId, :) = {actionId, "gamma_heatmap", ...
                    profileNames(profile), profiles(profile, 1), ...
                    profiles(profile, 2), profiles(profile, 3), ...
                    gammaV(i), gammaA(j)};
            end
        end
    end

    dRValues = linspace(-cfg.DR_MAX, cfg.DR_MAX, 5);
    gammaVFixed = cfg.GAMMA_V_MISSION;
    gammaAFixed = mean([cfg.GAMMA_A_MIN, cfg.GAMMA_A_MAX]);
    for axis = 1:3
        for value = dRValues
            dR = zeros(1, 3);
            dR(axis) = value;
            actionId = actionId + 1;
            rows(actionId, :) = {actionId, "dR_slice", "dR" + axis, ...
                dR(1), dR(2), dR(3), gammaVFixed, gammaAFixed};
        end
    end

    rngState = rng;
    cleanupObj = onCleanup(@() rng(rngState));
    rng(cfg.RNG_SEED, cfg.RNG_ALGORITHM);
    centers = [ ...
        -cfg.DR_MAX, -cfg.DR_MAX, cfg.DR_MAX, ...
            cfg.GAMMA_V_MAX, cfg.GAMMA_A_MIN
        0, 0, 0, cfg.GAMMA_V_MISSION, gammaAFixed];
    lower = [-cfg.DR_MAX, -cfg.DR_MAX, -cfg.DR_MAX, ...
        cfg.GAMMA_V_MIN, cfg.GAMMA_A_MIN];
    upper = [cfg.DR_MAX, cfg.DR_MAX, cfg.DR_MAX, ...
        cfg.GAMMA_V_MAX, cfg.GAMMA_A_MAX];
    scale = 0.12*(upper - lower);
    for center = 1:size(centers, 1)
        for sample = 1:20
            action = centers(center, :) + scale.*randn(1, 5);
            action = min(max(action, lower), upper);
            actionId = actionId + 1;
            rows(actionId, :) = {actionId, "local_perturbation", ...
                "center" + center, action(1), action(2), action(3), ...
                action(4), action(5)};
        end
    end
    samples = cell2table(rows, 'VariableNames', ...
        {'action_id','sample_type','profile','dR1','dR2','dR3', ...
        'gamma_v','gamma_a'});
end
