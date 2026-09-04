function candidates = select_viable_action_candidates(mapFile, cfg, outputFolder)
%select_viable_action_candidates Select mission-sufficient actions inside sampled safe sets.

    data = load(mapFile, 'map');
    map = data.map;
    if nargin < 3 || strlength(string(outputFolder)) == 0
        outputFolder = fileparts(mapFile);
    end
    missionSpeed = cfg.MISSION.D_TARGET_M/cfg.MISSION_DURATION;
    baseIds = unique(map.base_id, 'stable');
    rows = repmat(localEmptyRow(), numel(baseIds), 1);
    actionScale = [2*cfg.DR_MAX, 2*cfg.DR_MAX, 2*cfg.DR_MAX, ...
        cfg.GAMMA_V_MAX-cfg.GAMMA_V_MIN, ...
        cfg.GAMMA_A_MAX-cfg.GAMMA_A_MIN];

    for i = 1:numel(baseIds)
        baseRows = map(map.base_id == baseIds(i), :);
        eligible = baseRows(baseRows.safe_action & ...
            baseRows.v_cmd >= missionSpeed, :);
        row = localEmptyRow();
        row.base_id = baseIds(i);
        row.base_label = baseRows.base_label(1);
        row.mission_speed_required = missionSpeed;
        row.safe_sample_fraction = mean(baseRows.safe_action);
        row.mission_sufficient_safe_count = height(eligible);
        if isempty(eligible)
            row.status = "no_sampled_mission_sufficient_safe_action";
            rows(i) = row;
            continue
        end
        eligible = sortrows(eligible, ...
            {'risk_score','solver_iterations_max','v_cmd'}, ...
            {'ascend','ascend','ascend'});
        best = eligible(1, :);
        baselineMask = baseRows.profile == "Agent74_75_initial_dR" & ...
            abs(baseRows.gamma_v-cfg.GAMMA_V_MAX) < 1e-12 & ...
            abs(baseRows.gamma_a-cfg.GAMMA_A_MIN) < 1e-12;
        baseline = baseRows(find(baselineMask, 1), :);
        row.status = "candidate_for_long_horizon_validation";
        row.dR1 = best.dR1;
        row.dR2 = best.dR2;
        row.dR3 = best.dR3;
        row.gamma_v = best.gamma_v;
        row.gamma_a = best.gamma_a;
        row.v_cmd = best.v_cmd;
        row.a_cmd = best.a_cmd;
        row.risk_score = best.risk_score;
        row.solver_iterations_max = best.solver_iterations_max;
        row.sampled_safe_margin = localSafeMargin(best, baseRows, actionScale);
        if ~isempty(baseline)
            row.Agent74_initial_action_safe = baseline.safe_action;
            row.Agent74_initial_action_risk = baseline.risk_score;
            row.risk_delta_vs_Agent74_initial = ...
                best.risk_score-baseline.risk_score;
            row.action_distance_from_Agent74_initial = norm( ...
                ([best.dR1,best.dR2,best.dR3,best.gamma_v,best.gamma_a] - ...
                [baseline.dR1,baseline.dR2,baseline.dR3, ...
                baseline.gamma_v,baseline.gamma_a])./actionScale);
        end
        rows(i) = row;
    end
    candidates = struct2table(rows);
    writetable(candidates, fullfile(outputFolder, ...
        'viable_action_candidates.csv'));
    save(fullfile(outputFolder, 'viable_action_candidates.mat'), ...
        'candidates', 'missionSpeed');
end

function margin = localSafeMargin(candidate, rows, scale)
    unsafe = rows(~rows.safe_action, :);
    if isempty(unsafe)
        margin = inf;
        return
    end
    candidateAction = [candidate.dR1,candidate.dR2,candidate.dR3, ...
        candidate.gamma_v,candidate.gamma_a];
    unsafeActions = [unsafe.dR1,unsafe.dR2,unsafe.dR3, ...
        unsafe.gamma_v,unsafe.gamma_a];
    distances = vecnorm((unsafeActions-candidateAction)./scale, 2, 2);
    margin = min(distances);
end

function row = localEmptyRow()
    row = struct('base_id', NaN, 'base_label', "", 'status', "", ...
        'mission_speed_required', NaN, 'safe_sample_fraction', NaN, ...
        'mission_sufficient_safe_count', NaN, ...
        'dR1', NaN, 'dR2', NaN, 'dR3', NaN, ...
        'gamma_v', NaN, 'gamma_a', NaN, 'v_cmd', NaN, 'a_cmd', NaN, ...
        'risk_score', NaN, 'solver_iterations_max', NaN, ...
        'sampled_safe_margin', NaN, 'Agent74_initial_action_safe', false, ...
        'Agent74_initial_action_risk', NaN, ...
        'risk_delta_vs_Agent74_initial', NaN, ...
        'action_distance_from_Agent74_initial', NaN);
end
