function experiences = rebuild_energy_mission_replay(root, maxStepsPerEpisode)
%REBUILD_ENERGY_MISSION_REPLAY TD3 replay memory from a captured training run.
% Saved agents carry an empty replay memory, so runs trained before
% training/replay_buffer.mat existed are rebuilt from their records: the
% observation, next observation, reward and done flag of decisions/ and the
% agent-side action of normalized_actions/. As in the toolbox, a decision
% that reaches maxStepsPerEpisode without a native terminal is stored with
% IsDone = 2 (truncation, the critic still bootstraps).
% Matches allExperiences(agent.ExperienceBuffer) exactly (tested).
    files = dir(fullfile(root,'decisions','episode_*_decision_*.mat'));
    names = sort({files.name});   % zero-padded: lexical order is chronological
    count = numel(names);
    assert(count > 0,'rebuild_energy_mission_replay:Empty','No decisions in %s.',root);
    keys = regexp(names,'^episode_(\d+)_decision_(\d+)\.mat$','tokens','once');
    index = cellfun(@(t) str2double(t),vertcat(keys{:}));
    experiences = repmat(struct('NextObservation',{{}},'Observation',{{}}, ...
        'Action',{{}},'Reward',0,'IsDone',uint8(0)),count,1);
    for k = 1:count
        saved = load(fullfile(root,'decisions',names{k}),'event');
        record = saved.event.record;
        audit = load(fullfile(root,'normalized_actions',names{k}),'audit').audit;
        assert(audit.episode == index(k,1) && audit.decision == index(k,2), ...
            'rebuild_energy_mission_replay:Order','Action audit does not match %s.',names{k});
        isDone = double(record.is_done);
        lastOfEpisode = k == count || index(k+1,1) ~= index(k,1);
        if isDone == 0 && lastOfEpisode && index(k,2) == maxStepsPerEpisode
            isDone = 2;
        end
        experiences(k) = struct('NextObservation',{{record.next_observation}}, ...
            'Observation',{{record.observation}},'Action',{{audit.requested_normalized}}, ...
            'Reward',record.reward,'IsDone',uint8(isDone));
    end
    % Every episode must be contiguous from decision 1.
    for k = 1:count
        if k == 1 || index(k,1) ~= index(k-1,1)
            assert(index(k,2) == 1,'rebuild_energy_mission_replay:Gap','Episode %d does not start at decision 1.',index(k,1));
        else
            assert(index(k,1) == index(k-1,1) && index(k,2) == index(k-1,2)+1, ...
                'rebuild_energy_mission_replay:Gap','Decision gap at %s.',names{k});
        end
    end
end
