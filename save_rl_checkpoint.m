function save_rl_checkpoint(checkpointFile, checkpoint)
    if exist(checkpointFile, 'file')
        S = load(checkpointFile, 'checkpointData');
        checkpointData = S.checkpointData;
    else
        checkpointData = struct();
        checkpointData.run_stamp = '';
        checkpointData.run_dir = '';
        checkpointData.created_at = '';
        checkpointData.every_decisions = [];
        checkpointData.history = struct([]);
        checkpointData.latest = struct([]);
        checkpointData.bestReward = struct([]);
        checkpointData.bestCompleted = struct([]);
    end

    if ~isfield(checkpointData, 'history') || isempty(checkpointData.history)
        checkpointData.history = checkpoint;
    else
        checkpointData.history(end+1,1) = checkpoint;
    end

    checkpointData.latest = checkpoint;

    if isempty(checkpointData.bestReward)
        checkpointData.bestReward = checkpoint;
    elseif checkpoint.reward > checkpointData.bestReward.reward
        checkpointData.bestReward = checkpoint;
    end

    if checkpoint.completed_episode
        if isempty(checkpointData.bestCompleted)
            checkpointData.bestCompleted = checkpoint;
        elseif checkpoint.reward > checkpointData.bestCompleted.reward
            checkpointData.bestCompleted = checkpoint;
        end
    end

    save(checkpointFile, 'checkpointData', '-v7.3');
end