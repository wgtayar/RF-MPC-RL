function updateBestRLog(R_new, SOC_current, te, ue, reward)

    [bestRewards, bestR, bestSOC, bestTE, bestUE] = loadBestRLog();

    if isempty(bestRewards)
        % first entry
        bestRewards = reward;
        bestR       = R_new(:).';   % 1x3 row
        bestSOC     = SOC_current;
        bestTE      = te;
        bestUE      = ue;
    else
        N = numel(bestRewards);

        if N < 10
            % still filling the top-10 buffer
            bestRewards = [bestRewards; reward];
            bestR       = [bestR; R_new(:).'];
            bestSOC     = [bestSOC; SOC_current];
            bestTE      = [bestTE; te];
            bestUE      = [bestUE; ue];
        else
            % replace worst entry if current reward is better
            [minVal, idxMin] = min(bestRewards);
            if reward > minVal
                bestRewards(idxMin) = reward;
                bestR(idxMin,:)     = R_new(:).';
                bestSOC(idxMin)     = SOC_current;
                bestTE(idxMin)      = te;
                bestUE(idxMin)      = ue;
            end
        end

        % sort by reward descending
        [bestRewards, idxSort] = sort(bestRewards,'descend');
        bestR   = bestR(idxSort,:);
        bestSOC = bestSOC(idxSort);
        bestTE  = bestTE(idxSort);
        bestUE  = bestUE(idxSort);
    end

    % rename for saving
    bestTrackingError = bestTE;
    bestControlEffort = bestUE;

    save('best_R_log.mat', ...
         'bestRewards','bestR','bestSOC', ...
         'bestTrackingError','bestControlEffort');
end
