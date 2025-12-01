function [bestRewards, bestR, bestSOC, bestTE, bestUE] = loadBestRLog()

    if exist('best_R_log.mat','file')
        try
            B = load('best_R_log.mat', ...
                     'bestRewards','bestR', ...
                     'bestSOC','bestTrackingError', ...
                     'bestControlEffort');

            if isfield(B,'bestRewards');
                bestRewards = B.bestRewards; 
            else
                bestRewards = []; 
            end

            if isfield(B,'bestR')
                bestR = B.bestR; 
            else
                bestR = []; 
            end
            
            if isfield(B,'bestSOC')
                bestSOC = B.bestSOC; 
            else 
                bestSOC = [];
            end
            
            if isfield(B,'bestTrackingError')
                bestTE = B.bestTrackingError;
            else
                bestTE = [];
            end
            
            if isfield(B,'bestControlEffort')
                bestUE = B.bestControlEffort;
            else
                bestUE = []; 
            end
        catch
            bestRewards = [];
            bestR = [];
            bestSOC = [];
            bestTE = [];
            bestUE = [];
        end
    else
        bestRewards = [];
        bestR = [];
        bestSOC = [];
        bestTE = [];
        bestUE = [];
    end
end
