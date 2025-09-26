% function train_RL_MPC()
%     % Load environment and agent
%     data = load('rlEnv_MPC_R.mat', 'env', 'agent');
%     env = data.env;
%     agent = data.agent;
% 
%     % Training options
%     maxEpisodes = 60; % Total episodes
%     trainOpts = rlTrainingOptions(...
%         'MaxEpisodes', maxEpisodes, ...
%         'MaxStepsPerEpisode', 1, ...
%         'ScoreAveragingWindowLength', 20, ...
%         'Verbose', true, ...
%         'Plots', 'training-progress', ...
%         'StopTrainingCriteria', 'AverageReward', ...
%         'StopTrainingValue', 200);
%         % 'SaveAgentCriteria', 'EpisodeReward', ...
%         % 'SaveAgentValue', Inf, ...
%         % 'SaveAgentDirectory', pwd);
% 
%     % bestReward = -Inf;
% 
%     % trainOpts.StopTrainingFcn = @(agentInfo) saveBestAgent(agentInfo, bestReward);
% 
%     % Train agent
%     trainingStats = train(agent, env, trainOpts);
% 
%     save('final_RL_agent.mat', 'agent', 'trainingStats');
%     disp('Training complete. Final agent saved.');
% end
% 
% % function stop = saveBestAgent(agentInfo, bestReward)
% %     stop = false;
% % 
% %     currentReward = agentInfo.EpisodeReward;
% % 
% %     if currentReward > bestReward
% %         save('best_RL_Agent.mat', 'agentInfo', '-v7.3');
% %         fprintf('New best agent saved with the reward: %.4f\n', currentReward);
% %         bestReward = currentReward;
% %     end
% % end

% function train_RL_MPC()
%     data = load('rlEnv_MPC_R.mat', 'env', 'agent');
%     env = data.env;
%     agent = data.agent;
% 
%     % Training options
%     maxEpisodes = 600; % Total episodes
%     trainOpts = rlTrainingOptions( ...
%         'MaxEpisodes', maxEpisodes, ...
%         'MaxStepsPerEpisode', 1, ...
%         'ScoreAveragingWindowLength', 20, ...
%         'Verbose', true, ...
%         'Plots', 'training-progress', ...
%         'StopTrainingCriteria', 'AverageReward', ...
%         'StopTrainingValue', 200);
% 
%     % trainOpts.StopTrainingFcn = @checkpointcharlie;  % checkpoint saver
% 
%     trainingStats = train(agent, env, trainOpts);
% 
%     save('final_RL_agent.mat', 'agent', 'trainingStats');
%     disp('Training complete. Final agent saved.');
% 
%     function stop = checkpointcharlie(info)
%         persistent best lastSaved havePending
%         if isempty(best)
%             best = -inf; 
%         end
%         if isempty(lastSaved)
%             lastSaved = -inf;
%         end
%         if isempty(havePending)
%             havePending = false;
%         end
%         stop = false;
% 
%         if info.EpisodeReward > best
%             best = info.EpisodeReward;
%             save('best_agent_pending.mat','agent','best','-v7.3');
%             havePending = true;
%         end
% 
%         if mod(info.EpisodeIndex,10)==0 && havePending && best > lastSaved
%             S = load('best_agent_pending.mat','agent','bestEver');
%             agent = S.agent;
%             save('best_RL_agent.mat','agent','best','-v7.3');
%             lastSaved = S.bestEver;
%             havePending = false;
%             fprintf('Checkpointed charlie @%d: saved best agent (reward %.4f)\n', info.EpisodeIndex, lastSaved);
%         end
%     end
% end


function train_RL_MPC()
    S = load('rlEnv_MPC_R.mat','env','agent','cfg');
    env   = S.env;
    agent = S.agent;

    trainOpts = rlTrainingOptions( ...
        'MaxEpisodes', 600, ...
        'MaxStepsPerEpisode', 10, ...
        'ScoreAveragingWindowLength', 20, ...
        'Verbose', true, ...
        'Plots', 'training-progress', ...
        'StopTrainingCriteria','EpisodeReward', ... % improvement-based, can cross zero
        'StopTrainingValue', 50); % tweak later

    stats = train(agent, env, trainOpts);

    save('final_RL_agent.mat','agent','stats');
    disp('Training complete.');
end
