classdef testEnergyMissionResume < matlab.unittest.TestCase
    % A run continued with resume_from gets the previous agent, its full replay
    % memory and its random stream; the rebuilt replay memory equals the saved one.
    properties
        Folder
        First
        Request
    end
    methods(TestClassSetup)
        function trainFirstTranche(tc)
            source=bootstrap_RF_MPC_RL();
            [status,text]=git_output(source,'status --porcelain');
            tc.assumeTrue(status==0 && isempty(strtrim(text)), ...
                'Training needs a clean committed source tree.');
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fixture.Folder;
            tc.Request=struct('mode','train','seed',20260903,'max_episodes',2, ...
                'max_steps_per_episode',3,'setup_options',struct('time_weight',0.63));
            tc.First=fullfile(tc.Folder,'t1');
            evalc('train_energy_mission_rl(tc.First,tc.Request);');
        end
    end
    methods(Test)
        function rebuiltReplayEqualsSavedReplay(tc)
            saved=load(fullfile(tc.First,'training','replay_buffer.mat'),'experiences').experiences;
            tc.verifyEqual(numel(saved),6);
            tc.verifyEqual(double([saved.IsDone]),[0 0 2 0 0 2]);
            tc.verifyEqual(rebuild_energy_mission_replay(tc.First,3),saved);
        end
        function continuedRunKeepsAgentReplayAndStream(tc)
            first=load(fullfile(tc.First,'training','final_agent.mat'),'agent','finalRng');
            firstReplay=load(fullfile(tc.First,'training','replay_buffer.mat'),'experiences').experiences;
            request=tc.Request; request.resume_from=tc.First;
            second=fullfile(tc.Folder,'t2');
            evalc('report=train_energy_mission_rl(second,request);');
            started=load(fullfile(second,'training','initial_agent.mat'),'agent','initialRng');
            tc.verifyEqual(getLearnableParameters(getActor(started.agent)), ...
                getLearnableParameters(getActor(first.agent)));
            tc.verifyEqual(started.initialRng,first.finalRng);
            replay=load(fullfile(second,'training','replay_buffer.mat'),'experiences').experiences;
            tc.verifyEqual(numel(replay),12);
            tc.verifyEqual(replay(1:6),firstReplay);
            tc.verifyEqual(report.episodes_trained,2);
            tc.verifyEqual(report.cumulative_episodes_trained,4);
            tc.verifyEqual(report.cumulative_training_decisions,12);
            tc.verifyEqual(report.resumed_from.replay_origin,'saved_replay_buffer');
            tc.verifyTrue(report.validation.valid);
            % 12 transitions stay below the 64-transition warm-up, so no update here.
        end
        function runWithoutSavedReplayIsRebuilt(tc)
            % Tranche-1 runs (before replay_buffer.mat existed) are rebuilt.
            copy=fullfile(tc.Folder,'t1-old'); copyfile(tc.First,copy);
            delete(fullfile(copy,'training','replay_buffer.mat'));
            request=tc.Request; request.resume_from=copy; request.max_episodes=1;
            evalc('report=train_energy_mission_rl(fullfile(tc.Folder,''t2-old''),request);');
            tc.verifyEqual(report.resumed_from.replay_origin,'rebuilt_from_decisions');
            tc.verifyEqual(report.resumed_from.replay_transitions,6);
            tc.verifyEqual(report.cumulative_episodes_trained,3);
        end
        function loadedReplayCountsTowardsWarmUp(tc)
            % With a 4-transition warm-up, 6 loaded transitions let learning
            % start at once; 2 new steps alone would stay below it.
            request=tc.Request; request.agent_options=struct('mini_batch_size',4,'num_warm_start_steps',4);
            request.max_episodes=2;
            first=fullfile(tc.Folder,'w1');
            evalc('train_energy_mission_rl(first,request);');
            request.resume_from=first; request.max_episodes=1; request.max_steps_per_episode=3;
            evalc('report=train_energy_mission_rl(fullfile(tc.Folder,''w2''),request);');
            tc.verifyTrue(report.actor_parameters_changed);
            tc.verifyTrue(all(report.critic_parameters_changed));
        end
        function differentConfigurationIsRejected(tc)
            request=tc.Request; request.resume_from=tc.First;
            request.setup_options.time_weight=0.3;
            tc.verifyError(@() train_energy_mission_rl(fullfile(tc.Folder,'t2-bad'),request), ...
                'train_energy_mission_rl:ResumeOptions');
            tc.verifyFalse(isfolder(fullfile(tc.Folder,'t2-bad')));
        end
    end
end
