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
        function trainingSummaryMatchesRun(tc)
            % The exported return, rebuilt from its four reward terms, equals the
            % toolbox episode reward; the run folder is left unchanged.
            before=dir(fullfile(tc.First,'**','*'));
            out=fullfile(tc.Folder,'summary-t1');
            summary=export_energy_mission_training_summary(tc.First,out);
            tc.verifyEqual(summary.decisions,6);
            tc.verifyEqual(summary.probe_checkpoints,2);
            E=readtable(fullfile(out,'episodes.csv'),'TextType','string');
            D=readtable(fullfile(out,'decisions.csv'),'TextType','string');
            tc.verifyEqual(E.episode_return,E.toolbox_episode_reward,'AbsTol',1e-12);
            tc.verifyEqual(D.r_progress+D.r_time+D.r_charge+D.r_terminal,D.reward,'AbsTol',1e-12);
            tc.verifyEqual(E.end_reason,["step_limit";"step_limit"]);
            replay=load(fullfile(tc.First,'training','replay_buffer.mat'),'experiences').experiences;
            x=load(fullfile(out,'transitions.mat'));
            tc.verifyEqual(x.rewards,[replay.Reward]);
            stored=[replay.Action]; tc.verifyEqual(x.actions,double([stored{:}]),'AbsTol',1e-12);
            after=dir(fullfile(tc.First,'**','*'));
            tc.verifyEqual({after.name},{before.name});
            tc.verifyEqual([after.bytes],[before.bytes]);
            tc.verifyError(@() export_energy_mission_training_summary(tc.First,out), ...
                'export_energy_mission_training_summary:OutputExists');
            files=plot_energy_mission_training_summary({out},fullfile(tc.Folder,'figures'));
            tc.verifyEqual(numel(files),3);
            tc.verifyTrue(all(cellfun(@isfile,files)));
        end
        function splitSummaryKeepsWholeEpisodesBelowLimit(tc)
            % Split decision files hold whole episodes, stay below the byte
            % limit, and together equal the unsplit table to 6 significant digits.
            whole=fullfile(tc.Folder,'summary-whole');
            export_energy_mission_training_summary(tc.First,whole,'probe_checkpoints',false);
            D=readtable(fullfile(whole,'decisions.csv'),'TextType','string');
            one=fullfile(tc.Folder,'summary-one');
            export_energy_mission_training_summary(tc.First,one,'probe_checkpoints',false, ...
                'split_decisions',true);
            tc.verifyFalse(isfile(fullfile(one,'decisions.csv')));
            single=dir(fullfile(one,'decisions-*.csv'));
            tc.verifyEqual({single.name},{'decisions-001.csv'});
            % A limit that fits one episode but not two gives one file per episode.
            fid=fopen(fullfile(one,'decisions-001.csv'),'r'); bytes=fread(fid,Inf,'*uint8')'; fclose(fid);
            ends=find(bytes==uint8(10));
            tc.assertNumElements(ends,height(D)+1);
            header=ends(1); rowBytes=diff(ends(:));
            perEpisode=arrayfun(@(e) sum(rowBytes(D.episode==e)),unique(D.episode));
            limit=header+max(perEpisode)+1;
            tc.assumeLessThan(limit,header+sum(perEpisode));
            out=fullfile(tc.Folder,'summary-split');
            evalc(['summary=export_energy_mission_training_summary(tc.First,out,' ...
                '''probe_checkpoints'',false,''split_decisions'',true,''max_file_bytes'',limit);']);
            parts=dir(fullfile(out,'decisions-*.csv'));
            tc.verifyEqual({parts.name},{'decisions-001.csv','decisions-002.csv'});
            tc.verifyLessThan([parts.bytes],limit);
            tc.verifyEqual(summary.decision_files,{'decisions-001.csv','decisions-002.csv'});
            tc.verifyEqual(summary.decisions,height(D));
            P=cell(numel(parts),1);
            for k=1:numel(parts)
                P{k}=readtable(fullfile(out,parts(k).name),'TextType','string');
                tc.verifyNumElements(unique(P{k}.episode),1);
            end
            S=vertcat(P{:});
            tc.verifyEqual(S.episode,D.episode);
            tc.verifyEqual(S.decision,D.decision);
            tc.verifyEqual(S.reward,D.reward,'RelTol',1e-5);
            tc.verifyEqual(S.soc_end_pct,D.soc_end_pct,'RelTol',1e-5);
            tc.verifyEqual(S.qp_solves,D.qp_solves);
            % A limit below one episode is refused.
            tc.verifyError(@() export_energy_mission_training_summary(tc.First, ...
                fullfile(tc.Folder,'summary-tiny'),'probe_checkpoints',false, ...
                'split_decisions',true,'max_file_bytes',header+10), ...
                'export_energy_mission_training_summary:EpisodeTooLarge');
            % The plots read the split files.
            files=plot_energy_mission_training_summary({out},fullfile(tc.Folder,'figures-split'));
            tc.verifyEqual(numel(files),3);
            for k=1:numel(files)
                info=dir(files{k});
                tc.verifyLessThan(info.bytes,1e6,files{k});
            end
        end
    end
end
