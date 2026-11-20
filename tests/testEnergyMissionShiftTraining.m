classdef testEnergyMissionShiftTraining < matlab.unittest.TestCase
    % End to end through train_energy_mission_rl (monitor #40): the learner's
    % replay memory marks mission boundaries inside an episode as not done (0)
    % and the mission cap as a truncation (2), and a warm start copies the
    % source agent's networks with an empty replay memory.
    properties
        Folder
        First
        Request
    end
    methods(TestClassSetup)
        function trainShortShifts(tc)
            source=bootstrap_RF_MPC_RL();
            [status,text]=git_output(source,'status --porcelain');
            tc.assumeTrue(status==0 && isempty(strtrim(text)), ...
                'Training needs a clean committed source tree.');
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fixture.Folder;
            shift=struct('enabled',true,'max_missions',2,'min_start_soc',.30);
            tc.Request=struct('mode','train','seed',20260903,'max_episodes',2, ...
                'agent_options',struct('mini_batch_size',2,'num_warm_start_steps',2), ...
                'setup_options',struct('time_weight',0.63,'initial_soc_range',[.90 .95], ...
                    'target_distance_m',3,'mission_duration_s',10,'shift',shift));
            tc.First=fullfile(tc.Folder,'s1');
            evalc('train_energy_mission_rl(tc.First,tc.Request);');
        end
    end
    methods(Test)
        function replayMarksBoundariesAndTheCap(tc)
            r=jsondecode(fileread(fullfile(tc.First,'training','report.json')));
            tc.verifyTrue(r.shift.enabled);
            v=load(fullfile(tc.First,'training','validation.mat')).validation;
            tc.verifyTrue(v.valid,'Every captured mission validates.');
            e=load(fullfile(tc.First,'training','replay_buffer.mat')).experiences;
            d=double([e.IsDone]);
            log=readtable(fullfile(tc.First,'shift_missions.csv'));
            tc.verifyEqual(sum(d==2),sum(log.learner_done==2),'Each cap is stored as a truncation.');
            tc.verifyEqual(sum(d==1),sum(log.learner_done==1),'Only shift ends and failures are terminal.');
            tc.verifyGreaterThanOrEqual(sum(log.learner_done==0),1,'At least one mission boundary inside an episode.');
            % One captured native episode per mission; after a final cap the next
            % mission has already been started (to bridge the cut) but not walked.
            tc.verifyEqual(r.captured_episodes,height(log)+double(log.learner_done(end)==2), ...
                'One captured native episode per mission (plus a started one after a final cap).');
        end

        function warmStartCopiesNetworksWithEmptyReplay(tc)
            request=tc.Request; request.max_episodes=1; request.init_agent_from=tc.First;
            second=fullfile(tc.Folder,'s2');
            evalc('report=train_energy_mission_rl(second,request);');
            source=load(fullfile(tc.First,'training','final_agent.mat'),'agent').agent;
            start=load(fullfile(second,'training','initial_agent.mat'),'agent').agent;
            tc.verifyEqual(getLearnableParameters(getActor(start)),getLearnableParameters(getActor(source)));
            c0=getCritic(source); c1=getCritic(start);
            tc.verifyEqual(getLearnableParameters(c1(1)),getLearnableParameters(c0(1)));
            tc.verifyEqual(getLearnableParameters(c1(2)),getLearnableParameters(c0(2)));
            tc.verifyEqual(start.ExperienceBuffer.Length,0,'The source replay memory is not carried.');
            tc.verifyEqual(report.warm_started_from.run_id,'s1');
            tc.verifyEqual(report.warm_started_from.replay,'not_carried');
        end

        function warmStartRejectsADifferentAgent(tc)
            request=tc.Request; request.max_episodes=1; request.init_agent_from=tc.First;
            request.setup_options.battery_blind=true;
            tc.verifyError(@()train_energy_mission_rl(fullfile(tc.Folder,'s3'),request), ...
                'train_energy_mission_rl:WarmStartAgent');
        end
    end
end
