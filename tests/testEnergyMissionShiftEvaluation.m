classdef testEnergyMissionShiftEvaluation < matlab.unittest.TestCase
    % Shift evaluation (monitor #42, protocol A3): missions chained on one
    % battery with no cap, the robot restarted each mission, a normal end below
    % the minimum start SOC, the shift score G at the scoring weight, and
    % resume after an interruption. Short missions test the plumbing only.
    properties
        Folder
        Source
    end
    methods(TestClassSetup)
        function prepare(tc)
            tc.Source=bootstrap_RF_MPC_RL();
            [status,text]=git_output(tc.Source,'status --porcelain');
            tc.assumeTrue(status==0 && isempty(strtrim(text)),'Evaluation needs a clean committed source tree.');
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fixture.Folder;
        end
    end
    methods(Test)
        function fixedSpeedShiftChainsScoresAndResumes(tc)
            o=struct('policy','fixed','speed_m_s',1.0,'start_soc',.95,'min_start_soc',.9488, ...
                'smoke_distance_m',3,'smoke_duration_s',10);
            root=fullfile(tc.Folder,'fixed');
            evalc('r=run_energy_mission_shift_evaluation(root,o);');
            m=r.missions;
            tc.verifyEqual(numel(m),2,'Two short missions reach the minimum start SOC.');
            tc.verifyTrue(all([m.mission_complete]));
            tc.verifyEqual(m(2).start_soc_true,m(1).end_soc_true,'Battery carried exactly.');
            tc.verifyEqual(m(2).start_soc_estimated,m(1).end_soc_estimated);
            tc.verifyLessThan(m(2).end_soc_estimated,o.min_start_soc,'Normal end below the minimum start SOC.');
            tc.verifyEqual(r.G,sum(2-.8*[m.tau]-.2*[m.kappa]),'AbsTol',1e-12,'G at the scoring weight.');
            tc.verifyTrue(r.all_datasets_validated);
            tc.verifyEqual(r.failed_qps,0);
            % Interrupt during mission 2: restore the state saved after mission 1
            % (taken from an identical one-mission shift) and run again.
            s=load(fullfile(root,'state.mat'));options=s.options;source_sha=s.source_sha; %#ok<NASGU>
            evalc('r1=run_energy_mission_shift_evaluation(fullfile(tc.Folder,''fixed1''),setfield(o,''min_start_soc'',.9495));');
            tc.verifyEqual(numel(r1.missions),1);
            b=load(fullfile(tc.Folder,'fixed1','state.mat'));
            missions=b.missions;carried=b.carried;done=false; %#ok<NASGU>
            save(fullfile(root,'state.mat'),'missions','carried','done','options','source_sha','-mat');
            evalc('r2=run_energy_mission_shift_evaluation(root,o);');
            tc.verifyTrue(isfolder(fullfile(root,'mission_02_interrupted_1')),'Interrupted mission preserved.');
            tc.verifyEqual(numel(r2.missions),2);
            tc.verifyEqual(r2.missions(2).charge_As,m(2).charge_As,'AbsTol',1e-9,'Resumed mission repeats exactly.');
            tc.verifyError(@()run_energy_mission_shift_evaluation(root,setfield(o,'speed_m_s',.9)), ...
                'shiftEval:Resume');
        end

        function agentShiftUsesTheDeterministicActor(tc)
            shift=struct('enabled',true,'max_missions',2,'min_start_soc',.30);
            request=struct('mode','train','seed',20260903,'max_episodes',1, ...
                'agent_options',struct('mini_batch_size',2,'num_warm_start_steps',2), ...
                'setup_options',struct('time_weight',0.80,'initial_soc_range',[.90 .95], ...
                    'target_distance_m',3,'mission_duration_s',10,'shift',shift,'battery_blind',true));
            run=fullfile(tc.Folder,'agent_run');
            evalc('train_energy_mission_rl(run,request);');
            o=struct('policy','agent','checkpoint',run,'start_soc',.95,'min_start_soc',.9495, ...
                'smoke_distance_m',3,'smoke_duration_s',10);
            evalc('r=run_energy_mission_shift_evaluation(fullfile(tc.Folder,''agent_eval''),o);');
            tc.verifyTrue(r.policy.battery_blind,'The checkpoint configuration (blind) is kept.');
            tc.verifyEqual(r.policy.episodes_trained,1);
            tc.verifyTrue(all([r.missions.actions_reconstructed]),'Logged actions match the actor.');
            tc.verifyTrue(r.all_datasets_validated);
            tc.verifyEqual(r.missions_attempted,1);
        end
    end
end
