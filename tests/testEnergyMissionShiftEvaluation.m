classdef testEnergyMissionShiftEvaluation < matlab.unittest.TestCase
    % Shift evaluation (monitor #42, protocol A3): missions chained on one
    % battery with no cap, the robot restarted each mission, a normal end below
    % the minimum start SOC, the shift score G at the scoring weight, and
    % resume after an interruption, and the B3-shift charge-budget policy
    % choosing its speed at each mission start, and the B4 headroom-feedback
    % governor acting at every decision. Short missions test the plumbing only.
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
            tc.verifyEqual([r2.missions.chosen_speed_m_s],[1 1],'Fixed policy records its speed.');
            csv=readtable(fullfile(root,'missions.csv'));
            tc.verifyEqual(csv.chosen_speed_m_s.',[1 1]);
            tc.verifyFalse(isfile(fullfile(root,'mission_01_policy_choice.json')));
        end

        function b3ShiftChoosesSpeedAtEachMissionStart(tc)
            % Synthetic sweep: starts 0.30, 0.9495 and 1.0 for every candidate
            % speed; 1.0 m/s failed at 0.30. Mission 1 starts at estimated SOC
            % 0.95 (nearest start at or below: 0.9495, 1.0 m/s completed) and
            % walks at 1.0 m/s; mission 2 starts below 0.9495 (nearest start
            % 0.30, 1.0 m/s failed) and walks at 0.9 m/s.
            speeds=[.75 .8 .85 .9 1];starts=[.30 .9495 1];
            [S,V]=ndgrid(starts,speeds);Q=repmat(7200*[.09 .0925 .095 .0975 .10],numel(starts),1);
            outcome=repmat("mission_complete",size(S));fail=V==1 & S==.30;outcome(fail)="actuator_limit";
            t=table(S(:),V(:),outcome(:),Q(:),repmat("benchmark",numel(S),1),'VariableNames', ...
                {'initial_soc','requested_speed_m_s','outcome','charge_As','role'});
            tablePath=fullfile(tc.Folder,'b3_sweep.csv');writetable(t,tablePath);
            o=struct('policy','b3_shift','table',{{tablePath}},'start_soc',.95,'min_start_soc',.9488, ...
                'smoke_distance_m',3,'smoke_duration_s',10);
            tc.verifyError(@()run_energy_mission_shift_evaluation(fullfile(tc.Folder,'bad'), ...
                setfield(o,'speed_m_s',1)),'shiftEval:Speed');
            tc.verifyError(@()run_energy_mission_shift_evaluation(fullfile(tc.Folder,'bad'), ...
                rmfield(o,'table')),'shiftEval:Table');
            root=fullfile(tc.Folder,'b3');
            evalc('r=run_energy_mission_shift_evaluation(root,o);');
            m=r.missions;
            tc.verifyGreaterThanOrEqual(numel(m),2);
            tc.verifyTrue(all([m.mission_complete]));
            tc.verifyLessThan(m(end).end_soc_estimated,o.min_start_soc,'Normal end below the minimum start SOC.');
            tc.verifyEqual([m.chosen_speed_m_s],[1 .9*ones(1,numel(m)-1)]);
            tc.verifyEqual(m(1).start_soc_estimated,.95,'AbsTol',1e-12);
            tc.verifyLessThan(m(2).start_soc_estimated,.9495);
            tc.verifyEqual(r.policy.table_sha256,{sha256_file(tablePath)});
            choice=jsondecode(fileread(fullfile(root,'mission_02_policy_choice.json')));
            tc.verifyEqual(choice.speed_m_s,.9);
            tc.verifyFalse(choice.completes(1),'1.0 m/s excluded by the failed 0.30 cell.');
            [v,a]=energy_mission_b3_shift_speed(tablePath,m(2).start_soc_estimated);
            tc.verifyEqual(v,.9);tc.verifyEqual(choice.missions_frugal,a.missions_frugal);
            csv=readtable(fullfile(root,'missions.csv'));
            tc.verifyEqual(csv.chosen_speed_m_s.',[m.chosen_speed_m_s]);
            tc.verifyEqual(r.failed_qps,0);
            tc.verifyTrue(r.all_datasets_validated);
        end

        function b4GovernorReactsToHeadroomEveryDecision(tc)
            % h_low above any reachable headroom: the command starts at v* and
            % falls 0.1 m/s at every later decision. Thresholds below any
            % headroom with a cap of 0.8 m/s: the command holds at the cap and
            % never rises toward v*. One short mission per shift.
            o=struct('policy','b4','b4_h_low_V',100,'b4_h_high_V',200,'b4_v_star_m_s',1.0, ...
                'start_soc',.95,'min_start_soc',.9495,'smoke_distance_m',3,'smoke_duration_s',10);
            tc.verifyError(@()run_energy_mission_shift_evaluation(fullfile(tc.Folder,'bad'), ...
                setfield(o,'speed_m_s',1)),'shiftEval:Speed');
            tc.verifyError(@()run_energy_mission_shift_evaluation(fullfile(tc.Folder,'bad'), ...
                setfield(o,'b4_h_low_V',300)),'b4Governor:Thresholds');
            root=fullfile(tc.Folder,'b4_lower');
            evalc('r=run_energy_mission_shift_evaluation(root,o);');
            m=r.missions;
            tc.verifyEqual(numel(m),1);
            choice=jsondecode(fileread(fullfile(root,'mission_01_policy_choice.json')));
            d=choice.decisions;n=numel(d);
            tc.verifyGreaterThanOrEqual(n,2,'At least one feedback decision.');
            tc.verifyEqual([d.command_m_s].',1.0-.1*(0:n-1).','AbsTol',1e-9,'Steps down 0.1 m/s per decision.');
            tc.verifyEqual({d.event}.',[{'start'};repmat({'lower'},n-1,1)]);
            h=[d(2:end).headroom_per_cell_V];
            tc.verifyTrue(all(isfinite(h) & h<o.b4_h_low_V),'Headroom measured after the first decision.');
            tc.verifyEqual(m.b4_decisions,n);
            tc.verifyEqual(m.b4_speed_reductions,n-1);tc.verifyEqual(m.b4_speed_increases,0);
            tc.verifyEqual(m.chosen_speed_m_s,mean([d.command_m_s]),'AbsTol',1e-12,'Mean B4 command.');
            tc.verifyEqual([m.b4_h_low_V m.b4_h_high_V m.b4_v_star_m_s],[100 200 1]);
            manifest=jsondecode(fileread(fullfile(root,'manifest.json')));
            tc.verifyEqual([manifest.options.b4_h_low_V manifest.options.b4_h_high_V],[100 200]);
            tc.verifyEqual(manifest.policy.parameters.max_change_m_s,.2,'AbsTol',1e-12);
            csv=readtable(fullfile(root,'missions.csv'));
            tc.verifyEqual(csv.b4_speed_reductions,n-1);tc.verifyEqual(csv.b4_h_low_V,100);
            tc.verifyEqual(r.failed_qps,0);
            tc.verifyTrue(r.all_datasets_validated);
            % Raise requested every decision (headroom above h_high, v < v*),
            % but v_cap(estimated SOC) = 0.8 m/s blocks it.
            o2=setfield(setfield(setfield(o,'b4_h_low_V',-100),'b4_h_high_V',-50),'b4_v_cap_table',[0 .8;1 .8]); %#ok<SFLD>
            root=fullfile(tc.Folder,'b4_cap');
            evalc('r=run_energy_mission_shift_evaluation(root,o2);');
            m=r.missions;
            choice=jsondecode(fileread(fullfile(root,'mission_01_policy_choice.json')));
            d=choice.decisions;n=numel(d);
            tc.verifyEqual([d.command_m_s].',.8*ones(n,1),'AbsTol',1e-12,'Never above the cap.');
            tc.verifyEqual({d(2:end).event}.',repmat({'raise'},n-1,1));
            tc.verifyEqual(m.b4_speed_increases,0);tc.verifyEqual(m.b4_speed_reductions,0);
            tc.verifyEqual(m.b4_cap_limited_decisions,n);tc.verifyTrue(m.b4_v_cap_from_table);
            tc.verifyEqual(m.chosen_speed_m_s,.8,'AbsTol',1e-12);
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
            tc.verifyTrue(isnan(r.missions.chosen_speed_m_s),'The actor has no constant speed.');
        end
    end
end
