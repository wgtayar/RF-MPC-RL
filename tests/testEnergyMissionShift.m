classdef testEnergyMissionShift < matlab.unittest.TestCase
    % Shift training (monitor #40): missions chained on one battery inside the
    % learner's episode, a mission cap stored as truncation (IsDone=2) that the
    % next episode continues, and a normal end when the battery is too low.
    properties
        Folder
        Source
    end
    methods(TestMethodSetup)
        function makeFolder(tc)
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fixture.Folder;
            tc.Source=bootstrap_RF_MPC_RL();
        end
    end
    methods(Test)
        function shiftIsOffByDefaultAndValidated(tc)
            b=setup_energy_mission_rl();
            tc.verifyFalse(b.cfg.ENERGY_RL.options.shift.enabled);
            tc.verifyEqual(b.cfg.ENERGY_RL.options.shift.max_missions,4);
            tc.verifyEqual(b.cfg.ENERGY_RL.options.shift.min_start_soc,.30);
            tc.verifyError(@()setup_energy_mission_rl(struct('shift',struct('typo',1))),'energyMission:Options');
            p=setup_energy_mission_rl(struct('shift',struct('enabled',true)));
            tc.verifyEqual(p.cfg.ENERGY_RL.options.shift.max_missions,4);
        end

        function missionsChainCarryTheBatteryAndTruncateAtTheCap(tc)
            env=localEnv(tc,'chain',struct('enabled',true,'max_missions',2,'min_start_soc',.30));
            cleanup=onCleanup(@()localClose(env)); %#ok<NASGU>
            reset(env);
            first=env.Native.Episode;
            [done1,info1]=localRunMission(tc,env);
            tc.verifyEqual(done1,0,'A completed mission with charge left continues the episode.');
            tc.verifyEqual(env.Native.Episode,first+1,'The next mission is a new captured native episode.');
            tc.verifyEqual(env.Native.Decision,0);
            e=env.Native.State.energy; prev=info1.state.energy;
            tc.verifyEqual([e.soc;e.rc;e.estimate],[prev.soc;prev.rc;prev.estimate],'Battery carried exactly.');
            tc.verifyEqual(e.cov,prev.cov);
            tc.verifyEqual([e.chargeAs e.energyWh],[0 0],'Per-mission counters restart.');
            tc.verifyEqual(env.Native.State.decision_bookkeeping.distance_m,0,'Robot restarts its mission.');
            [done2,info2]=localRunMission(tc,env);
            tc.verifyEqual(done2,2,'The mission cap is a truncation (IsDone=2), not a terminal.');
            pending=env.Native.Episode;
            observation=reset(env);
            tc.verifyEqual(env.Native.Episode,pending,'The next learner episode continues the started mission.');
            tc.verifyEqual(observation,env.Native.Observation);
            tc.verifyEqual(env.Native.State.energy.soc,info2.state.energy.soc,'Battery carried across learner episodes.');
            log=readtable(fullfile(env.Native.Writer.Root,'shift_missions.csv'));
            tc.verifyEqual(log.learner_done,[0;2]);
            tc.verifyEqual(log.shift,[1;1]);
            tc.verifyEqual(log.mission_in_episode,[1;2]);
        end

        function shiftEndsNormallyWhenTooLowForAnotherMission(tc)
            env=localEnv(tc,'end',struct('enabled',true,'max_missions',4,'min_start_soc',.99));
            cleanup=onCleanup(@()localClose(env)); %#ok<NASGU>
            reset(env);
            [done,info]=localRunMission(tc,env);
            tc.verifyEqual(done,1,'Below min_start_soc the shift ends (IsDone=1).');
            tc.verifyEqual(char(info.terminal_reason),'mission_complete','A normal end, not a failure.');
            tc.verifyEqual(info.reward_info.terminal_contribution,1,'The completion bonus is kept; no penalty.');
            n=env.Native.Episode;
            reset(env);
            tc.verifyEqual(env.Native.Episode,n+1,'After a shift ends, a fresh start-SOC draw begins a new shift.');
            log=readtable(fullfile(env.Native.Writer.Root,'shift_missions.csv'));
            tc.verifyEqual(log.learner_done,1);
        end

        function shiftDisabledKeepsSingleMissionEpisodes(tc)
            env=localEnv(tc,'off',struct('enabled',false));
            cleanup=onCleanup(@()localClose(env)); %#ok<NASGU>
            reset(env);
            [done,info]=localRunMission(tc,env);
            tc.verifyTrue(logical(done));
            tc.verifyEqual(char(info.terminal_reason),'mission_complete');
            tc.verifyFalse(isfile(fullfile(env.Native.Writer.Root,'shift_missions.csv')));
        end
    end
end

function env=localEnv(tc,name,shift)
setup=struct('initial_soc_mode','uniform','initial_soc_range',[.90 .95], ...
    'target_distance_m',3,'mission_duration_s',10,'shift',shift);
b=setup_energy_mission_rl(setup);
m=struct('run_id',['shift_' name],'run_type','shift_unit', ...
    'monitor_root',fullfile(fileparts(tc.Source),'RL-MPC-Monitor'), ...
    'seed',b.cfg.RNG_SEED,'source_policy','scripted_fast');
env=EnergyMissionTrainingEnvironment(b,fullfile(tc.Folder,name),m);
end

function [done,info]=localRunMission(tc,env)
% Fastest legal command so a short mission completes within its 2 decisions.
start=env.Native.Episode;
for k=1:env.Native.Config.EP_STEPS
    [~,~,done,info]=step(env,[0;0;0;1;0]);
    if done~=0 || env.Native.Episode~=start, return, end
end
tc.assertFail('The scripted mission did not end within its horizon.');
end

function localClose(env)
if ~env.Native.Writer.Closed, close(env,'unit_complete'); end
end
