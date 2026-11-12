classdef testInitialSocCurriculum < matlab.unittest.TestCase
    % Initial-SOC curriculum (uniform 30-100%, no staging), reward v2 with a
    % start-independent success bonus, fixed-start evaluation, and the
    % carried-battery reset used for continuous depletion.
    properties
        Folder
        Source
    end
    methods(TestClassSetup)
        function setupFolder(tc)
            tc.Source=bootstrap_RF_MPC_RL();
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fixture.Folder;
        end
    end
    methods(Test)
        function defaultsDeclareRangeAndReward(tc)
            c=setup_energy_mission_rl().cfg;
            tc.verifyEqual(c.INITIAL_SOC.mode,'fixed');
            tc.verifyEqual(c.INITIAL_SOC.range,[.30 1]);
            tc.verifyEqual(c.REWARD.version,'reward_energy_mission_v3');
            tc.verifyEqual(c.ENERGY_RL.bonus_reference_soc,1);
            u=setup_energy_mission_rl(struct('initial_soc_mode','uniform')).cfg;
            tc.verifyEqual(u.INITIAL_SOC.mode,'uniform');
            tc.verifyError(@()setup_energy_mission_rl(struct('initial_soc',.25)),'energyMission:InitialSOC');
            tc.verifyEqual(setup_energy_mission_rl(struct('initial_soc',.35)).cfg.BATTERY.SOC_init,.35);
        end

        function successBonusDoesNotDependOnStartSoc(tc)
            low=energy_mission_scales(setup_energy_mission_rl(struct('initial_soc',.30)).cfg);
            high=energy_mission_scales(setup_energy_mission_rl(struct('initial_soc',1)).cfg);
            tc.verifyEqual(low.completion_bonus,high.completion_bonus,'AbsTol',1e-12);
            tc.verifyEqual(low.available_charge_budget_As,7200*.8,'AbsTol',1e-9);
            tc.verifyEqual(low.reward_version,'reward_energy_mission_v3');
            % The standardized costs use spans fixed by configuration, not by start SOC.
            tc.verifyEqual([low.minimum_time_s,low.time_span_s,low.charge_span_As], ...
                [high.minimum_time_s,high.time_span_s,high.charge_span_As],'AbsTol',1e-12);
        end

        function uniformResetSamplesRangeReproducibly(tc)
            first=localSocs(tc,'a');
            second=localSocs(tc,'b');
            tc.verifyEqual(first,second);
            tc.verifyGreaterThanOrEqual(min(first),.30);
            tc.verifyLessThanOrEqual(max(first),1);
            tc.verifyGreaterThan(numel(unique(first)),3);
        end

        function evaluationBundleForcesFixedStart(tc)
            b=setup_energy_mission_rl(struct('initial_soc_mode','uniform'));
            e=energy_mission_evaluation_bundle(b,struct());
            tc.verifyEqual(e.cfg.INITIAL_SOC.mode,'fixed');
            e=energy_mission_evaluation_bundle(b,struct('initial_soc',.45));
            tc.verifyEqual(e.cfg.INITIAL_SOC.mode,'fixed');
            tc.verifyEqual(e.cfg.BATTERY.SOC_init,.45);
            tc.verifyError(@()energy_mission_evaluation_bundle(b,struct('initial_soc',.25)),'energyMission:InitialSOC');
        end

        function carriedResetKeepsBatteryAndRestartsCounters(tc)
            b=setup_energy_mission_rl(struct('initial_soc',.95));
            env=Phase3MpcEnvironment(b,fullfile(tc.Folder,'carry'),localMeta(tc,b,'carry'));
            cleanup=onCleanup(@()localClose(env)); %#ok<NASGU>
            reset(env);
            carried=struct('soc',.6123,'rc',[.01;.02;.03],'estimate',[.61;.011;.019;.031], ...
                'cov',2*env.State.energy.cov);
            reset(env,carried);
            e=env.State.energy;
            tc.verifyEqual([e.soc;e.rc;e.estimate],[carried.soc;carried.rc;carried.estimate]);
            tc.verifyEqual(e.cov,carried.cov);
            tc.verifyEqual([e.chargeAs,e.energyWh],[0 0]);
            tc.verifyEqual(env.State.battery.soc_pct,61.23,'AbsTol',1e-10);
        end
    end
end

function socs=localSocs(tc,name)
b=setup_energy_mission_rl(struct('initial_soc_mode','uniform'));
env=Phase3MpcEnvironment(b,fullfile(tc.Folder,name),localMeta(tc,b,name));
cleanup=onCleanup(@()localClose(env)); %#ok<NASGU>
socs=zeros(6,1);
for k=1:6
    reset(env);
    socs(k)=env.State.energy.soc;
    tc.verifyEqual(env.State.energy.estimate(1),socs(k));
end
end

function m=localMeta(tc,b,name)
m=struct('run_id',['curriculum_' name],'run_type','initial_soc_curriculum_unit', ...
    'monitor_root',fullfile(fileparts(tc.Source),'RL-MPC-Monitor'), ...
    'seed',b.cfg.RNG_SEED,'source_policy','none');
end

function localClose(env)
if ~env.Writer.Closed, close(env,'unit_complete'); end
end
