classdef testEnergyMissionDataset < matlab.unittest.TestCase
    % Rehash deliberate annotation changes to test semantic reconstruction,
    % independently of the ordinary file-integrity checks.
    properties
        Baseline
        Folder
    end
    methods(TestClassSetup)
        function captureTwoNativeRows(tc)
            source=bootstrap_RF_MPC_RL();
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fixture.Folder;
            tc.Baseline=fullfile(tc.Folder,'baseline');
            bundle=setup_energy_mission_rl(struct('initial_soc',.95, ...
                'target_distance_m',.01,'mission_duration_s',.02,'decision_interval_s',.01));
            metadata=struct('run_id','energy_dataset_unit','run_type','two_row_native_dataset_test', ...
                'monitor_root',fullfile(fileparts(source),'RL-MPC-Monitor'), ...
                'seed',bundle.cfg.RNG_SEED,'source_policy','fixed_0p5mps');
            env=Phase3MpcEnvironment(bundle,tc.Baseline,metadata);
            cleanup=onCleanup(@() localClose(env)); %#ok<NASGU>
            reset(env);
            for k=1:2
                [~,~,done]=step(env,[0;0;0;.25;bundle.cfg.GAMMA_A_MIN]);
                tc.assertEqual(done,k==2);
            end
            tc.assertEqual(env.Writer.RowCount,2);
            close(env,'unit_two_row_capture_complete');
        end
    end
    methods(Test)
        function defaultValidationReconstructsEnergyReward(tc)
            report=validate_exact_state_dataset(tc.Baseline);
            tc.verifyTrue(report.valid);
            tc.verifyEqual(report.windows_reconstructed,2);
            tc.verifyEqual(report.rewards_reconstructed,2);
        end
        function rehashedWrongRewardIsRejected(tc)
            root=clone(tc); relative=decisionPath();
            event=readEvent(root,relative); event.record.reward=event.record.reward+1;
            writeEvent(root,relative,event);
            tc.verifyError(@()validate_exact_state_dataset(root), ...
                'validate_exact_state_dataset:RewardReconstruction');
        end
        function rehashedWrongRewardAuditIsRejected(tc)
            root=clone(tc); relative=decisionPath();
            event=readEvent(root,relative);
            event.record.reward_info.charge_cost=event.record.reward_info.charge_cost+1;
            writeEvent(root,relative,event);
            tc.verifyError(@()validate_exact_state_dataset(root), ...
                'validate_exact_state_dataset:RewardReconstruction');
        end
        function rehashedWrongChargeWindowIsRejected(tc)
            root=clone(tc); relative=decisionPath();
            event=readEvent(root,relative);
            event.record.window.charge_As=event.record.window.charge_As+1;
            writeEvent(root,relative,event);
            tc.verifyError(@()validate_exact_state_dataset(root), ...
                'validate_exact_state_dataset:WindowReconstruction');
        end
        function postChunkElectricalStateMustMatchPhysicalRows(tc)
            root=clone(tc); relative='snapshots/segment_000001_after.mat';
            path=fullfile(root,relative); saved=load(path,'snapshot');
            snapshot=saved.snapshot; snapshot.state.energy.chargeAs=snapshot.state.energy.chargeAs+1;
            save(path,'snapshot','-v7.3'); rehashFile(root,relative);
            tc.verifyError(@()validate_exact_state_dataset(root), ...
                'validate_exact_state_dataset:EnergyChunkLink');
        end
        function decisionElectricalStateMustMatchPhysicalRows(tc)
            root=clone(tc); relative=decisionPath();
            event=readEvent(root,relative);
            event.record.state.energy.chargeAs=event.record.state.energy.chargeAs+1;
            writeEvent(root,relative,event);
            tc.verifyError(@()validate_exact_state_dataset(root), ...
                'validate_exact_state_dataset:EnergyDecisionLink');
        end
    end
end

function root=clone(tc)
root=tempname(tc.Folder); copyfile(tc.Baseline,root);
end
function relative=decisionPath()
relative='decisions/episode_000001_decision_000001.mat';
end
function event=readEvent(root,relative)
saved=load(fullfile(root,relative),'event'); event=saved.event;
end
function writeEvent(root,relative,event)
save(fullfile(root,relative),'event','-v7.3'); rehashFile(root,relative);
end
function rehashFile(root,relative)
path=fullfile(root,relative); indexPath=fullfile(root,'checksums','files.csv');
index=readtable(indexPath,'TextType','string'); row=index.file==string(relative);
assert(nnz(row)==1); details=dir(path);
index.bytes(row)=details.bytes; index.sha256(row)=string(sha256_file(path));
writetable(index,indexPath);
end
function localClose(env)
if ~env.Writer.Closed,close(env,'unit_test_exception');end
end
