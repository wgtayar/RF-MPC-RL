classdef testFixedPackLegacyTiming < matlab.unittest.TestCase
    % Run the actual legacy entrypoint bodies in an isolated directory because
    % their configuration/LastR paths are relative to mfilename('fullpath').
    properties
        Bundle
        Folder
    end
    methods (TestMethodSetup)
        function configure(tc)
            src=bootstrap_RF_MPC_RL();
            tc.Bundle=setup_RL_MPC();
            tc.Bundle.cfg.CHUNK_DURATION=.02;
            tc.Bundle.cfg.APPLY_EVERY=1;
            tc.Bundle.cfg.EP_STEPS=10;
            tc.Bundle.cfg.MISSION_DURATION=.2;
            tc.Bundle.cfg.MISSION.D_TARGET_M=1;
            tc.Bundle.cfg.LOG.enable=false;
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fixture.Folder;
            tc.Bundle.cfg.RUN.sim_snapshot_file=fullfile(tc.Folder,'SimSnapshot_RL.mat');
            localIsolatedCopy(src,tc.Folder,'rlStepFunction','fixedPackLegacyStepUnderTest');
            localIsolatedCopy(src,tc.Folder,'rlResetFunction','fixedPackLegacyResetUnderTest');
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(tc.Folder));
        end
    end
    methods (Test)
        function firstStepPowerFailureDoesNotAdvanceMissionClock(tc)
            b=tc.Bundle; b.cfg.ENERGY.auxW=1e8;
            save(fullfile(tc.Folder,'rlEnv_MPC_R.mat'),'-struct','b');
            [~,logged]=fixedPackLegacyResetUnderTest();
            [obs,reward,done,after]=fixedPackLegacyStepUnderTest(localAction(b),logged);
            tc.verifyTrue(done);
            tc.verifyTrue(all(isfinite(obs)) && isfinite(reward));
            tc.verifyEqual(after.episode_time_total,0);
            tc.verifyEqual(after.window.time_frac,0);
            tc.verifyEqual(obs(5),0); % No elapsed-time lag at a rejected first interval.
        end
        function failedNextDecisionRetainsActualAcceptedTime(tc)
            b=tc.Bundle;
            save(fullfile(tc.Folder,'rlEnv_MPC_R.mat'),'-struct','b');
            [~,logged]=fixedPackLegacyResetUnderTest();
            [~,~,done,before]=fixedPackLegacyStepUnderTest(localAction(b),logged);
            tc.assertFalse(done);
            tc.assertEqual(before.episode_time_total,.02,'AbsTol',1e-12);
            % Fault injection changes only the next requested interval.
            b.cfg.ENERGY.auxW=1e8;
            save(fullfile(tc.Folder,'rlEnv_MPC_R.mat'),'-struct','b');
            [obs,reward,done,after]=fixedPackLegacyStepUnderTest(localAction(b),before);
            tc.verifyTrue(done);
            tc.verifyTrue(all(isfinite(obs)) && isfinite(reward));
            tc.verifyEqual(after.episode_time_total,before.episode_time_total);
            tc.verifyEqual(after.window.time_frac,before.episode_time_total/b.cfg.MISSION_DURATION,'AbsTol',1e-12);
            tc.verifyEqual(after.episode_charge_total,before.episode_charge_total);
        end
    end
end

function action=localAction(b)
action=[zeros(3,1);b.cfg.GAMMA_V_MISSION;b.cfg.GAMMA_A_MIN];
end

function localIsolatedCopy(source,destination,name,isolatedName)
code=fileread(fullfile(source,[name '.m']));
% Rename only the entrypoint; the production body remains byte-for-byte.
code=strrep(code,[name '('],[isolatedName '(']);
file=fopen(fullfile(destination,[isolatedName '.m']),'w');
assert(file>=0); cleanup=onCleanup(@()fclose(file));
fwrite(file,code);
end
