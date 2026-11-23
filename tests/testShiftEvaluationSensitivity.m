classdef testShiftEvaluationSensitivity < matlab.unittest.TestCase
    % Post-hoc sensitivity variants of the deployable shift rules (review of
    % 23 Nov 2026): B3 planned from (estimated SOC - b3_soc_margin), and B4
    % started at b4_v_start_m_s (soft start) instead of v*. The defaults must
    % reproduce the frozen rules. Short missions test the plumbing only.
    properties
        Folder
        TablePath
    end
    methods(TestClassSetup)
        function prepare(tc)
            source=bootstrap_RF_MPC_RL();
            [status,text]=git_output(source,'status --porcelain');
            tc.assumeTrue(status==0 && isempty(strtrim(text)),'Evaluation needs a clean committed source tree.');
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fixture.Folder;
            tc.TablePath=fullfile(tc.Folder,'synthetic_sweep.csv');
            writetable(localSynthetic(0.30:0.10:1.00),tc.TablePath);
        end
    end
    methods(Test)
        function variantsApplyOnlyToTheirRule(tc)
            o=struct('policy','fixed','speed_m_s',1.0,'b3_soc_margin',.05);
            tc.verifyError(@()run_energy_mission_shift_evaluation(fullfile(tc.Folder,'x1'),o),'shiftEval:Options');
            o=struct('policy','fixed','speed_m_s',1.0,'b4_v_start_m_s',.85);
            tc.verifyError(@()run_energy_mission_shift_evaluation(fullfile(tc.Folder,'x2'),o),'shiftEval:Options');
            o=struct('policy','b3_shift','table',tc.TablePath,'b3_soc_margin',.4);
            tc.verifyError(@()run_energy_mission_shift_evaluation(fullfile(tc.Folder,'x3'),o),'MATLAB:expectedLessThanOrEqual');
        end

        function b4SoftStartHoldsUntilTheFirstHeadroomReading(tc)
            base=struct('policy','b4','start_soc',.95,'min_start_soc',.9495,'smoke_distance_m',3,'smoke_duration_s',10);
            evalc('r0=run_energy_mission_shift_evaluation(fullfile(tc.Folder,''b4''),base);');
            soft=base;soft.b4_v_start_m_s=.85;
            evalc('r1=run_energy_mission_shift_evaluation(fullfile(tc.Folder,''b4soft''),soft);');
            c0=jsondecode(fileread(fullfile(tc.Folder,'b4','mission_01_policy_choice.json')));
            c1=jsondecode(fileread(fullfile(tc.Folder,'b4soft','mission_01_policy_choice.json')));
            tc.verifyEqual(c0.decisions(1).command_m_s,1.0,'Frozen B4 starts at v*.');
            tc.verifyEqual(c0.decisions(1).event,'start');
            tc.verifyEqual(c1.decisions(1).command_m_s,.85,'Soft start holds the start command.');
            tc.verifyEqual(c1.decisions(1).event,'hold_no_headroom');
            tc.verifyTrue(isnan(r0.missions(1).b4_v_start_m_s));
            tc.verifyEqual(r1.missions(1).b4_v_start_m_s,.85);
            tc.verifyEqual(r1.failed_qps,0);
        end

        function b3MarginPlansFromALowerSoc(tc)
            % Synthetic table of testEnergyMissionB3ShiftSpeed: from 0.69 the
            % rule picks 1.0 m/s; planned from 0.3935 it picks 0.8 m/s.
            base=struct('policy','b3_shift','table',tc.TablePath,'start_soc',.69,'min_start_soc',.6895, ...
                'smoke_distance_m',3,'smoke_duration_s',10);
            evalc('r0=run_energy_mission_shift_evaluation(fullfile(tc.Folder,''b3''),base);');
            m=base;m.b3_soc_margin=.2965;
            evalc('r1=run_energy_mission_shift_evaluation(fullfile(tc.Folder,''b3m''),m);');
            tc.verifyEqual(r0.missions(1).chosen_speed_m_s,1.0);
            tc.verifyEqual(r1.missions(1).chosen_speed_m_s,.8);
            c=jsondecode(fileread(fullfile(tc.Folder,'b3m','mission_01_policy_choice.json')));
            tc.verifyEqual(c.soc_margin,.2965);
            tc.verifyEqual(c.soc_planned,c.soc_estimated-.2965,'AbsTol',1e-12);
        end
    end
end

function t = localSynthetic(starts)
% Same synthetic sweep as testEnergyMissionB3ShiftSpeed (rows default to the
% benchmark role), written to a file so the evaluation runner can load it.
speeds = [0.75 0.8 0.85 0.9 1.0];
charge = 7200*[.09 .0925 .095 .0975 .10];
[S,V] = ndgrid(starts,speeds);
Q = repmat(charge,numel(starts),1);
outcome = repmat("mission_complete",size(S));
fail = V==1.0 & abs(S-0.30)<1e-9;
outcome(fail) = "actuator_limit"; Q(fail) = 600;
t = table(S(:),V(:),outcome(:),Q(:),'VariableNames', ...
    {'initial_soc','requested_speed_m_s','outcome','charge_As'});
end
