classdef testEnergyMissionB3ShiftSpeed < matlab.unittest.TestCase
    % B3-shift charge-budget rule (protocol A3.3) on a synthetic sweep table.
    % Charges are constant in start SOC: dSOC = 0.09 at 0.75 m/s, 0.0925 at
    % 0.8, 0.095 at 0.85, 0.0975 at 0.9 and 0.10 at 1.0 m/s. Starts are
    % 0.30:0.10:1.00 and 1.0 m/s fails at 0.30. With these values the frugal
    % shift count is N(s) = floor((s-0.30)/0.09)+1 for s >= 0.30, else 0.
    % Test SOCs avoid exact boundaries of that expression.
    properties
        Table
    end
    methods(TestClassSetup)
        function prepare(tc)
            bootstrap_RF_MPC_RL();
            tc.Table = localSynthetic(0.30:0.10:1.00);
        end
    end
    methods(Test)
        function fastestSpeedWhenNoMissionIsLost(tc)
            [v,a] = energy_mission_b3_shift_speed(tc.Table,0.95);
            tc.verifyEqual(v,1.0);
            tc.verifyEqual(a.rule,'fastest_qualified');
            tc.verifyEqual(a.missions_frugal,8);
            tc.verifyEqual(a.missions_after(1),7,'N(0.85) after a 1.0 m/s mission.');
            tc.verifyEqual(a.dsoc.',[.10 .0975 .095 .0925 .09],'AbsTol',1e-12);
        end

        function missionCountGuardSelectsASlowerSpeed(tc)
            % From 0.3935 the frugal plan completes 2 missions. 1.0, 0.9 and
            % 0.85 m/s end below 0.30 (1 mission); 0.8 m/s ends at 0.301.
            [v,a] = energy_mission_b3_shift_speed(tc.Table,0.3935);
            tc.verifyEqual(v,0.8);
            tc.verifyEqual(a.missions_frugal,2);
            tc.verifyEqual(a.qualified.',[false false false true true]);
            tc.verifyEqual(a.missions_after.',[0 0 0 1 1]);
        end

        function failedSweepCellExcludesTheSpeed(tc)
            % Nearest start at or below 0.33 is 0.30, where 1.0 m/s failed.
            [v,a] = energy_mission_b3_shift_speed(tc.Table,0.33);
            tc.verifyEqual(v,0.9);
            tc.verifyFalse(a.completes(1));
            tc.verifyGreaterThanOrEqual(a.end_soc(1),0.21,'1.0 m/s is excluded by (a) alone.');
            % At 0.41 the nearest start at or below is 0.40, where 1.0 m/s completed.
            tc.verifyEqual(energy_mission_b3_shift_speed(tc.Table,0.41),1.0);
        end

        function reserveMarginExcludesFastSpeeds(tc)
            % From 0.302 only 0.75 m/s ends at or above 0.21 (0.212).
            [v,a] = energy_mission_b3_shift_speed(tc.Table,0.302);
            tc.verifyEqual(v,0.75);
            tc.verifyEqual(a.rule,'fastest_qualified');
            tc.verifyEqual(a.qualified.',[false false false false true]);
            tc.verifyEqual(a.end_soc(4),0.302-.0925,'AbsTol',1e-12);
        end

        function noQualifyingSpeedFallsBackToFrugal(tc)
            [v,a] = energy_mission_b3_shift_speed(tc.Table,0.2995);
            tc.verifyEqual(v,0.75);
            tc.verifyEqual(a.rule,'fallback_frugal');
            tc.verifyEqual(a.missions_frugal,0,'No mission starts below 0.30.');
            tc.verifyFalse(any(a.qualified));
        end

        function belowLowestStartUsesTheLowestStart(tc)
            % Table starts at 0.40: at 0.35, (a) is judged at 0.40, where
            % 1.0 m/s completed, and the charge is clamped at the 0.40 value.
            t = localSynthetic(0.40:0.10:1.00);
            [v,a] = energy_mission_b3_shift_speed(t,0.35);
            tc.verifyEqual(v,1.0);
            tc.verifyTrue(a.completes(1));
            tc.verifyEqual(a.missions_frugal,1);
            tc.verifyEqual(a.charge_As(1),720,'AbsTol',1e-9);
        end

        function chargeIsInterpolatedAndClamped(tc)
            t = localSynthetic(0.30:0.10:1.00);
            t.charge_As(t.requested_speed_m_s==1.0 & t.initial_soc==0.40) = 700;
            t.charge_As(t.requested_speed_m_s==1.0 & t.initial_soc==0.50) = 740;
            [~,a] = energy_mission_b3_shift_speed(t,0.45);
            tc.verifyEqual(a.charge_As(1),720,'AbsTol',1e-9);
            tc.verifyEqual(a.dsoc(1),0.10,'AbsTol',1e-12);
            [~,a] = energy_mission_b3_shift_speed(t,0.35);
            tc.verifyEqual(a.charge_As(1),700,'AbsTol',1e-9,'Failed cells are not interpolation nodes.');
            [~,a] = energy_mission_b3_shift_speed(t,1.00);
            tc.verifyEqual(a.charge_As(5),648,'AbsTol',1e-9);
        end

        function csvPathsAreConcatenatedAndFiltered(tc)
            folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            t = tc.Table;
            t.role = repmat("benchmark",height(t),1);
            t.dataset_validated = repmat("True",height(t),1);
            extra = t(1,:);
            extra.role = "version_cross_check";      % would duplicate a cell if kept
            extra.outcome = "mission_complete";
            low = t.initial_soc < 0.65;
            a = fullfile(folder,'a.csv'); b = fullfile(folder,'b.csv');
            writetable([t(low,:);extra],a);
            writetable(t(~low,:),b);
            for s = [0.33 0.3935 0.95]
                tc.verifyEqual(energy_mission_b3_shift_speed({a,b},s), ...
                    energy_mission_b3_shift_speed(tc.Table,s));
            end
            tc.verifyEqual(energy_mission_b3_shift_speed(string({b,a}),0.33),0.9);
            % One file alone (starts 0.30-0.60): (a) and Q are judged at 0.60.
            tc.verifyEqual(energy_mission_b3_shift_speed(a,0.95),1.0);
            tc.verifyError(@()energy_mission_b3_shift_speed(fullfile(folder,'missing.csv'),0.5),'b3Shift:Table');
        end

        function duplicateCellsAreRejected(tc)
            t = [tc.Table;tc.Table(3,:)];
            tc.verifyError(@()energy_mission_b3_shift_speed(t,0.5),'b3Shift:DuplicateCell');
        end

        function structInputMatchesTableInput(tc)
            s = table2struct(tc.Table,'ToScalar',true);
            tc.verifyEqual(energy_mission_b3_shift_speed(s,0.3935),0.8);
        end
    end
end

function t = localSynthetic(starts)
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
