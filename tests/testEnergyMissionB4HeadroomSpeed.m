classdef testEnergyMissionB4HeadroomSpeed < matlab.unittest.TestCase
    % Decision rule of the B4 headroom-feedback governor (BASELINES_V3 row
    % B4): start at v*, lower 0.1 m/s below h_low, raise 0.1 m/s above h_high
    % while below v*, never above v_cap(estimated SOC), never by more than the
    % 0.2 m/s per-decision change cap, and hold without a headroom value.
    properties
        P
    end
    methods(TestClassSetup)
        function prepare(tc)
            bootstrap_RF_MPC_RL();
            tc.P = struct('h_low_V',.05,'h_high_V',.15,'v_star_m_s',1.0,'v_cap_table',[], ...
                'v_min_m_s',.3,'v_max_m_s',1.2,'step_m_s',.1,'max_change_m_s',.2);
        end
    end
    methods(Test)
        function startsAtVStarUnderTheCap(tc)
            [v,a] = energy_mission_b4_headroom_speed(NaN,NaN,.95,tc.P);
            tc.verifyEqual(v,1.0);tc.verifyEqual(a.event,'start');
            tc.verifyFalse(a.reduced || a.increased);
            p = tc.P;p.v_cap_table = [.3 .7;.9 .9];
            [v,a] = energy_mission_b4_headroom_speed(NaN,NaN,.6,p);
            tc.verifyEqual(v,.8,'AbsTol',1e-12,'Cap interpolated at SOC 0.6.');
            tc.verifyTrue(a.cap_limited);
            tc.verifyEqual(energy_mission_b4_headroom_speed(NaN,NaN,.95,p),.9,'AbsTol',1e-12,'Clamped above the table.');
            tc.verifyEqual(energy_mission_b4_headroom_speed(NaN,NaN,.1,p),.7,'AbsTol',1e-12,'Clamped below the table.');
        end

        function lowersBelowHLowAndStopsAtTheBound(tc)
            [v,a] = energy_mission_b4_headroom_speed(1.0,.01,.5,tc.P);
            tc.verifyEqual(v,.9,'AbsTol',1e-12);tc.verifyEqual(a.event,'lower');tc.verifyTrue(a.reduced);
            [v,a] = energy_mission_b4_headroom_speed(.3,-1,.5,tc.P);
            tc.verifyEqual(v,.3);tc.verifyFalse(a.reduced,'No command below the action bound.');
            tc.verifyEqual(energy_mission_b4_headroom_speed(1.0,.05,.5,tc.P),1.0,'Equal to h_low holds.');
        end

        function raisesAboveHHighOnlyBelowVStar(tc)
            [v,a] = energy_mission_b4_headroom_speed(.7,.2,.5,tc.P);
            tc.verifyEqual(v,.8,'AbsTol',1e-12);tc.verifyEqual(a.event,'raise');tc.verifyTrue(a.increased);
            tc.verifyEqual(energy_mission_b4_headroom_speed(.95,.2,.5,tc.P),1.0,'AbsTol',1e-12,'Raise stops at v*.');
            [v,a] = energy_mission_b4_headroom_speed(1.0,.2,.5,tc.P);
            tc.verifyEqual(v,1.0);tc.verifyEqual(a.event,'hold');
            tc.verifyEqual(energy_mission_b4_headroom_speed(.7,.1,.5,tc.P),.7,'Inside the band holds.');
            p = tc.P;p.v_cap_table = [0 .7;1 .7];
            [v,a] = energy_mission_b4_headroom_speed(.7,.2,.5,p);
            tc.verifyEqual(v,.7);tc.verifyTrue(a.cap_limited);tc.verifyFalse(a.increased,'No raise above the cap.');
        end

        function holdsWithoutHeadroomAndPrefersLowering(tc)
            [v,a] = energy_mission_b4_headroom_speed(.8,NaN,.5,tc.P);
            tc.verifyEqual(v,.8);tc.verifyEqual(a.event,'hold_no_headroom');
            p = tc.P;p.h_low_V = .1;p.h_high_V = .1;
            tc.verifyEqual(energy_mission_b4_headroom_speed(.8,.05,.5,p),.7,'AbsTol',1e-12);
        end

        function capDropRespectsTheChangeCap(tc)
            p = tc.P;p.v_cap_table = [0 .5;1 .5];
            [v,a] = energy_mission_b4_headroom_speed(1.0,.1,.5,p);
            tc.verifyEqual(v,.8,'AbsTol',1e-12,'At most 0.2 m/s per decision.');
            tc.verifyTrue(a.cap_limited && a.rate_limited && a.reduced);
        end

        function rejectsInvalidParameters(tc)
            p = tc.P;p.h_low_V = .2;
            tc.verifyError(@()energy_mission_b4_headroom_speed(NaN,NaN,.5,p),'b4Governor:Thresholds');
            p = tc.P;p.v_cap_table = [.9 1;.3 .8];
            tc.verifyError(@()energy_mission_b4_headroom_speed(NaN,NaN,.5,p),'b4Governor:CapTable');
            tc.verifyError(@()energy_mission_b4_headroom_speed(NaN,NaN,.5,rmfield(tc.P,'h_low_V')),'b4Governor:Params');
        end
    end
end
