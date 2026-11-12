classdef testGaitForecast < matlab.unittest.TestCase
    % Contact forecast across multiple gait cycles and per-leg swing anchors
    % (fcn_FSM). Witnesses from RL-MPC-Monitor control-authority-20261111-a.
    methods (TestMethodSetup)
        function setup(~)
            bootstrap_RF_MPC_RL();
            clear fcn_FSM
        end
    end
    methods (Test)
        function multiCycleHorizonAlternatesPhases(tc)
            p = get_params(0);
            x = localState(p, 1);
            time = .19 + (0:p.predHorizon-1)*p.Tmpc;
            ta = [0;.175;.175;0];
            s = struct('FSM',ones(4,1),'Ta',ta,'Tb',ta+.2,'pf_R_trans',x(19:30));
            [~,~,ud] = fcn_FSM(time,x,p,s);
            nominal = p.mass*p.g/2;
            tc.verifyEqual(ud([3,6,9,12],end).',[0 nominal nominal 0],'AbsTol',1e-12);
        end

        function forecastMatchesPeriodicTrotSchedule(tc)
            p = get_params(0); rng(7);
            for v = [.5 .7 .8 .9 1 1.1 1.2 1.3]
                Tst = min(p.Tst,.2/v); cycle = Tst + p.Tsw;
                for n = 1:60
                    x = localState(p, v);
                    t0 = 1 + rand*cycle;
                    s = struct('FSM',zeros(4,1),'Ta',zeros(4,1),'Tb',zeros(4,1),'pf_R_trans',x(19:30));
                    valid = true;
                    for leg = 1:4
                        ph = mod(t0 - localOffset(leg,cycle), cycle);
                        if ph < Tst - 1e-6
                            s.FSM(leg) = 1; s.Ta(leg) = t0-ph; s.Tb(leg) = s.Ta(leg)+Tst;
                        elseif ph >= Tst + 1e-6 && ph < cycle - 1e-6
                            s.FSM(leg) = 2; s.Ta(leg) = t0-ph+Tst; s.Tb(leg) = s.Ta(leg)+p.Tsw;
                        else
                            valid = false;
                        end
                    end
                    if ~valid, continue; end
                    tt = t0 + p.Tmpc*(0:p.predHorizon-1);
                    [~,~,ud] = fcn_FSM(tt,x,p,s);
                    for leg = 1:4
                        m = mod(tt - localOffset(leg,cycle), cycle);
                        clear_ = abs(m-Tst) > 1e-3 & m > 1e-3 & cycle-m > 1e-3;
                        truth = m < Tst;
                        got = ud(3*leg,:) > 0;
                        tc.verifyEqual(got(clear_), truth(clear_), sprintf('v=%g leg=%d', v, leg));
                    end
                end
            end
        end

        function speedChangeKeepsSavedActivePhaseEnd(tc)
            % 2026-11-12 review witness: stance fixed at 0.8 m/s (ends 0.25 s),
            % forecast made at 0.22 s while moving at 0.9 m/s. At 0.38 s legs
            % 1/4 are still swinging (until 0.40 s).
            p = get_params(0); p.vel_d = [.9;0]; p.acc_d = .5; p.yaw_d = 0;
            x = localState(p, .8);
            init = struct('FSM',zeros(4,1),'Ta',zeros(4,1),'Tb',ones(4,1),'pf_R_trans',x(19:30));
            [~,~,~,~,s0] = fcn_FSM((0:p.predHorizon-1)*p.Tmpc,x,p,init);
            tc.verifyEqual(s0.Tb([1 4]),[.25;.25],'AbsTol',1e-12);
            x(4) = .9;
            tt = .22 + (0:p.predHorizon-1)*p.Tmpc;
            [~,~,forecast,~,s22] = fcn_FSM(tt,x,p,s0);
            [~,~,~,~,s25] = fcn_FSM(.25+(0:p.predHorizon-1)*p.Tmpc,x,p,s22);
            [actualFSM,~,actual] = fcn_FSM(.38+(0:p.predHorizon-1)*p.Tmpc,x,p,s25);
            tc.verifyEqual(actualFSM,[2;1;1;2]);
            tc.verifyEqual(forecast([3,6,9,12],3),actual([3,6,9,12],1),'AbsTol',1e-12);
            tc.verifyEqual(forecast([3,6,9,12],3).',[0 1 1 0]*p.mass*p.g/2,'AbsTol',1e-12);
        end

        function forecastMatchesClockAcrossSpeedChanges(tc)
            % Advance the state machine on its 10 ms clock while the speed
            % steps every 0.75 s. Within one speed segment every forecast
            % entry must equal the contact state the machine later reaches,
            % including phases whose end was saved at an earlier speed.
            % Samples within one clock step of an actual transition are
            % excluded: the machine switches on the first tick at or after
            % a nominal phase end, which the forecast cannot resolve.
            p = get_params(0);
            speeds = [.6 .75 .85 .95 1.05 1.15 .9 .7];
            segment = .75; dt = .01;
            x = localState(p, speeds(1));
            state = struct('FSM',zeros(4,1),'Ta',zeros(4,1),'Tb',ones(4,1),'pf_R_trans',x(19:30));
            times = 0:dt:(numel(speeds)*segment-dt);
            forecasts = cell(numel(times),1); actual = false(4,numel(times));
            for k = 1:numel(times)
                x(4) = speeds(1+floor(times(k)/segment+1e-9));
                tt = times(k) + (0:p.predHorizon-1)*p.Tmpc;
                [fsm,~,ud,~,state] = fcn_FSM(tt,x,p,state);
                forecasts{k} = struct('t',tt,'stance',ud([3,6,9,12],:)>0);
                actual(:,k) = fsm==1;
            end
            changes = [false(4,1), diff(actual,1,2)~=0];
            checked = 0;
            for k = 1:numel(times)
                f = forecasts{k};
                for ii = 2:numel(f.t)
                    j = find(abs(times-f.t(ii))<1e-9,1);
                    if isempty(j) || floor(f.t(ii)/segment+1e-9) ~= floor(times(k)/segment+1e-9), continue; end
                    for leg = 1:4
                        near = any(changes(leg,max(1,j-1):min(numel(times),j+1)));
                        if near, continue; end
                        tc.verifyEqual(f.stance(leg,ii),actual(leg,j), ...
                            sprintf('leg %d forecast at %.2f for %.2f',leg,times(k),f.t(ii)));
                        checked = checked+1;
                    end
                end
            end
            tc.verifyGreaterThan(checked,1000);
        end

        function exactPhaseBoundaries(tc)
            p = get_params(0); x = localState(p,.5);
            % Leg 1 in stance ending exactly at a forecast sample.
            s = struct('FSM',[1;2;2;1],'Ta',[0;.1;.1;0],'Tb',[.24;.25;.25;.24],'pf_R_trans',x(19:30));
            tt = .08 + (0:p.predHorizon-1)*p.Tmpc;    % .08 .16 .24 .32 .40 .48
            [~,~,ud] = fcn_FSM(tt,x,p,s);
            stance = ud([3,6,9,12],:)>0;
            tc.verifyFalse(stance(1,3));             % t = Tb: swing begins
            tc.verifyTrue(stance(1,2));              % before Tb: stance
            tc.verifyFalse(stance(1,4));             % inside the 0.15 s swing
            tc.verifyTrue(stance(1,5));              % 0.40 = Tb + Tsw: stance again
            tc.verifyTrue(stance(2,4));              % swing ended at 0.25: stance
        end

        function otherLegTransitionKeepsActiveSwingAnchor(tc)
            p = get_params(0);
            x = localState(p, .5); x(19) = x(19) + .02;
            anchor = reshape(p.pf34,[12,1]);
            s = struct('FSM',[2;1;1;1],'Ta',[0;0;0;0],'Tb',[.2;.1;.3;.3],'pf_R_trans',anchor);
            [~,~,~,~,after] = fcn_FSM(.1+(0:p.predHorizon-1)*p.Tmpc,x,p,s);
            tc.verifyEqual(after.FSM(1:2),[2;2]);
            tc.verifyEqual(after.pf_R_trans(1:3),anchor(1:3));
            tc.verifyEqual(after.pf_R_trans(4:6),x(22:24));
        end
    end
end

function x = localState(p, v)
x = zeros(30,1); x(3) = p.z0; x(4) = v;
x(7:15) = reshape(eye(3),9,1); x(19:30) = p.pf34(:);
end

function off = localOffset(leg, cycle)
off = 0; if leg == 2 || leg == 3, off = cycle/2; end
end
