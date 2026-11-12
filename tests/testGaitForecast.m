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
