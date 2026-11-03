classdef testInstantaneousDrive < matlab.unittest.TestCase
    methods(Test)
        function analyticSpeedMatchesDirectionalIKDerivative(tc)
            [s,p,e]=fixture(); x=s.Xt; dx=zeros(30,1);
            dx(1:3)=[.02;-.01;.015]; rot=reshape(x(7:15),3,3);
            dx(7:15)=reshape(rot*hatMap([.04;-.03;.02]),9,1);
            dx(19:30)=repmat([.006;.002;-.003],4,1);
            f=struct('FSM',ones(4,1)); F=repmat([2;-1;13],4,1);
            m=robotenergy.instantaneousLoad(x,dx,F,p,0,f,s.knee_template,s.knee_parameters.Tmc,e);
            h=1e-7; numeric=zeros(3,4);
            for j=1:4
                p.sign_L=sign(p.pf34(1,j)); p.sign_d=sign(p.pf34(2,j)); idx=18+(3*j-2:3*j);
                xp=x+h*dx; xm=x-h*dx;
                qp=fcn_invKin3(xp,xp(idx),p)'; qm=fcn_invKin3(xm,xm(idx),p)';
                numeric(:,j)=atan2(sin(qp-qm),cos(qp-qm))/(2*h);
            end
            tc.verifyEqual(m.jointSpeed,numeric,'AbsTol',1e-8);
        end
        function staticLoadMatchesIntervalProvider(tc)
            [s,p,e]=fixture(); F=repmat([0;0;p.mass*p.g/4],4,1); f=struct('FSM',ones(4,1));
            a=robotenergy.instantaneousLoad(s.Xt,zeros(30,1),F,p,0,f,s.knee_template,s.knee_parameters.Tmc,e);
            b=robotenergy.loadStep(s.Xt,s.Xt,F,p,.01,0,f,s.knee_template,s.knee_parameters.Tmc,e);
            tc.verifyEqual(a.jointSpeed,zeros(3,4));
            for n={'jointTorque','phaseCurrent','motorVoltage','watts','peakTorque'}
                tc.verifyEqual(a.(n{1}),b.(n{1}),'AbsTol',1e-13);
            end
        end
        function servoDerivativeUsesActualTargetAndBodyMotion(tc)
            [s,p,e]=fixture(); x=s.Xt;
            x(4:6)=[.2;-.01;.015]; x(16:18)=[.01;-.02;.03];
            xd=x; xd(19)=xd(19)+.01; F=repmat([0;0;p.mass*p.g/4],4,1);
            [~,p.p_ext]=fcn_get_disturbance(0,p);
            dx=dynamics_SRB(0,x,F,xd,zeros(3,1),p);
            f=struct('FSM',ones(4,1));
            m=robotenergy.instantaneousLoad(x,dx,F,p,0,f,s.knee_template,s.knee_parameters.Tmc,e);
            tc.verifyGreaterThan(max(abs(m.jointSpeed),[],'all'),0);
            tc.verifyEqual(.75*m.motorVoltage.*m.phaseCurrent, ...
                m.motorTorque.*m.motorSpeed+.75*e.lineResistance*m.phaseCurrent.^2,'AbsTol',1e-12);
            tc.verifyEqual(dx(19:30),p.Kp_sw*(xd(19:30)-x(19:30)));
        end
        function outOfReachIsRejected(tc)
            [s,p,e]=fixture(); x=s.Xt; x(19:21)=[5;5;5];
            tc.verifyError(@()robotenergy.instantaneousLoad(x,zeros(30,1),zeros(12,1),p,0, ...
                struct('FSM',ones(4,1)),s.knee_template,s.knee_parameters.Tmc,e),'robotenergy:LegReach');
        end
    end
end
function [s,p,e]=fixture()
b=setup_RL_MPC(); s=initialize_mpc_replay_state(b.cfg,0); p=get_params(0); e=b.cfg.ENERGY;
end
