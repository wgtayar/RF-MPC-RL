classdef testFixedPackMath < matlab.unittest.TestCase
    methods(Test)
        function exactLocalJacobiansMatchFiniteDifferences(tc)
            s=robotenergy.start(phase3battery.manuscriptConfig());
            z=[.923;.02;-.01;.03]; i=2.7; dt=.01; h=1e-7;
            [xp,~,F,H]=robotenergy.linearizeEstimator(s,z,i,dt);
            numericalF=zeros(4); numericalH=zeros(1,4);
            for k=1:4
                d=zeros(4,1); d(k)=h;
                plus=robotenergy.linearizeEstimator(s,z+d,i,dt);
                minus=robotenergy.linearizeEstimator(s,z-d,i,dt);
                numericalF(:,k)=(plus-minus)/(2*h);
                numericalH(k)=(measurement(s,xp+d,i,dt)-measurement(s,xp-d,i,dt))/(2*h);
            end
            tc.verifyEqual(F,numericalF,'AbsTol',2e-8);
            tc.verifyEqual(H,numericalH,'AbsTol',2e-8);
            tc.verifyGreaterThan(norm(F(2:4,1)),0);
        end
        function estimatorUsesThePlantsIntervalTiming(tc)
            s=robotenergy.start(phase3battery.manuscriptConfig());
            s.soc=.923; s.rc=[.02;-.01;.03];
            s.estimate=[s.soc;s.rc];
            [after,y]=robotenergy.advance(s,120,.01);
            [predicted,predictedV]=robotenergy.linearizeEstimator(s,s.estimate,y.cellA,.01);
            tc.verifyEqual(predicted,[after.soc;after.rc],'AbsTol',1e-14);
            tc.verifyEqual(predictedV,y.cellV,'AbsTol',1e-14);
            tc.verifyEqual(after.estimate,predicted,'AbsTol',1e-14);
            tc.verifyGreaterThanOrEqual(min(eig(after.cov)),-1e-14);
            tc.verifyEqual(after.cov,after.cov','AbsTol',1e-14);
        end
        function lookupUsesActualPolynomialDerivative(tc)
            s=robotenergy.start(phase3battery.manuscriptConfig());
            z=.923; h=1e-7;
            [~,~,~,slope]=robotenergy.lookup(s,z);
            tc.verifyEqual(slope,(ppval(s.ocv,z+h)-ppval(s.ocv,z-h))/(2*h),'AbsTol',1e-8);
        end
        function lookupBoundaryAndKnotConventions(tc)
            s=robotenergy.start(phase3battery.manuscriptConfig());
            for z=[s.grid(1)-.01,s.grid(end)+.01]
                [~,~,dr,slope]=robotenergy.lookup(s,z);
                tc.verifyEqual(dr,zeros(7,1)); tc.verifyEqual(slope,0);
            end
            [~,~,dr]=robotenergy.lookup(s,s.grid(2));
            tc.verifyEqual(dr,((s.values(3,:)-s.values(2,:))/(s.grid(3)-s.grid(2)))');
            [~,~,dr]=robotenergy.lookup(s,s.grid(end));
            tc.verifyEqual(dr,((s.values(end,:)-s.values(end-1,:))/(s.grid(end)-s.grid(end-1)))');
        end
        function zeroLoadConservesChargeAndEnergy(tc)
            s=robotenergy.start(phase3battery.manuscriptConfig());
            s.rc=[.02;.01;.03]; s.estimate=[s.soc;s.rc];
            [a,y]=robotenergy.advance(s,0,.01);
            tc.verifyEqual(a.soc,s.soc); tc.verifyEqual(a.chargeAs,0); tc.verifyEqual(a.energyWh,0);
            tc.verifyEqual(y.packA,0);
            tc.verifyEqual(a.estimate,[a.soc;a.rc],'AbsTol',1e-14);
        end
        function unversionedStatesKeepHistoricalEstimator(tc)
            s=robotenergy.start(phase3battery.manuscriptConfig());
            old=rmfield(s,{'estimatorVersion','ocvDerivative'});
            [a,y]=robotenergy.advance(old,120,.01);
            z=old.estimate; z(1)=z(1)-y.cellA*.01/(3600*old.capacity);
            [r,o]=robotenergy.lookup(old,z(1)); alpha=exp(-.01./(r(2:4).*r(5:7)));
            z(2:4)=alpha.*z(2:4)+(1-alpha).*r(2:4)*y.cellA;
            h=interp1(old.grid,old.slope,z(1),'linear');
            F=diag([1;alpha]); C=F*old.cov*F'+old.Q; H=[h -1 -1 -1];
            K=C*H'/(H*C*H'+old.Rv);
            z=z+K*(y.cellV-(o-r(1)*y.cellA-sum(z(2:4))));
            tc.verifyEqual(a.estimate,z,'AbsTol',1e-14);
            tc.verifyFalse(isfield(a,'estimatorVersion'));
        end
        function explicitEstimatorVersionAndLegacyDefault(tc)
            b=phase3battery.manuscriptConfig(); s=robotenergy.start(b);
            tc.verifyEqual(s.estimatorVersion,'fixed_pack_exact_jacobian_ekf_v2');
            old=rmfield(b,'streaming_estimator_version'); s=robotenergy.start(old);
            tc.verifyEqual(s.estimatorVersion,'fixed_pack_streaming_3rc_ekf_v1');
            b.streaming_estimator_version='unknown';
            tc.verifyError(@()robotenergy.start(b),'robotenergy:EstimatorVersion');
        end
        function bodyMotionVirtualWorkAndElectricalPower(tc)
            b=setup_RL_MPC('legacy_frozen_v1'); s=initialize_mpc_replay_state(b.cfg,0); p=get_params(0);
            x0=s.Xt; x1=x0; dt=1e-6;
            v=[.02;-.01;.015]; omega=[.04;-.03;.02];
            rot=reshape(x0(7:15),3,3);
            x1(1:3)=x0(1:3)+v*dt;
            x1(7:15)=reshape(rot*expm(hatMap(omega)*dt),9,1);
            forces=[2 -1 3 -2; -1 2 -2 1; 12 14 13 15];
            m=robotenergy.loadStep(x0,x1,forces(:),p,dt,0,struct('FSM',ones(4,1)), ...
                s.knee_template,s.knee_parameters.Tmc,b.cfg.ENERGY);
            feet=reshape(x0(19:30),3,4);
            pointVelocity=v+hatMap(rot*omega)*(feet-x0(1:3));
            tc.verifyEqual(sum(m.jointTorque.*m.jointSpeed,'all'), ...
                sum(forces.*pointVelocity,'all'),'AbsTol',1e-6);
            % The matched MIT equivalent factors obey P=(3/4)*v*i, before
            % per-motor dissipative-braking clipping and inverter losses.
            expected=m.motorTorque.*m.motorSpeed+ ...
                .75*b.cfg.ENERGY.lineResistance*m.phaseCurrent.^2;
            tc.verifyEqual(.75*m.motorVoltage.*m.phaseCurrent,expected,'AbsTol',1e-12);
            tc.verifyEqual(m.watts,b.cfg.ENERGY.auxW+sum(max(0,expected),'all')/ ...
                b.cfg.ENERGY.inverterEfficiency,'AbsTol',1e-12);
        end
    end
end
function v=measurement(s,z,i,dt)
    [r,o]=robotenergy.lookup(s,z(1)+i*dt/(3600*s.capacity));
    v=o-r(1)*i-sum(z(2:4));
end
