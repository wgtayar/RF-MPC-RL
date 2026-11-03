function m = instantaneousLoad(x,xdot,U,p,time,fsm,tpl,Tmc,e)
%INSTANTANEOUSLOAD Diagnostic demand from the actual SRB state derivative.
% This does not replace the historical interval-average production provider.
% Supply xdot from dynamics_SRB with the same held force/reference/FSM used
% inside the ODE interval. A sample is not a bound on the continuous-time peak.
validateattributes(x,{'numeric'},{'real','finite','vector','numel',30});
validateattributes(xdot,{'numeric'},{'real','finite','vector','numel',30});
x=x(:); xdot=xdot(:);
rot=reshape(x(7:15),3,3); rotDot=reshape(xdot(7:15),3,3);
feet=reshape(x(19:30),3,4); feetDot=reshape(xdot(19:30),3,4);
force=reshape(U,3,4); tau=zeros(3,4); speed=tau; reciprocalCondition=zeros(1,4);
for j=1:4
    p.sign_L=sign(p.pf34(1,j)); p.sign_d=sign(p.pf34(2,j));
    q=fcn_invKin3(x,feet(:,j),p)';
    assert(isreal(q) && all(isfinite(q)),'robotenergy:LegReach','Leg outside IK domain.');
    ax=[1;0;0]; ay=[0;cos(q(1));sin(q(1))];
    offset=p.sign_d*p.d*[0;cos(q(1));sin(q(1))];
    relative=feet(:,j)-x(1:3);
    hip=rot'*relative-[p.sign_L*p.L/2;p.sign_d*p.W/2;0];
    link=p.l1*[cos(q(2));sin(q(1))*sin(q(2));-cos(q(1))*sin(q(2))];
    J=[cross(ax,hip),cross(ay,hip-offset),cross(ay,hip-offset-link)];
    % Differentiate exactly the body-frame vector used by fcn_invKin3.
    hipDot=rotDot'*relative+rot'*(feetDot(:,j)-xdot(1:3));
    reciprocalCondition(j)=rcond(J);
    assert(reciprocalCondition(j)>0,'robotenergy:LegReach','Singular leg Jacobian.');
    speed(:,j)=J\hipDot;
    assert(all(isfinite(speed(:,j))),'robotenergy:LegReach','Nonfinite joint derivative.');
    tau(:,j)=-J'*(rot'*force(:,j));
    if fsm.FSM(j)==2
        span=fsm.Tb(j)-fsm.Ta(j);
        tau(3,j)=tau(3,j)+tpl.tauSw((time-fsm.Ta(j))/span)*((1-tpl.betaMc)*Tmc/span)^2;
    end
end
t=tau+e.damping*speed+e.dryFriction*sign(speed);
motor=t./e.ratio; drive=t.*speed>=0;
motor(drive)=motor(drive)/e.gearEfficiency;
motor(~drive)=motor(~drive)*e.gearEfficiency;
i=motor/e.phaseKt; copper=.75*e.lineResistance*i.^2;
m.motorSpeed=e.ratio.*speed; m.motorTorque=motor; m.phaseCurrent=i;
m.motorVoltage=e.lineResistance*i+2*e.fluxPolePairs*m.motorSpeed;
m.watts=e.auxW+sum(max(0,motor.*m.motorSpeed+copper),'all')/e.inverterEfficiency;
m.peakTorque=max(abs(motor),[],'all');
[m.requiredV,k]=max(abs(m.motorVoltage),[],'all','linear');
[m.voltageAxis,m.voltageLeg]=ind2sub(size(speed),k); m.voltageFSM=fsm.FSM(m.voltageLeg);
m.copperW=sum(copper,'all'); m.jointTorque=tau; m.jointSpeed=speed;
m.jacobianReciprocalCondition=reciprocalCondition;
m.speedConvention='analytic derivative of existing IK under supplied state derivative';
end
