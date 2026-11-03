function m = loadStep(x0,x1,U,p,dt,time,fsm,tpl,Tmc,e)
% World GRF acts on the foot; actuator support torque is -J' * GRF.
rot=reshape(x0(7:15),3,3); force=reshape(U,3,4); feet=reshape(x0(19:30),3,4);
tau=zeros(3,4); speed=tau;
for j=1:4
    p.sign_L=sign(p.pf34(1,j)); p.sign_d=sign(p.pf34(2,j));
    q=fcn_invKin3(x0,feet(:,j),p)';
    q1=fcn_invKin3(x1,x1(18+(3*j-2:3*j)),p)';
    assert(isreal([q;q1]) && all(isfinite([q;q1])),'robotenergy:LegReach','Leg outside IK domain.');
    speed(:,j)=atan2(sin(q1-q),cos(q1-q))/dt;
    % Analytic Jacobian of the existing IK's forward geometry.
    ax=[1;0;0]; ay=[0;cos(q(1));sin(q(1))];
    offset=p.sign_d*p.d*[0;cos(q(1));sin(q(1))];
    hip=rot'*(feet(:,j)-x0(1:3))-[p.sign_L*p.L/2;p.sign_d*p.W/2;0];
    link=p.l1*[cos(q(2));sin(q(1))*sin(q(2));-cos(q(1))*sin(q(2))];
    J=[cross(ax,hip),cross(ay,hip-offset),cross(ay,hip-offset-link)];
    tau(:,j)=-J'*(rot'*force(:,j));
    if fsm.FSM(j)==2
        span=fsm.Tb(j)-fsm.Ta(j);
        tau(3,j)=tau(3,j)+tpl.tauSw((time-fsm.Ta(j))/span)*((1-tpl.betaMc)*Tmc/span)^2;
    end
end
t=tau+e.damping*speed+e.dryFriction*sign(speed);
mech=t.*speed; motor=t./e.ratio;
drive=mech>=0; motor(drive)=motor(drive)/e.gearEfficiency;
motor(~drive)=motor(~drive)*e.gearEfficiency;
i=motor/e.phaseKt; copper=.75*e.lineResistance*i.^2;
shaft=motor.*(e.ratio.*speed);
m.watts=e.auxW+sum(max(0,shaft+copper),'all')/e.inverterEfficiency;
m.peakTorque=max(abs(motor),[],'all');
[m.requiredV,k]=max(abs(e.lineResistance*i+2*e.fluxPolePairs*e.ratio.*speed),[],'all','linear');
[m.voltageAxis,m.voltageLeg]=ind2sub(size(speed),k); m.voltageFSM=fsm.FSM(m.voltageLeg);
m.copperW=sum(copper,'all'); m.jointTorque=tau; m.jointSpeed=speed;
end
