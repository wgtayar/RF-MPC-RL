classdef testTrackedSwing < matlab.unittest.TestCase
% Swing model tracked_bezier_v1: continuous Bezier plan with velocity
% feed-forward, versus the historical held-target servo.
 methods(TestMethodSetup)
  function clearFsm(~)
   clear fcn_FSM
  end
 end
 methods(Test)
  function historicalModelIsTheDefault(tc)
   tc.verifyEqual(swing_model_of(struct()),'held_target_v0');
   tc.verifyEqual(swing_model_of(struct('SWING',struct('model','tracked_bezier_v1'))), ...
       'tracked_bezier_v1');
   tc.verifyError(@()swing_model_of(struct('SWING',struct('model','other'))), ...
       'swing_model_of:Unknown');
  end
  function planMatchesTheReferenceFootTarget(tc)
   [p,Xt,state,t]=localSwingState();
   [~,Xd,~,~,~,ref]=fcn_FSM(t+p.Tmpc*(0:p.predHorizon-1),Xt,p,state);
   tc.verifyEqual(ref.active(:).',[true false false true]);
   for leg=find(ref.active(:)).'
    s=(t-ref.Ta(leg))/(ref.Tb(leg)-ref.Ta(leg));
    idx=3*(leg-1)+(1:3);
    tc.verifyEqual(polyval_bz(ref.coefficients(:,:,leg),s),Xd(18+idx,1),'AbsTol',1e-12);
   end
  end
  function onPlanSwingFootMovesWithThePlanVelocity(tc)
   [p,Xt,state,t]=localSwingState();
   [~,Xd,Ud,Xt,~,ref]=fcn_FSM(t+p.Tmpc*(0:p.predHorizon-1),Xt,p,state);
   p.swing_reference=ref; tau=t+.004; leg=1; idx=1:3;
   span=ref.Tb(leg)-ref.Ta(leg); s=(tau-ref.Ta(leg))/span; co=ref.coefficients(:,:,leg);
   Xt(idx+18)=polyval_bz(co,s);
   dx=dynamics_SRB(tau,Xt,Ud(:,1),Xd,zeros(3,1),p);
   h=1e-6; numeric=(polyval_bz(co,s+h)-polyval_bz(co,s-h))/(2*h*span);
   tc.verifyEqual(dx(idx+18),numeric,'AbsTol',1e-6);
   % Stance legs keep the historical servo exactly.
   legacy=rmfield(p,'swing_reference');
   dxLegacy=dynamics_SRB(tau,Xt,Ud(:,1),Xd,zeros(3,1),legacy);
   stance=18+(4:9);
   tc.verifyEqual(dx(stance),dxLegacy(stance),'AbsTol',0);
   tc.verifyEqual(dx(1:18),dxLegacy(1:18),'AbsTol',0);
  end
  function finishedSwingHoldsTheTouchdownTarget(tc)
   [p,Xt,state,t]=localSwingState();
   [~,Xd,Ud,Xt,~,ref]=fcn_FSM(t+p.Tmpc*(0:p.predHorizon-1),Xt,p,state);
   p.swing_reference=ref; leg=1; idx=1:3; co=ref.coefficients(:,:,leg);
   Xt(idx+18)=co(:,end);
   dx=dynamics_SRB(ref.Tb(leg)+.002,Xt,Ud(:,1),Xd,zeros(3,1),p);
   tc.verifyEqual(dx(idx+18),zeros(3,1),'AbsTol',1e-12);
  end
  function onPlanFootFollowsThePlanThroughTheInterval(tc)
   % Starting on the plan, the tracked foot stays on it and its speed never
   % exceeds the plan speed: no held-target sawtooth peak inside the interval.
   [p,Xt,state,t]=localSwingState();
   [~,Xd,Ud,Xt,~,ref]=fcn_FSM(t+p.Tmpc*(0:p.predHorizon-1),Xt,p,state);
   dt=p.simTimeStep; p.swing_reference=ref; leg=1; idx=19:21;
   co=ref.coefficients(:,:,leg); span=ref.Tb(leg)-ref.Ta(leg);
   s=@(tau)(tau-ref.Ta(leg))/span;
   Xt(idx)=polyval_bz(co,s(t));
   f=@(tau,x)dynamics_SRB(tau,x,Ud(:,1),Xd,zeros(3,1),p);
   sol=ode45(f,[t,t+dt],Xt,odeset('RelTol',1e-9,'AbsTol',1e-12));
   grid=linspace(t,t+dt,201); X=deval(sol,grid); speed=0;
   for k=1:numel(grid)
    d=f(grid(k),X(:,k)); speed=max(speed,norm(d(idx)));
   end
   sGrid=linspace(s(t),s(t+dt),201);
   planSpeed=max(vecnorm(polyval_bz(5*diff(co,1,2),sGrid)/span));
   tc.verifyEqual(X(idx,end),polyval_bz(co,s(t+dt)),'AbsTol',1e-8);
   tc.verifyLessThanOrEqual(speed,planSpeed*(1+1e-6));
  end
 end
end
function [p,Xt,state,t]=localSwingState()
p=get_params(0); p.vel_d=[.5;0]; p.acc_d=.1; p.yaw_d=0; p.p_ext=zeros(3,1);
Xt=[0;0;p.z0;.5;0;0;reshape(eye(3),9,1);zeros(3,1);reshape(p.pf34,12,1)];
t=.05;
state=struct('FSM',[2;1;1;2],'Ta',[0;0;0;0],'Tb',[p.Tsw;p.Tst;p.Tst;p.Tsw], ...
    'pf_R_trans',Xt(19:30));
end
