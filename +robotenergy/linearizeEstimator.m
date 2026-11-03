function [predicted,voltage,F,H] = linearizeEstimator(s,z,i,dt)
%LINEARIZEESTIMATOR Exact local Jacobians of the frozen-parameter cell step.
% Current is the measured input, held fixed when differentiating. The plant
% freezes OCV/R/C at interval-start SOC, updates RC branches, then books SOC.
% The end-state voltage observation therefore reconstructs start SOC as
% predicted(1)+i*dt/Q. No plant ground-truth SOC enters this calculation.
% Predict SOC without clipping; project the posterior after the update. This
% preserves the interval-start reconstruction even at an estimator boundary.
[r,~,dr]=robotenergy.lookup(s,z(1));
res=r(2:4); cap=r(5:7); dres=dr(2:4); dcap=dr(5:7);
tau=res.*cap; a=exp(-dt./tau);
da=a.*dt.*(dres.*cap+res.*dcap)./(tau.^2);
dq=i*dt/(3600*s.capacity);
predicted=[z(1)-dq; a.*z(2:4)+(1-a).*res*i];
F=diag([1;a]);
F(2:4,1)=da.*(z(2:4)-res*i)+(1-a).*dres*i;
[r,o,dr,do]=robotenergy.lookup(s,predicted(1)+dq);
voltage=o-r(1)*i-sum(predicted(2:4));
H=[do-dr(1)*i,-1,-1,-1];
end
