function [s,y] = advancePlant(s,P,dt)
%ADVANCEPLANT Physical constant-power interval only; no EKF update.
% This shared core retains the original plant arithmetic exactly. A declared
% evaluation mismatch (s.plant, robotenergy.plantMismatch) changes only the
% plant resistances and capacity through robotenergy.plantView.
validateattributes([P dt],{'numeric'},{'real','finite','nonnegative'});
assert(dt>0 && isscalar(P) && isscalar(dt),'robotenergy:Interval','Invalid interval.');
q=robotenergy.plantView(s);
[r,ocv]=robotenergy.lookup(q,s.soc);
a=exp(-dt./(r(2:4).*r(5:7))); b=(1-a).*r(2:4);
E=ocv-sum(a.*s.rc); R=r(1)+sum(b);
w=P/(s.ns*s.np); D=E^2-4*R*w;
assert(E>0 && D>=0,'robotenergy:PowerLimit','Demand exceeds modeled pack power.');
i=2*w/(E+sqrt(D)); v=E-R*i;
s.rc=a.*s.rc+b*i; s.soc=s.soc-i*dt/(3600*q.capacity);
assert(s.soc>=s.grid(1),'robotenergy:Empty','Cell SOC below characterized range.');
I=i*s.np; V=v*s.ns; s.elapsed=s.elapsed+dt;
s.chargeAs=s.chargeAs+I*dt; s.energyWh=s.energyWh+V*I*dt/3600;
y=struct('packA',I,'cellA',i,'packV',V,'cellV',v,'watts',P);
end
