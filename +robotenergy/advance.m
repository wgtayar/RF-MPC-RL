function [s,y] = advance(s,P,dt)
% Constant terminal watts during one interval. Physical (high-voltage) root.
validateattributes([P dt],{'numeric'},{'real','finite','nonnegative'});
assert(dt>0 && isscalar(P) && isscalar(dt),'robotenergy:Interval','Invalid interval.');
[r,ocv]=robotenergy.lookup(s,s.soc);
a=exp(-dt./(r(2:4).*r(5:7))); b=(1-a).*r(2:4);
E=ocv-sum(a.*s.rc); R=r(1)+sum(b);
w=P/(s.ns*s.np); D=E^2-4*R*w;
assert(E>0 && D>=0,'robotenergy:PowerLimit','Demand exceeds modeled pack power.');
i=2*w/(E+sqrt(D)); v=E-R*i;
s.rc=a.*s.rc+b*i; s.soc=s.soc-i*dt/(3600*s.capacity);
assert(s.soc>=s.grid(1),'robotenergy:Empty','Cell SOC below characterized range.');
I=i*s.np; V=v*s.ns; s.elapsed=s.elapsed+dt;
s.chargeAs=s.chargeAs+I*dt; s.energyWh=s.energyWh+V*I*dt/3600;
% Newly initialized states use exact derivatives of this discrete cell model.
% Unversioned captured states retain the original approximate estimator so
% replaying historical evidence does not silently change its state transition.
version='fixed_pack_streaming_3rc_ekf_v1';
if isfield(s,'estimatorVersion')
    version=robotenergy.estimatorVersion(struct('streaming_estimator_version',s.estimatorVersion));
end
if strcmp(version,'fixed_pack_exact_jacobian_ekf_v2')
    [z,predictedV,F,H]=robotenergy.linearizeEstimator(s,s.estimate,i,dt);
else
    z=s.estimate; z(1)=max(s.grid(1),min(s.grid(end),z(1)-i*dt/(3600*s.capacity)));
    [r,o]=robotenergy.lookup(s,z(1)); a=exp(-dt./(r(2:4).*r(5:7)));
    z(2:4)=a.*z(2:4)+(1-a).*r(2:4)*i;
    k=min(numel(s.grid)-1,find(s.grid<=z(1),1,'last'));
    u=(z(1)-s.grid(k))/(s.grid(k+1)-s.grid(k));
    h=(1-u)*s.slope(k)+u*s.slope(k+1);
    F=diag([1;a]); H=[h -1 -1 -1];
    predictedV=o-r(1)*i-sum(z(2:4));
end
C=F*s.cov*F'+s.Q;
K=C*H'/(H*C*H'+s.Rv); z=z+K*(v-predictedV);
z(1)=max(s.grid(1),min(s.grid(end),z(1))); s.estimate=z;
M=eye(4)-K*H; s.cov=M*C*M'+K*s.Rv*K';
y=struct('packA',I,'cellA',i,'packV',V,'cellV',v,'watts',P,'socPct',100*z(1));
end
