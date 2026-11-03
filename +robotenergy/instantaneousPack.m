function y=instantaneousPack(s,P)
%INSTANTANEOUSPACK Zero-duration load solve; never books current or energy.
validateattributes(P,{'numeric'},{'scalar','real','finite','nonnegative'});
[r,ocv]=robotenergy.lookup(s,s.soc);
E=ocv-sum(s.rc); R=r(1); w=P/(s.ns*s.np); D=E^2-4*R*w;
assert(E>0 && D>=0,'robotenergy:PowerLimit','Instantaneous demand exceeds modeled pack power.');
i=2*w/(E+sqrt(D)); v=E-R*i;
y=struct('cellA',i,'packA',s.np*i,'cellV',v,'packV',s.ns*v,'watts',P);
end
