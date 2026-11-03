function [r,o,dr,do] = lookup(s,z)
%LOOKUP Cell interpolation and its actual local derivatives with respect to SOC.
% Electrical parameters are piecewise linear; OCV is the stored PCHIP.
% At an interior knot use the right derivative, at the final knot the left.
% Outside the characterized interval the lookup is constant (derivative zero).
g=s.grid;
outside=z<g(1) || z>g(end);
z=max(g(1),min(g(end),z));
k=min(numel(g)-1,find(g<=z,1,'last'));
width=g(k+1)-g(k); u=(z-g(k))/width;
r=((1-u)*s.values(k,:)+u*s.values(k+1,:))';
dr=((s.values(k+1,:)-s.values(k,:))/width)';
o=ppval(s.ocv,z);
if isfield(s,'ocvDerivative')
    derivative=s.ocvDerivative;
else
    [breaks,coefs,~,order]=unmkpp(s.ocv);
    derivative=mkpp(breaks,coefs(:,1:end-1).*(order-1:-1:1));
end
do=ppval(derivative,z);
if outside, dr(:)=0; do=0; end
end
