function value = weightedQuantile(samples,weights,probability)
%WEIGHTEDQUANTILE Time-weighted empirical inverse CDF, without interpolation.
% For equal positive durations this is x_sorted(ceil(p*N)), with p=0
% returning the minimum. Zero-weight observations carry no probability mass.
assert(isnumeric(samples) && isnumeric(weights) && isreal(samples) && ...
    isreal(weights) && isvector(samples) && isvector(weights) && ...
    numel(samples)==numel(weights) && ~isempty(samples) && ...
    all(isfinite(samples(:))) && all(isfinite(weights(:))) && ...
    all(weights(:)>=0) && sum(weights(:))>0, ...
    'robotenergy:QuantileInput','Require finite samples and nonnegative durations with positive total.');
validateattributes(probability,{'numeric'},{'scalar','real','finite','>=',0,'<=',1});
keep=weights(:)>0; samples=samples(:); weights=weights(:);
[samples,order]=sort(samples(keep)); weights=weights(keep); weights=weights(order);
if probability==0, value=samples(1); return; end
index=find(cumsum(weights)>=probability*sum(weights),1,'first');
% A summation-rounding difference at probability=1 cannot discard the maximum.
if isempty(index), index=numel(samples); end
value=samples(index);
end
