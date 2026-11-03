function version = estimatorVersion(b)
%ESTIMATORVERSION Missing configuration preserves historical streaming EKF.
version='fixed_pack_streaming_3rc_ekf_v1';
if isfield(b,'streaming_estimator_version')
    version=b.streaming_estimator_version;
end
assert((ischar(version) && isrow(version)) || ...
    (isstring(version) && isscalar(version) && ~ismissing(version)), ...
    'robotenergy:EstimatorVersion','Estimator version must be a scalar name.');
assert(any(strcmp(version,{'fixed_pack_streaming_3rc_ekf_v1','fixed_pack_exact_jacobian_ekf_v2'})), ...
    'robotenergy:EstimatorVersion','Unknown fixed-pack estimator version.');
version=char(version);
end
