function health = exact_mpc_health(Xt, Ut, Xd)
%exact_mpc_health Preserve invalid states without inventing finite health data.
    health = struct('valid',all(isfinite([Xt(:);Ut(:);Xd(:)])), ...
        'components',struct(),'per_leg_force_norms',nan(1,4), ...
        'linear_velocity_error',NaN,'Ut_norm',NaN);
    if health.valid
        health.components = decompose_srb_state(Xt,Xd);
        health.per_leg_force_norms = vecnorm(reshape(Ut,3,4),2,1);
        health.linear_velocity_error = norm(Xt(4:6)-Xd(4:6,1));
        health.Ut_norm = norm(Ut);
    end
end
