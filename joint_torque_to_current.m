function I = joint_torque_to_current(tauJointAbs, N, eta, Kt)
I = abs(tauJointAbs) / (eta * N * Kt);
end