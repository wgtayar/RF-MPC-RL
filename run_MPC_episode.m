function [norm_tracking_error, norm_control_effort] = run_MPC_episode(R_weights)

    [tracking_error_total, control_effort_total] = run_MPC_simulation(R_weights);

    T_ref = 10;
    U_ref = 7.5e5;

    norm_tracking_error = tracking_error_total / T_ref;
    norm_control_effort = control_effort_total / U_ref;
end

