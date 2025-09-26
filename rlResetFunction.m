% function [initialObs, logged_signals] = rlResetFunction()
%     initialObs = [0; 0];
%     logged_signals = [];
% end


function [initialObs, logged] = rlResetFunction()
% Keep SimSnapshot_RL.mat. Reset only RL episode state.
    gait = 0;
    p = get_params(gait);
    initial_R = [p.R(1,1); p.R(2,2); p.R(3,3)];
    upper_abs = 5.00 * initial_R;

    if exist('LastR.mat','file')
        S = load('LastR.mat','last_R');
        last_R = S.last_R;
    else
        last_R = initial_R;
        save('LastR.mat','last_R');
    end

    t_abs = get_snapshot_time_or_zero();
    omega = get_gait_omega();

    % Start-of-episode obs (no cost yet)
    nt = 0; nu = 0;
    initialObs = [nt; nu; last_R./upper_abs; sin(omega*t_abs); cos(omega*t_abs)];

    logged.prevCost = []; % will be filled on first step after sim
    logged.last_R   = last_R;
end
