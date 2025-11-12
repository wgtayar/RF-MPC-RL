function [initialObs, logged] = rlResetFunction()
    gait = 0;
    p = get_params(gait);
    initial_R = [p.R(1,1); p.R(2,2); p.R(3,3)];
    upper_abs = 5.00 * initial_R;

    snap_path = 'SimSnapshot_RL.mat';
    if exist(snap_path, 'file')
        delete(snap_path);
    end
    
    last_R = initial_R;
    save('LastR.mat','last_R');

    nt = 0; nu = 0; i_consumed = 0;
    initialObs = [nt; nu; i_consumed; last_R./upper_abs];

    logged.prevCost = [];
    logged.last_R = last_R;
end
