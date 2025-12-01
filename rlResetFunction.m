
% ========== Iteration of the reset function where R 
% ========== always starts from p.R
% function [initialObs, logged] = rlResetFunction()
%     gait = 0;
%     p = get_params(gait);
%     initial_R = [p.R(1,1); p.R(2,2); p.R(3,3)];
%     upper_abs = 5.00 * initial_R;
% 
%     snap_path = 'SimSnapshot_RL.mat';
%     if exist(snap_path, 'file')
%         delete(snap_path);
%     end
% 
%     last_R = initial_R;
%     save('LastR.mat','last_R');
% 
%     nt = 0; nu = 0; i_consumed = 0;
%     initialObs = [nt; nu; i_consumed; last_R./upper_abs];
% 
%     logged.prevCost = [];
%     logged.last_R = last_R;
% end

% ========== Iteration of the reset function where R
% ========== always starts from the last R

function [initialObs, logged] = rlResetFunction()
    gait = 0;
    p = get_params(gait);

    % Nominal R used only as a reference / for first-ever initialization
    initial_R = [p.R(1,1); p.R(2,2); p.R(3,3)];
    upper_abs = 3.00 * initial_R;

    % --- Reset the simulation snapshot (start dynamics from scratch) ---
    snap_path = 'SimSnapshot_RL.mat';
    if exist(snap_path, 'file')
        delete(snap_path);
    end

    clear fcn_FSM fcn_FSM_bound fcn_gen_XdUd ...
          fcn_get_QP_form_eta dynamics_SRB power_calc ...
          Ibus_pred estimateSOC

    % --- Keep / initialize controller weights R across episodes ---
    if exist('LastR.mat','file')
        % Continue from last episode's R
        S = load('LastR.mat','last_R');
        last_R = S.last_R;
    else
        % First ever call: seed from nominal p.R
        last_R = initial_R;
        save('LastR.mat','last_R');
    end

    % Initial observation at the start of the episode
    nt = 0; 
    nu = 0; 
    SOC_current = 1;

    initialObs = [nt; nu; SOC_current; last_R./upper_abs];

    logged.prevCost = [];
    logged.last_R = last_R;

    if exist('RL_meta.mat','file')
        try
            M = load('RL_meta.mat','ep_idx');
            ep_idx = M.ep_idx + 1;
        catch
            ep_idx = 1;
        end
    else
        ep_idx = 1;
    end
    save('RL_meta.mat','ep_idx');
end
