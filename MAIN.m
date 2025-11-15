% Representation-Free Model Predictive Control for Dynamic Quadruped Panther
% Author: Yanran Ding
% Last modified: 2020/12/21
% 
% Code accompanying the paper:
% Yanran Ding, Abhishek Pandala, Chuanzheng Li, Young-Ha Shin, Hae-Won Park
% "Representation-Free Model Predictive Control for Dynamic Motions in Quadrupeds"
% Transactions on Robotics
% 
% preprint available at: https://arxiv.org/abs/2012.10002
% video available at: https://www.youtube.com/watch?v=iMacEwQisoQ&t=101s

%% initialization
% clear all;close all;clc
% addpath fcns fcns_MPC
% 
% %% --- parameters ---
% % ---- gait ----
% % 0-trot; 1-bound; 2-pacing 3-gallop; 4-trot run; 5-crawl
% gait = 0;
% p = get_params(gait);
% p.playSpeed = 10;
% p.flag_movie = 1;       % 1 - make movie
% use_qpSWIFT = 0;        % 0 - quadprog, 1 - qpSWIFT (external)
% 
% dt_sim = p.simTimeStep;
% SimTimeDuration = 1;  % [sec]
% MAX_ITER = floor(SimTimeDuration/p.simTimeStep);
% 
% % desired trajectory
% p.acc_d = 1;
% p.vel_d = [0.5;0];
% p.yaw_d = 0;
% 
% %% Model Predictive Control
% % --- initial condition ---
% % Xt = [pc dpc vR wb pf]': [30,1]
% if gait == 1
%     [p,Xt,Ut] = fcn_bound_ref_traj(p);
% else
%     [Xt,Ut] = fcn_gen_XdUd(0,[],[1;1;1;1],p);
% end
% 
% tracking_error = [];
% control_effort = [];
% 
% % --- logging ---
% tstart = 0;
% tend = dt_sim;
% 
% [tout,Xout,Uout,Xdout,Udout,Uext,FSMout] = deal([]);
% % --- simulation ----
% h_waitbar = waitbar(0,'Calculating...');
% tic
% for ii = 1:MAX_ITER
%     % --- time vector ---
%     t_ = dt_sim * (ii-1) + p.Tmpc * (0:p.predHorizon-1);
% 
%     % --- FSM ---
%     if gait == 1
%         [FSM,Xd,Ud,Xt] = fcn_FSM_bound(t_,Xt,p);
%     else
%         [FSM,Xd,Ud,Xt] = fcn_FSM(t_,Xt,p);
%     end
% 
%     % --- MPC ----
%     % form QP
%     [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta(Xt,Ut,Xd,Ud,p);
% 
%     if ~use_qpSWIFT
%         % solve QP using quadprog
%         [zval] = quadprog(H,g,Aineq,bineq,Aeq,beq,[],[]);
%     else
%         % interface with the QP solver qpSWIFT
%         [zval,basic_info] = qpSWIFT(sparse(H),g,sparse(Aeq),beq,sparse(Aineq),bineq);
%     end
% 
%     Ut = Ut + zval(1:12);
% 
%     % --- external disturbance ---
%     [u_ext,p_ext] = fcn_get_disturbance(tstart,p);
%     p.p_ext = p_ext;        % position of external force
%     u_ext = 0*u_ext;
% 
%     % --- simulate ---
%     [t,X] = ode45(@(t,X)dynamics_SRB(t,X,Ut,Xd,0*u_ext,p),[tstart,tend],Xt);
%     fprintf("ODE output X size: %s\n", mat2str(size(X)));
% 
%     % --- update ---
%     Xt = X(end,:)';
%     tstart = tend;
%     tend = tstart + dt_sim;
% 
%     tracking_error = [tracking_error; sum((Xt - Xd(:,1)).^2)];
%     control_effort = [control_effort; sum(Ut.^2)];
% 
%     disp('Xd:')
%     disp(size(Xd))
%     disp('Xt')
%     disp(size(Xt))
% 
%     % --- log ---  
%     lent = length(t(2:end));
%     tout = [tout;t(2:end)];
%     Xout = [Xout;X(2:end,:)];
%     Uout = [Uout;repmat(Ut',[lent,1])];
%     Xdout = [Xdout;repmat(Xd(:,1)',[lent,1])];
%     Udout = [Udout;repmat(Ud(:,1)',[lent,1])];
%     Uext = [Uext;repmat(u_ext',[lent,1])];
%     FSMout = [FSMout;repmat(FSM',[lent,1])];
% 
%     waitbar(ii/MAX_ITER,h_waitbar,'Calculating...');
% end
% close(h_waitbar)
% fprintf('Calculation Complete!\n')
% toc
% 
% %% Animation
% [t,EA,EAd] = fig_animate(tout,Xout,Uout,Xdout,Udout,Uext,p);

%% initialization
addpath fcns fcns_MPC

%% --- parameters ---
% ---- gait ----
% 0-trot; 1-bound; 2-pacing 3-gallop; 4-trot run; 5-crawl
gait = 0;
p = get_params(gait);
p.playSpeed = 10;
p.flag_movie = 1;
use_qpSWIFT = 0;

dt_sim = p.simTimeStep;
SimTimeDuration = 1;
MAX_ITER = floor(SimTimeDuration/dt_sim);

% desired trajectory
p.acc_d = 1;
p.vel_d = [0.5;0];
p.yaw_d = 0;

% m file to save the final simulation time and state X
SnapFile = 'SimSnapshot.mat';
if exist(SnapFile,'file')
    S = load(SnapFile);
    Sim = S.Sim;   % resume
    tstart = Sim.t;
    tend = tstart + dt_sim;
    Xt = Sim.Xt;
    Ut = Sim.Ut;
    quarter_control_effort = Sim.quarter_control_effort;
    control_effort = Sim.control_effort;
    tracking_error = Sim.tracking_error;

    % optional: sanitize tiny drift in base orientation if stored as rotation matrix
    if isfield(Sim,'R_base')
        [U,~,V] = svd(Sim.R_base); Sim.R_base = U*V';
    end
else
    % fresh start
    if gait == 1
        [p,Xt,Ut] = fcn_bound_ref_traj(p);
    else
        % use absolute start time 0 for the very first chunk
        [Xt,Ut] = fcn_gen_XdUd(0,[],[1;1;1;1],p);
    end
    Sim = struct();
    Sim.t  = 0;              % absolute simulation time at start of this chunk
    Sim.Xt = Xt;             % full plant state at chunk boundary
    Sim.Ut = Ut;             % last control at chunk boundary
    tstart = 0;
    tend = dt_sim;
    Sim.quarter_control_effort = [];
    Sim.control_effort = [];
    Sim.tracking_error = [];
    tracking_error = [];
    control_effort = [];
end

if isfield(Sim,'t_power')
    t_power = Sim.t_power;
else
    t_power = [];
end
if isfield(Sim,'Pc_all')
    Pc_all = Sim.Pc_all;
else
    Pc_all = [];
end

% tracking_error = [];
% control_effort = [];

% logging
[tout,Xout,Uout,Xdout,Udout,Uext,FSMout] = deal([]);

h_waitbar = waitbar(0,'Calculating...');
tic
for ii = 1:MAX_ITER
    % absolute time vector over the MPC horizon
    t_abs0 = Sim.t + dt_sim*(ii-1);
    t_ = t_abs0 + p.Tmpc * (0:p.predHorizon-1);

    % FSM and references from absolute time
    if gait == 1
        [FSM,Xd,Ud,Xt] = fcn_FSM_bound(t_,Xt,p);
    else
        [FSM,Xd,Ud,Xt] = fcn_FSM(t_,Xt,p);
    end

    % QP
    [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta(Xt,Ut,Xd,Ud,p);
    if ~use_qpSWIFT
        zval = quadprog(H,g,Aineq,bineq,Aeq,beq,[],[]);
    else
        [zval,~] = qpSWIFT(sparse(H),g,sparse(Aeq),beq,sparse(Aineq),bineq);
    end
    Ut = Ut + zval(1:12);

    % external disturbance
    [u_ext,p_ext] = fcn_get_disturbance(tstart,p);
    p.p_ext = p_ext;
    u_ext = 0*u_ext;

    % tripwires before integrating
    % assert(isreal(Xt) && all(isfinite(Xt)), 'Xt nonreal or NaN before ode');
    % assert(isreal(Ut) && all(isfinite(Ut)), 'Ut nonreal or NaN before ode');

    % simulate one step
    [t,X] = ode45(@(t,X)dynamics_SRB(t,X,Ut,Xd,0*u_ext,p),[tstart,tend],Xt);
    Xt = X(end,:)';

    % update rolling time window
    tstart = tend;
    tend = tstart + dt_sim;

    % metrics
    % tracking_error = [tracking_error; sum((Xt - Xd(:,1)).^2)];
    % quarter_control_effort = [quarter_control_effort; sum(Ut())]
    control_effort = [control_effort; sum(Ut)];

    % log
    lent = length(t(2:end));
    tout = [tout; t(2:end)];
    Xout = [Xout; X(2:end,:)];
    Uout = [Uout; repmat(Ut',[lent,1])];
    Xdout = [Xdout; repmat(Xd(:,1)',[lent,1])];
    Udout = [Udout; repmat(Ud(:,1)',[lent,1])];
    Uext  = [Uext;  repmat(u_ext',[lent,1])];
    FSMout= [FSMout;repmat(FSM',[lent,1])];

    waitbar(ii/MAX_ITER,h_waitbar,'Calculating...');
end
close(h_waitbar)
fprintf('Calculation Complete!\n'); toc

[tpc_chunk, Pc_chunk] = power_calc(tout, Xout, Uout, struct('BI', p.J));  % uses ALL samples of this chunk
t_power = [t_power; tpc_chunk];
Pc_all  = [Pc_all;  Pc_chunk];

% === Predict DC bus current from power trace and plot ===
Iopts = struct('I0', 1.0, 'eta', 1.4, 'clipNeg', false, 'smoothWin', 0); % adjust if you like
stats_pred = Ibus_pred(t_power, Pc_all, 20, Iopts);

figure('Name','Predicted Ibus'); 
plot(stats_pred.t, stats_pred.Ipred, 'LineWidth', 1.25); grid on
xlabel('time [s]'); ylabel('I_{bus} predicted [A]');
title(sprintf('I_{bus} prediction, I_0=%.2f A, \\eta=%.2f, V=%.1f V', ...
      stats_pred.I0, stats_pred.eta, stats_pred.Vbus));


%% === Save for next simulation ===
Sim.t  = Sim.t + MAX_ITER*dt_sim;   % advance absolute time
Sim.Xt = Xt;    % persist final plant state
Sim.Ut = Ut;    % persist last control
Sim.control_effort = control_effort;

Sim.t_power = t_power;
Sim.Pc_all  = Pc_all;

save(SnapFile,'Sim');

disp('Current:')
disp(sum(stats_pred.Ipred(:,1))/4120)

disp('Control Effort:')
disp(sum(control_effort(:,1)))

%% Animation
[t,EA,EAd] = fig_animate(tout,Xout,Uout,Xdout,Udout,Uext,p);
