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
rootDir = bootstrap_RF_MPC_RL(); %#ok<NASGU>

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
    % quarter_control_effort = Sim.quarter_control_effort;
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

qp_options = optimoptions('quadprog', ...
    'Display', 'off');

cfgBundle=setup_RL_MPC(); cfg=cfgBundle.cfg;
tpl=load_knee_template(cfg.PROXY.kneeCsv,cfg.PROXY.betaMc);
data=readmatrix(cfg.PROXY.kneeCsv); Tmc=data(end,1)-data(1,1);
if ~isfield(Sim,'energy')
    assert(Sim.t==0,'robotenergy:Snapshot','Use a fresh snapshot for this energy model.');
    Sim.energy=robotenergy.start(cfg.BATTERY); Sim.battery=struct();
end
energyRows=zeros(MAX_ITER,4);
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
        [FSM,Xd,Ud,Xt,fsm] = fcn_FSM_bound(t_,Xt,p);
    else
        [FSM,Xd,Ud,Xt,fsm] = fcn_FSM(t_,Xt,p);
    end

    % QP
    [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta(Xt,Ut,Xd,Ud,p);
    if ~use_qpSWIFT
        zval = quadprog(H,g,Aineq,bineq,Aeq,beq,[],[],[],qp_options);
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

    m=robotenergy.loadStep(X(1,:)',Xt,Ut,p,dt_sim,tstart,fsm,tpl,Tmc,cfg.ENERGY);
    [Sim.energy,Sim.battery,y]=robotenergy.sample(Sim.energy,m,dt_sim,cfg.ENERGY,Sim.battery);
    assert(~Sim.energy.voltageLimitSteps && ~Sim.energy.torqueLimitSteps,'robotenergy:ActuatorLimit','Motion exceeds the drive envelope.');
    energyRows(ii,:)=[tend y.packA y.packV y.watts];
    % update rolling time window
    tstart = tend;
    tend = tstart + dt_sim;

    % metrics
    tracking_error = [tracking_error; sum((Xt - Xd(:,1)).^2)];
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

fprintf('Pack %.3f Wh, estimated SOC %.3f percent\n',Sim.energy.energyWh,Sim.battery.soc_pct);
%% === Save for next simulation ===
Sim.t  = Sim.t + MAX_ITER*dt_sim;   % advance absolute time
Sim.Xt = Xt;    % persist final plant state
Sim.Ut = Ut;    % persist last control
Sim.control_effort = control_effort;
Sim.tracking_error = tracking_error;

Sim.last_energy_trace=array2table(energyRows,'VariableNames',{'Time','PackCurrent','PackVoltage','PowerW'});

save(SnapFile,'Sim');

disp('Current:')
disp(mean(energyRows(:,2)))

disp('Control Effort:')
disp(sum(control_effort(:,1)))

disp('Tracking Error:')
disp(sum(tracking_error(:,1)))

%% Animation
[t,EA,EAd] = fig_animate(tout,Xout,Uout,Xdout,Udout,Uext,p);
