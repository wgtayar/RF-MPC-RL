function out = run_proxy_sim_once(v_cmd, a_cmd, SimTimeDuration, kneeCsv, betaMc, tauHip4_joint, Nhip, eta, Kt)
% run_proxy_sim_once
% One RF-MPC sim (gait=0 trot) + knee proxy + constant hip proxy.
%
% Outputs:
%   Ieq_knee, Ah_knee, Ieq_total, Ah_total
%   mean timings (Tstance_mean, Tswing_mean, Tcycle_mean)
%   mean torques (tauMean_knee, tauMean_total)
%   tracking/control sums

% IMPORTANT: reset fcn_FSM persistent state between runs
clear fcn_FSM

gait = 0;                 % trot
p = get_params(gait);

% Keep your usual settings
use_qpSWIFT = 0;
p.playSpeed = 1;
p.flag_movie = 0;         % disable animation for sweep speed

% Commands
p.vel_d = [v_cmd; 0];
p.acc_d = a_cmd;
p.yaw_d = 0;

dt_sim = p.simTimeStep;
MAX_ITER = floor(SimTimeDuration/dt_sim);

qp_options = optimoptions('quadprog','Display','off');

% --- init plant state ---
% fresh start (no SimSnapshot for sweep)
[Xt,Ut] = fcn_gen_XdUd(0,[],[1;1;1;1],p);
tstart = 0;
tend   = dt_sim;

tracking_error = [];
control_effort = [];

% logging (only what we need)
[tout,Xout,Uout,Xdout,Udout,Uext] = deal([]);

% Gait timing logs (from leg 1)
leg_idx = 1;
prevFSM_leg = nan;
t_stance_start = nan;
t_swing_start  = nan;

cycles = struct();
cycles.Tstance = [];
cycles.Tswing  = [];
cycles.Tcycle  = [];

% --- knee template + parameters ---
kneeTpl = load_knee_template(kneeCsv, betaMc);

Dmc = readmatrix(kneeCsv);
Tmc = Dmc(end,1) - Dmc(1,1);

kneeParams = struct();
kneeParams.Tmc = Tmc;
kneeParams.alpha = 1;
kneeParams.beta = 2;
kneeParams.Kt = Kt;
kneeParams.eta = eta;
kneeParams.Nknee = 6/1.55;

kneeState = init_knee_proxy_state();

kneeLog = struct();
kneeLog.t = nan(MAX_ITER,1);
kneeLog.tau4 = nan(MAX_ITER,1);
kneeLog.I4 = nan(MAX_ITER,1);

% --- hip proxy (constant) ---
Ihip4 = joint_torque_to_current(tauHip4_joint, Nhip, eta, Kt);
tauHip4 = tauHip4_joint;

% --- main loop ---
for ii = 1:MAX_ITER
    t_abs0 = dt_sim*(ii-1);
    t_ = t_abs0 + p.Tmpc * (0:p.predHorizon-1);

    [FSM,Xd,Ud,Xt] = fcn_FSM(t_,Xt,p);

    % knee proxy (using leg 1)
    fsm_leg = FSM(leg_idx);
    [kneeState, kneeOut] = knee_proxy_step(t_abs0, fsm_leg, kneeState, kneeTpl, kneeParams);

    kneeLog.t(ii)    = kneeOut.t;
    kneeLog.tau4(ii) = kneeOut.tau4;
    kneeLog.I4(ii)   = kneeOut.I4;

    % timing extraction (stance->swing->stance)
    if isnan(prevFSM_leg)
        prevFSM_leg = fsm_leg;
        if fsm_leg == 1
            t_stance_start = t_abs0;
        else
            t_swing_start = t_abs0;
        end
    else
        if prevFSM_leg ~= fsm_leg
            if fsm_leg == 2
                t_swing_start = t_abs0;
            else
                t_next_stance = t_abs0;

                if isfinite(t_stance_start) && isfinite(t_swing_start)
                    Tstance_k = t_swing_start - t_stance_start;
                    Tswing_k  = t_next_stance - t_swing_start;
                    Tcycle_k  = Tstance_k + Tswing_k;

                    cycles.Tstance(end+1,1) = Tstance_k;
                    cycles.Tswing(end+1,1)  = Tswing_k;
                    cycles.Tcycle(end+1,1)  = Tcycle_k;
                end

                t_stance_start = t_next_stance;
            end
            prevFSM_leg = fsm_leg;
        end
    end

    % QP solve
    [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta(Xt,Ut,Xd,Ud,p);
    if ~use_qpSWIFT
        [zval,~,exitflag] = quadprog(H,g,Aineq,bineq,Aeq,beq,[],[],[],qp_options);
        if isempty(zval) || exitflag <= 0
            error("QP failed (exitflag=%d) at iter=%d, t=%.4f (v=%.3f, a=%.3f)", ...
                  exitflag, ii, t_abs0, v_cmd, a_cmd);
        end
    else
        [zval,~] = qpSWIFT(sparse(H),g,sparse(Aeq),beq,sparse(Aineq),bineq);
    
        if isempty(zval)
            error("qpSWIFT failed (empty zval) at iter=%d, t=%.4f (v=%.3f, a=%.3f)", ...
                  ii, t_abs0, v_cmd, a_cmd);
        end
    end
    Ut = Ut + zval(1:12);

    [u_ext,p_ext] = fcn_get_disturbance(tstart,p);
    p.p_ext = p_ext;
    u_ext = 0*u_ext;

    % simulate one step
    [t,X] = ode45(@(t,X)dynamics_SRB(t,X,Ut,Xd,0*u_ext,p),[tstart,tend],Xt);
    Xt = X(end,:)';

    tstart = tend;
    tend = tstart + dt_sim;

    tracking_error = [tracking_error; sum((Xt - Xd(:,1)).^2)];
    control_effort = [control_effort; sum(Ut)];

    lent = length(t(2:end));
    tout = [tout; t(2:end)];
    Xout = [Xout; X(2:end,:)];
    Uout = [Uout; repmat(Ut',[lent,1])];
    Xdout = [Xdout; repmat(Xd(:,1)',[lent,1])];
    Udout = [Udout; repmat(Ud(:,1)',[lent,1])];
    Uext = [Uext;  repmat(u_ext',[lent,1])];
end

% --- postprocess knee time series ---
mask = isfinite(kneeLog.t) & isfinite(kneeLog.tau4) & isfinite(kneeLog.I4);
t_k = kneeLog.t(mask);
tau4_k = kneeLog.tau4(mask);
I4_k = kneeLog.I4(mask);

if numel(t_k) >= 2
    [t_k, idx] = sort(t_k);
    tau4_k = tau4_k(idx);
    I4_k = I4_k(idx);

    Tsim_eff = t_k(end) - t_k(1);

    % knees only
    tauMean_knee = trapz(t_k, tau4_k) / Tsim_eff;
    Qknee = trapz(t_k, abs(I4_k));
    Ieq_knee = Qknee / Tsim_eff;
    Ah_knee  = Qknee / 3600;

    % knees + hips
    tau4_total = tau4_k + tauHip4;
    I4_total = I4_k + Ihip4;

    tauMean_total = trapz(t_k, tau4_total) / Tsim_eff;
    Qtotal = Qknee + abs(Ihip4) * Tsim_eff;
    Ieq_total = Qtotal / Tsim_eff;
    Ah_total = Qtotal / 3600;

    % Peak metrics from total current
    Iabs = abs(I4_total);

    % Ignore initial transient window
    t_ignore = 0.5;
    keep = (t_k >= (t_k(1) + t_ignore));
    t_pk = t_k(keep);
    I_pk = Iabs(keep);

    if numel(t_pk) >= 5
        % Peak detection
        minDist = 0.04;

        [pks, locs] = findpeaks(I_pk, t_pk, 'MinPeakDistance', minDist);

        out_pk_count = numel(pks);
        out_pk_freq  = out_pk_count / (t_pk(end) - t_pk(1));

        if out_pk_count >= 1
            out_pk_mean = mean(pks);
            out_pk_max = max(pks);
            out_pk_p95 = prctile(pks,95);
        else
            out_pk_mean = NaN;
            out_pk_max = NaN;
            out_pk_p95 = NaN;
        end
    else
        out_pk_count = NaN;
        out_pk_freq = NaN;
        out_pk_mean = NaN;
        out_pk_max = NaN;
        out_pk_p95 = NaN;
    end
else
    tauMean_knee = NaN; tauMean_total = NaN;
    Ieq_knee = NaN; Ah_knee = NaN;
    Ieq_total = NaN; Ah_total = NaN;
    out_pk_count = NaN;
    out_pk_freq = NaN;
    out_pk_mean = NaN;
    out_pk_max = NaN;
    out_pk_p95 = NaN;
end

% timings
if ~isempty(cycles.Tcycle)
    Tstance_mean = mean(cycles.Tstance);
    Tswing_mean = mean(cycles.Tswing);
    Tcycle_mean = mean(cycles.Tcycle);
else
    Tstance_mean = NaN;
    Tswing_mean = NaN;
    Tcycle_mean = NaN;
end

if ~isnan(Tcycle_mean) && Tcycle_mean > 0
    f_cycle = 1 / Tcycle_mean;
else
    f_cycle = NaN;
end

% pack output
out = struct();
out.v_cmd = v_cmd;
out.a_cmd = a_cmd;

out.Ieq_knee = Ieq_knee;
out.Ah_knee = Ah_knee;
out.Ieq_total = Ieq_total;
out.Ah_total = Ah_total;

out.tauMean_knee = tauMean_knee;
out.tauMean_total = tauMean_total;

out.Tstance_mean = Tstance_mean;
out.Tswing_mean = Tswing_mean;
out.Tcycle_mean = Tcycle_mean;

out.tracking_error_sum = sum(tracking_error);
out.control_effort_sum = sum(control_effort);

out.Ipk_mean = out_pk_mean;
out.Ipk_max  = out_pk_max;
out.Ipk_p95  = out_pk_p95;

out.fpk = out_pk_freq; % peak frequency [Hz]
out.Npk = out_pk_count; % number of peaks in window
out.f_cycle = f_cycle; % gait cycle frequency estimate [Hz]

% keep hip constants in output (traceability)
out.hip = struct('tauHip4',tauHip4,'Ihip4',Ihip4,'Nhip',Nhip,'eta',eta,'Kt',Kt);

end