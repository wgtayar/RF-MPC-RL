function repro_failure_from_dbg()
    addpath fcns fcns_MPC

    Sdbg = load('dbg_failure_last.mat','dbg_failure');
    df   = Sdbg.dbg_failure;

    gait = 0;
    p = get_params(gait);

    p.R = diag(df.R_weights);

    Xt = df.Xt;
    Ut = df.Ut;
    Xd = df.Xd;
    Ud = df.Ud;

    fprintf('Repro exact: call_idx=%d, ii=%d, t0_abs=%.3f, ||Xt||=%.3f, ||Ut||=%.3f\n', ...
        df.call_idx, df.ii, df.t0_abs, norm(Xt), norm(Ut));

    H = df.H; g = df.g;
    Aineq = df.Aineq; bineq = df.bineq;
    Aeq = df.Aeq; beq = df.beq;

    Hsym = (H + H')/2;
    lam = eig(Hsym);
    minEigH = min(lam);
    maxEigH = max(lam);
    condH = maxEigH / max(minEigH, 1e-12);

    fprintf('minEigH = %.3e, maxEigH = %.3e, condH = %.3e\n', ...
        minEigH, maxEigH, condH);

    qp_options = optimoptions('quadprog', ...
        'Display','iter', ...
        'Algorithm','interior-point-convex');

    [zval,fval,exitflag,output] = quadprog(Hsym,g,Aineq,bineq,Aeq,beq,[],[],[],qp_options);

    fprintf('exitflag=%d\n', exitflag);
    disp(output.message);
end
