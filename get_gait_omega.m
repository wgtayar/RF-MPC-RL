function omega = get_gait_omega()
% Match fcn_FSM timing: Tst = min(Tst_, 0.2/norm(v_xy)), T = Tst + Tsw
    T_default = 1.0;
    try
        gait = 0; p = get_params(gait);
        Tst_ = p.Tst; Tsw = p.Tsw;
        if exist('SimSnapshot_RL.mat','file')
            S = load('SimSnapshot_RL.mat','Sim');
            Xt = S.Sim.Xt; vxy = norm(Xt(4:5));
        else
            vxy = 0;
        end
        Tst_eff = min(Tst_, 0.2/max(vxy, eps));
        T = Tst_eff + Tsw;
        if ~isfinite(T) || T <= 1e-6, T = T_default; end
    catch
        T = T_default;
    end
    omega = 2*pi/T;
end
