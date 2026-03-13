function [state, out] = knee_proxy_step(tAbs, fsmLeg, state, tpl, params)

out = struct();
out.t = tAbs;
out.tau4 = NaN;
out.I4 = NaN;

if isnan(state.prevFsm)
    state.prevFsm = fsmLeg;
    if fsmLeg == 1
        state.tSt0 = tAbs;
    else
        state.tSw0 = tAbs;
    end
    return
end

if state.prevFsm ~= fsmLeg
    if fsmLeg == 2
        state.tSw0 = tAbs;
        if isfinite(state.tSt0)
            state.Tst = state.tSw0 - state.tSt0;
        end
    else
        state.tSt0 = tAbs;
        if isfinite(state.tSw0)
            state.Tsw = state.tSt0 - state.tSw0;
        end
    end
    state.prevFsm = fsmLeg;
end

TstMc = tpl.betaMc * params.Tmc;
TswMc = (1 - tpl.betaMc) * params.Tmc;

if fsmLeg == 1
    xi = (tAbs - state.tSt0) / state.Tst;
    s = (TstMc / state.Tst)^params.alpha;
    tau1 = s * tpl.tauSt(xi);
else
    xi = (tAbs - state.tSw0) / state.Tsw;
    s = (TswMc / state.Tsw)^params.beta;
    tau1 = s * tpl.tauSw(xi);
end

phi = mod((tAbs - state.tSt0) / (state.Tst + state.Tsw), 1.0);
phi2 = mod(phi + 0.5, 1.0);

if phi2 <= tpl.betaMc
    xi2 = phi2 / tpl.betaMc;
    tau2 = (TstMc / state.Tst)^params.alpha * tpl.tauSt(xi2);
else
    xi2 = (phi2 - tpl.betaMc) / (1 - tpl.betaMc);
    tau2 = (TswMc / state.Tsw)^params.beta * tpl.tauSw(xi2);
end

tau4 = 2 * abs(tau1) + 2 * abs(tau2);

N = params.Nknee;
tauMotor4 = tau4 / (params.eta * N);
I4 = tauMotor4 / params.Kt;

out.tau4 = tau4;
out.I4 = I4;

end