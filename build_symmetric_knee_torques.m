function build_symmetric_knee_torques(inputXlsx, outputCsv)
D = readmatrix(inputXlsx);
t = D(:,1);
tau = D(:,2);

[t, idx] = sort(t);
tau = tau(idx);

[t, ~, g] = unique(t);
tau = accumarray(g, tau, [], @mean);

t0 = t(1);
t = t - t0;

Tmc = t(end) - t(1);
phi = t / Tmc;

phiGrid = linspace(0, 1, 400).';
tauGrid = interp1(phi, tau, phiGrid, "pchip", "extrap");

phiShift = mod(phiGrid + 0.5, 1.0);
tauShift = interp1(phiGrid, tauGrid, phiShift, "pchip", "extrap");

tauPairA = 2 * tauGrid;
tauPairB = 2 * tauShift;
tau4 = tauPairA + tauPairB;

tGrid = phiGrid * Tmc;

out = [tGrid, phiGrid, tauGrid, tauPairA, tauPairB, tau4];
writematrix(out, outputCsv);
end
