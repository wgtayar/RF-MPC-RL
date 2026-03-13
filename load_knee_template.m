function tpl = load_knee_template(csvFile, betaMc)
D = readmatrix(csvFile);
phi = D(:,2);
tauSingle = D(:,3);

tpl.betaMc = betaMc;
tpl.tauSingle = @(ph) interp1(phi, tauSingle, ph, "pchip", "extrap");

phiSt = phi(phi <= betaMc) / betaMc;
tauSt = tauSingle(phi <= betaMc);

phiSw = (phi(phi >= betaMc) - betaMc) / (1 - betaMc);
tauSw = tauSingle(phi >= betaMc);

tpl.tauSt = @(x) interp1(phiSt, tauSt, min(max(x,0),1), "pchip", "extrap");
tpl.tauSw = @(x) interp1(phiSw, tauSw, min(max(x,0),1), "pchip", "extrap");
end
