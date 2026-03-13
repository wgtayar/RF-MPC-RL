function [n_series, n_parallel] = batterySizing(Time, Current, Voltage, DoD)
cell_V = 3.7;
cell_Ah = 2;

C_req = trapz(Time, Current) / 3600;
C_req = C_req / DoD;

Energy = C_req * Voltage;

n_series = ceil(Voltage / cell_V);
n_parallel = ceil(C_req / cell_Ah);

fprintf('Required capacity: %.2f Ah\n', C_req / n_parallel);
fprintf('Required energy: %.2f Wh\n', Energy / (n_parallel * n_series));
fprintf('%d cells in series and %d in parallel.\n', n_series, n_parallel);
end