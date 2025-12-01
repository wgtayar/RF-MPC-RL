function [SOC_current, I_chunk_mean, t_power_out, Pc_all_out, I_all_out] = ...
    update_SOC_from_power_RL(tout, Xout, Uout, t_power_in, Pc_all_in, p)

    [tpc_chunk, Pc_chunk] = power_calc(tout, Xout, Uout, struct('BI', p.J));

    t_power = [t_power_in; tpc_chunk];
    Pc_all = [Pc_all_in;  Pc_chunk];

    Iopts = struct('I0', 1.0, 'eta', 1.6, 'clipNeg', true, 'smoothWin', 0);
    voltageFile = 'Node8_Voltage.xlsx';

    [stats_pred, Vbus_interp, t_rel] = ...
        predict_Current_with_Voltage(t_power, Pc_all, voltageFile, Iopts);

    I_bus = stats_pred.Ipred(:);

    nChunk = numel(tpc_chunk);
    I_chunk = I_bus(end-nChunk+1:end);
    I_chunk_mean = mean(I_chunk);

    params = struct();
    params.V_cell_nom = 3.3;
    params.C_cell_nom = 0.12;  % smaller virtual capacity
    params.n_series_fixed = 6;

    [Energy_Wh, Capacity_Ah, n_s, n_p, pack] = ...
        battery_Sizing_from_Trace(t_rel, I_bus, Vbus_interp, params);

    I_cell = I_bus ./ n_p;
    V_cell_t = Vbus_interp ./ n_s;

    C_nom_cell = params.C_cell_nom;

    % Coulomb-count SOC
    dt_vec = [0; diff(t_rel)];                % [s]
    Q_drawn_cc = cumsum(I_cell .* dt_vec) / 3600; % [Ah]
    SOC_cc_vec = 1.0 - Q_drawn_cc / C_nom_cell;
    SOC_cc_vec = max(0, min(1, SOC_cc_vec));
    SOC_cc_final = SOC_cc_vec(end);

    % EKF SOC
    SOC_init = 1.0;
    [SOC_est_vec, SOC_EKF_final] = ...
        estimateSOC(t_rel, I_cell, V_cell_t, C_nom_cell, SOC_init);

    fprintf('SOC from estimator (final) = %.8f\n', SOC_EKF_final);
    fprintf('[RL-SOC debug] t_end=%.2f s | I_bus_mean=%.3f A | Q_drawn=%.6f Ah | SOC_cc=%.6f | SOC_EKF=%.6f\n', ...
        t_rel(end), mean(I_bus), Q_drawn_cc(end), SOC_cc_final, SOC_EKF_final);

    SOC_current = SOC_cc_final;   % Can be changed to SOC_EKF_final
    t_power_out = t_power;
    Pc_all_out = Pc_all;
    I_all_out = I_bus;
end
