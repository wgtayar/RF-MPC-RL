function [SOC_estimated, SOC_final] = estimateSOC(time, I_total, V_batt, C_nom, SOC_init) 

    step_time = [0; diff(time)];

    params = readtable('updated_params.xlsx');
    valid_idx = isfinite(params.SOC) & isfinite(params.OCV);
    SOC_lookup = params.SOC(valid_idx);
    OCV = params.OCV(valid_idx);
    R0 = params.R0(valid_idx);
    R1 = params.R1(valid_idx);
    C1 = params.C1(valid_idx);
    R2 = params.R2(valid_idx);
    C2 = params.C2(valid_idx);
    R3 = params.R3(valid_idx);
    C3 = params.C3(valid_idx);

    R0_func = @(soc) manualInterp(SOC_lookup, R0, soc);
    R1_func = @(soc) manualInterp(SOC_lookup, R1, soc);
    C1_func = @(soc) manualInterp(SOC_lookup, C1, soc);
    R2_func = @(soc) manualInterp(SOC_lookup, R2, soc);
    C2_func = @(soc) manualInterp(SOC_lookup, C2, soc);
    R3_func = @(soc) manualInterp(SOC_lookup, R3, soc);
    C3_func = @(soc) manualInterp(SOC_lookup, C3, soc);
    Voc_offset = 0.01;
    Voc_func = @(soc) interp1(SOC_lookup, OCV, soc, 'linear', 'extrap') - Voc_offset;


    % Kalman filter parameters
    P = 1;
    Q = 1e-8;
    % R_kf = 2e-1;
    R_kf = 50;

    % Initialization
    N = length(time);
    SOC = SOC_init;
    SOC_estimated = zeros(N, 1);
    V_RC1 = 0; V_RC2 = 0; V_RC3 = 0;

    % EKF loop
    for k = 1:N
        dt = step_time(k);
        I_k = I_total(k);

        % Prediction Step
        SOC_pred = SOC - (I_k * dt / (C_nom * 3600));
        SOC_pred = max(0, min(1, SOC_pred));
        P_pred = P + Q;

        R0_k = R0_func(SOC_pred * 100);
        R1_k = R1_func(SOC_pred * 100);
        C1_k = C1_func(SOC_pred * 100);
        R2_k = R2_func(SOC_pred * 100);
        C2_k = C2_func(SOC_pred * 100);
        R3_k = R3_func(SOC_pred * 100);
        C3_k = C3_func(SOC_pred * 100);

        alpha1 = exp(-dt / (R1_k * C1_k));
        alpha2 = exp(-dt / (R2_k * C2_k));
        alpha3 = exp(-dt / (R3_k * C3_k));

        V_RC1 = alpha1 * V_RC1 + (1 - alpha1) * I_k * R1_k;
        V_RC2 = alpha2 * V_RC2 + (1 - alpha2) * I_k * R2_k;
        V_RC3 = alpha3 * V_RC3 + (1 - alpha3) * I_k * R3_k;

        % Predicted voltage
        V_pred = Voc_func(SOC_pred * 100) - (R0_k * I_k) - V_RC1 - V_RC2 - V_RC3;

        % Kalman Gain
        K = P_pred / (P_pred + R_kf);

        % Update Step
        SOC = SOC_pred + K * (V_batt(k) - V_pred);
        SOC = max(0, min(1, SOC));
        SOC_estimated(k) = SOC;

        P = (1 - K) * P_pred;
    end

    SOC_final = SOC;
    fprintf('SOC from estimator (final) = %.8f\n', SOC);
    
    if nargout == 0
        figure;
        plot(time, SOC_estimated * 100, 'k', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('SOC (%)');
        title('Estimated SOC using EKF');
        grid on;
    
        figure;
        plot(time, V_batt, 'r','LineWidth', 2);
        hold on;
        % plot(time, V_pred, 'b','LineWidth', 2);
        % legend('Measured Voltage', 'Predicted Voltage');
        title('Battery Terminal Voltage');
        xlabel('Time (s)');
    end
end

% Helper interpolation function
function val = manualInterp(SOC_vector, param_vector, soc)
    if soc <= SOC_vector(1)
        val = param_vector(1);
    elseif soc >= SOC_vector(end)
        val = param_vector(end);
    else
        idx = find(SOC_vector <= soc, 1, 'last');
        soc_low = SOC_vector(idx);
        soc_high = SOC_vector(idx + 1);
        val_low = param_vector(idx);
        val_high = param_vector(idx + 1);
        val = val_low + (val_high - val_low) * (soc - soc_low) / (soc_high - soc_low);
    end
end