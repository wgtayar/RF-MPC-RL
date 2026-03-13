function [dt, time_trim , Vsim_trim] = model_battery(current, time, C_nom, SOC_init, verbose)

if nargin < 5
    verbose = true;
end

SOC_min = 0.20;     
SOC_max = 1.00;     

% Parameter lookup tables 
rootDir = fileparts(mfilename('fullpath'));
params = readtable(fullfile(rootDir, 'our params.xlsx'));
params = params(params.SOC >= SOC_min, :);

SOC_lookup = params.SOC;
R0 = params.R0;
R1 = params.R1; C1 = params.C1;
R2 = params.R2; C2 = params.C2;
R3 = params.R3; C3 = params.C3;
OCV = params.OCV;

% Manual  interpolation function (SOC clamped)
function val = manualInterp(SOC_vector, param_vector, soc)

    % Clamp SOC 20-100
    soc = max(SOC_min, min(SOC_max, soc));

    if soc <= SOC_vector(1)
        val = param_vector(1);

    elseif soc >= SOC_vector(end)
        val = param_vector(end);

    else
        idx = find(SOC_vector <= soc, 1, 'last');

        soc_low  = SOC_vector(idx);
        soc_high = SOC_vector(idx+1);

        val_low  = param_vector(idx);
        val_high = param_vector(idx+1);

        val = val_low + (val_high - val_low) * (soc - soc_low) / (soc_high - soc_low);
    end
end

% Parameter interpolation
R0_func = @(soc) manualInterp(SOC_lookup, R0, soc);

R1_func = @(soc) manualInterp(SOC_lookup, R1, soc);
C1_func = @(soc) manualInterp(SOC_lookup, C1, soc);

R2_func = @(soc) manualInterp(SOC_lookup, R2, soc);
C2_func = @(soc) manualInterp(SOC_lookup, C2, soc);

R3_func = @(soc) manualInterp(SOC_lookup, R3, soc);
C3_func = @(soc) manualInterp(SOC_lookup, C3, soc);

% OCV interpolation
Voc_func = @(soc) interp1(SOC_lookup, OCV, soc, 'pchip');

% Initialization
N = length(time);
V_battery  = zeros(N,1);
V_battery(1) = Voc_func(SOC_init);
SOC_vector = zeros(N,1);
SOC = SOC_init;

V_RC1 = 0;
V_RC2 = 0;
V_RC3 = 0;

dt_vector = [0; diff(time)];

% Simulation Loop
for k = 2:N

    dt  = dt_vector(k);
    I_k = current(k);

    % Stop discharge at 0.2
    if SOC <= SOC_min
        if verbose
            fprintf("Simulation stopped at SOC = %.3f \n", SOC);
        end

        V_battery(k:end)  = V_battery(k-1);
        SOC_vector(k:end) = SOC;
        break;
    end

    % Clamp SOC
    SOC = max(SOC_min, min(SOC_max, SOC));

    % RC branch exponential factors
    alpha1 = exp(-dt / (R1_func(SOC) * C1_func(SOC)));
    alpha2 = exp(-dt / (R2_func(SOC) * C2_func(SOC)));
    alpha3 = exp(-dt / (R3_func(SOC) * C3_func(SOC)));

    % Update RC voltages
    V_RC1 = alpha1 * V_RC1 + (1 - alpha1) * I_k * R1_func(SOC);
    V_RC2 = alpha2 * V_RC2 + (1 - alpha2) * I_k * R2_func(SOC);
    V_RC3 = alpha3 * V_RC3 + (1 - alpha3) * I_k * R3_func(SOC);

    % Terminal voltage equation
    V_battery(k) = Voc_func(SOC) - I_k * R0_func(SOC) - V_RC1 - V_RC2 - V_RC3;
    
    % SOC update (coulomb counting)
    SOC = SOC - (I_k * dt) / (3600 * C_nom);
    SOC_vector(k) = SOC;

end

idx_valid = SOC_vector >= SOC_min;
time_trim  = time(idx_valid);
Vsim_trim  = V_battery(idx_valid);
Vsim_trim = lowpass(Vsim_trim,0.04,1/dt);

end