function [SOC] = estimateSOC(time, I_total, V_batt, C_nom, SOC_init, doPlot)

if nargin < 6
    doPlot = false;
end

N0 = min([length(time), length(I_total), length(V_batt)]);
time    = time   (1:N0);
I_total = I_total(1:N0);
V_batt  = V_batt (1:N0);

N = length(time);

% Load parameters
params = readtable('our params.xlsx');
valid_idx = isfinite(params.SOC) & isfinite(params.OCV);

SOC_table = params.SOC(valid_idx);
OCV_table = params.OCV(valid_idx);

R0_table = params.R0(valid_idx);
R1_table = params.R1(valid_idx); C1_table = params.C1(valid_idx);
R2_table = params.R2(valid_idx); C2_table = params.C2(valid_idx);
R3_table = params.R3(valid_idx); C3_table = params.C3(valid_idx);

SOC_min = 0.1;
SOC_max = 1.0;

dOCV_table = gradient(OCV_table) ./ gradient(SOC_table);

% Manual interpolation
function val = manualInterp(SOC_vector,param_vector,soc)

soc = max(SOC_min,min(SOC_max,soc));

if soc <= SOC_vector(1)
    val = param_vector(1);

elseif soc >= SOC_vector(end)
    val = param_vector(end);

else
    idx = find(SOC_vector<=soc,1,'last');

    soc_low = SOC_vector(idx);
    soc_high = SOC_vector(idx+1);

    val_low = param_vector(idx);
    val_high = param_vector(idx+1);

    val = val_low + (val_high-val_low)*(soc-soc_low)/(soc_high-soc_low);
end
end

% Parameter functions
OCV_fun = @(soc) interp1(SOC_table, OCV_table, soc, 'pchip');

R0_fun = @(soc) manualInterp(SOC_table,R0_table,soc);

R1_fun = @(soc) manualInterp(SOC_table,R1_table,soc);
C1_fun = @(soc) manualInterp(SOC_table,C1_table,soc);

R2_fun = @(soc) manualInterp(SOC_table,R2_table,soc);
C2_fun = @(soc) manualInterp(SOC_table,C2_table,soc);

R3_fun = @(soc) manualInterp(SOC_table,R3_table,soc);
C3_fun = @(soc) manualInterp(SOC_table,C3_table,soc);

dOCV_fun = @(soc) manualInterp(SOC_table, dOCV_table, soc);

% EKF noise parameters
q_soc  = 1e-7;
q_vrc1 = 1e-3;
q_vrc2 = 1e-3;
q_vrc3 = 1e-3;

Q = diag([q_soc q_vrc1 q_vrc2 q_vrc3]);

R_kf = 1e-3;

P = diag([1e-4 1e-3 1e-3 1e-3]);

% Initialization
step_time = [0; diff(time)];

x = zeros(4,1);

x(1) = SOC_init;
x(2:4) = 0;

x_prior = zeros(4,N);
x_post  = zeros(4,N);

Vpred = zeros(N,1);
P_trace = zeros(N,1);

SOC_CC = zeros(N,1);
SOC_CC(1) = x(1);

% EKF loop
for k=1:N

dt = step_time(k);
if dt<=0
dt = 1e-6;
end

I_k = I_total(k);

% Prediction step
SOC_pred = x(1) - I_k*dt/(C_nom*3600);
SOC_pred = max(SOC_min,min(SOC_max,SOC_pred));

R0_k = R0_fun(SOC_pred);

R1_k = R1_fun(SOC_pred); C1_k = C1_fun(SOC_pred);
R2_k = R2_fun(SOC_pred); C2_k = C2_fun(SOC_pred);
R3_k = R3_fun(SOC_pred); C3_k = C3_fun(SOC_pred);

a1 = exp(-dt/(R1_k*C1_k)); b1 = (1-a1)*R1_k;
a2 = exp(-dt/(R2_k*C2_k)); b2 = (1-a2)*R2_k;
a3 = exp(-dt/(R3_k*C3_k)); b3 = (1-a3)*R3_k;

Vrc1_pred = a1*x(2) + b1*I_k;
Vrc2_pred = a2*x(3) + b2*I_k;
Vrc3_pred = a3*x(4) + b3*I_k;

x_pred = [SOC_pred; Vrc1_pred; Vrc2_pred; Vrc3_pred];

x_prior(:,k) = x_pred;

% Covariance prediction
F = eye(4);
F(2,2)=a1;
F(3,3)=a2;
F(4,4)=a3;

P_pred = F*P*F' + Q;

% Voltage prediction
V_pred = OCV_fun(SOC_pred) - R0_k*I_k - Vrc1_pred - Vrc2_pred - Vrc3_pred;
Vpred(k) = V_pred;

% Jacobian
dOCV = dOCV_fun(SOC_pred);

H = zeros(1,4);
H(1) = dOCV;
H(2) = -1;
H(3) = -1;
H(4) = -1;

% Kalman gain
S = H*P_pred*H' + R_kf;

K = (P_pred*H')/S;

% Update
e = V_batt(k) - V_pred;

x = x_pred + K*e;

x(1) = max(SOC_min,min(SOC_max,x(1)));

P = (eye(4)-K*H)*P_pred;

x_post(:,k) = x;

P_trace(k) = trace(P);

% Coulomb counting
if k>1

SOC_CC(k) = SOC_CC(k-1) - I_k*dt/(C_nom*3600);
SOC_CC(k) = max(SOC_min,min(SOC_max,SOC_CC(k)));
SOC_CC(1) = SOC_init;

else

SOC_CC(k) = x(1);

end

end

SOC = x_post(1,:)*100;
SOC(1) = SOC_init*100;

% Plots
if doPlot
    time_s = time - time(1);

    figure
    plot(time_s, SOC, 'r', 'LineWidth', 1.5);
    hold on
    plot(time_s, SOC_CC * 100, 'b--', 'LineWidth', 1.5)
    xlabel('Time (s)')
    ylabel('SOC (%)')
    legend('EKF', 'Coulomb Counting')
    grid on
    title('SOC Estimation')
    
    fprintf('Final EKF SOC: %.4f / Coulomb Counting SOC: %.4f\n', x(1), SOC_CC(end));
end


end