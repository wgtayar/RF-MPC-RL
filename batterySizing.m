% function batterySizing(Time, Total_Current, Voltage)
% 
% % Calculate power curve: P=I*V
% Power = Total_Current * Voltage;
% plot(Time,Power)
% title ('Total Power Curve');
% xlabel ('Time (sec)');
% ylabel ('Power (W)');
% 
% % Integrate the power curve to get the energy in Joules then convert to Wh
% Energy = trapz(Time, Power) / 3600; % Step size of the time vector in non-uniform
% 
% % Calculate total needed capacity
% Capacity =(Energy/Voltage);
% 
% % Calculate the number of battery cells needed based on the nominal values of a single cell
% n_series = ceil(Voltage/3.3);
% n_parallel = ceil(Capacity/1.2);
% 
% % Print out the results
% fprintf('The required battery energy capacity is %f Wh.\n', Energy);
% fprintf('The required battery capacity is %f Ah.\n', Capacity);
% fprintf('The required number of battery cells is: %d in series and %d in parallel.\n',n_series,n_parallel );
% 
% end

function [Energy_Wh, Capacity_Ah, n_series, n_parallel] = batterySizing(Time, I_total, V_pack_nom)

    % design parameters, per cell
    V_cell_nom = 3.3;
    C_cell_nom = 1.2;
    DoD = 1.0;
    
    Power = I_total .* V_pack_nom;
    
    % plot
    figure;
    plot(Time, Power);
    title('Total Power Curve');
    xlabel('Time (s)');
    ylabel('Power (W)');
    grid on;
    
    % energy
    Energy_Wh = trapz(Time, Power) / 3600;
    
    Capacity_Ah = Energy_required_Wh / (V_pack_nom * DoD);
    
    % series / parallel sizing
    n_series   = ceil(V_pack_nom / V_cell_nom);
    n_parallel = ceil(Capacity_Ah / C_cell_nom);
    
    fprintf('Required energy (no margin): %.3f Wh\n', Energy_Wh);
    fprintf('Required energy (with margin): %.3f Wh\n', Energy_required_Wh);
    fprintf('Required pack capacity: %.3f Ah\n', Capacity_Ah);
    fprintf('Number of cells: %d in series, %d in parallel\n', n_series, n_parallel);
end
