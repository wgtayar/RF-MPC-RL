function [stats_pred, Vbus_interp, t_rel] = predict_Current_with_Voltage( ...
                t_power, Pc_all, voltageLogFile, Iopts)

    if nargin < 4 || isempty(Iopts)
        Iopts = struct();
    end

    % ensure column vectors
    t_power = t_power(:);
    Pc_all  = Pc_all(:);

    if numel(t_power) ~= numel(Pc_all)
        error('t_power and Pc_all must have the same length.');
    end

    t_rel = t_power - t_power(1);
    T_sim = t_rel(end);

    Vtab = readtable(voltageLogFile);

    requiredCols = {'sec','nsec','bus_voltage'};
    if ~all(ismember(requiredCols, Vtab.Properties.VariableNames))
        error('voltageLogFile must contain columns: sec, nsec, bus_voltage.');
    end

    t_abs = Vtab.sec + Vtab.nsec * 1e-9;
    Vbus = Vtab.bus_voltage(:);

    % sort by time ascending
    [t_abs_sorted, idxSort] = sort(t_abs(:), 'ascend');
    Vbus_sorted = Vbus(idxSort);

    t_volt_rel = t_abs_sorted - t_abs_sorted(1);

    idx_win = (t_volt_rel >= 0) & (t_volt_rel <= T_sim + 1e-6);
    t_volt_rel = t_volt_rel(idx_win);
    Vbus_segment = Vbus_sorted(idx_win);

    if numel(t_volt_rel) < 2
        error('Not enough voltage samples in the selected time window to interpolate.');
    end

    Vbus_interp = interp1(t_volt_rel, Vbus_segment, t_rel, 'linear', 'extrap');

    stats_pred = Ibus_pred(t_rel, Pc_all, Vbus_interp, Iopts);

    % logging
    fprintf('Voltage samples used: %d over %.3f s. Mean V = %.3f V\n', ...
        numel(t_volt_rel), t_volt_rel(end) - t_volt_rel(1), mean(Vbus_interp));
end
