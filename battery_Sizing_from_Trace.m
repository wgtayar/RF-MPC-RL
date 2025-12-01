function [Energy_Wh, Capacity_Ah, n_series, n_parallel, pack] = ...
         battery_Sizing_from_Trace(Time, I_bus, V_bus, params)

    if nargin < 4 || isempty(params)
        params = struct();
    end
    if ~isfield(params,'V_cell_nom'), params.V_cell_nom = 3.3; end
    if ~isfield(params,'C_cell_nom'), params.C_cell_nom = 1.2; end

    Time  = Time(:);
    I_bus = I_bus(:);

    if isscalar(V_bus)
        V_bus = V_bus * ones(size(Time));
    else
        V_bus = V_bus(:);
        if numel(V_bus) ~= numel(Time)
            error('V_bus must be scalar or have same length as Time.');
        end
    end

    % Power and energy
    P = I_bus .* V_bus;           % [W]
    Energy_J  = trapz(Time, P);   % [J]
    Energy_Wh = Energy_J / 3600;

    V_pack_nom = mean(V_bus);

    Capacity_Ah = Energy_Wh / V_pack_nom;  % will be tiny for 1 s

    if isfield(params,'n_series_fixed')
        n_series = params.n_series_fixed;
    else
        n_series = max(1, round(V_pack_nom / params.V_cell_nom));
    end

    n_parallel = max(1, ceil(Capacity_Ah / params.C_cell_nom));

    % Pack info
    pack = struct();
    pack.V_pack_nom = V_pack_nom;
    pack.V_cell_nom = params.V_cell_nom;
    pack.C_cell_nom = params.C_cell_nom;
    pack.Energy_Wh = Energy_Wh;
    pack.Capacity_Ah = Capacity_Ah;
    pack.n_series = n_series;
    pack.n_parallel = n_parallel;

    fprintf('Energy drawn: %.6f Wh\n',  Energy_Wh);
    fprintf('Equivalent capacity: %.6f Ah\n',  Capacity_Ah);
    fprintf('Nominal pack V: %.2f V\n',  V_pack_nom);
    fprintf('Cells in series: %d, parallel: %d\n', n_series, n_parallel);
end