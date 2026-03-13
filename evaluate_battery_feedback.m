function battery = evaluate_battery_feedback(timeTrace, currentTrace, cfg, packIn)
rootDir = fileparts(mfilename('fullpath'));

timeTrace = timeTrace(:);
currentTrace = currentTrace(:);

battery = struct();
battery.metric_type = 'soc';
battery.metric_value = 100 * cfg.SOC_init;
battery.margin_norm = cfg.SOC_init;
battery.soc_pct = 100 * cfg.SOC_init;
battery.n_series = cfg.n_series;
battery.n_parallel = cfg.n_parallel;
battery.trace_time = [];
battery.trace_current = [];
battery.trace_voltage = [];
battery.trace_metric = [];
battery.bms_input = table();

if nargin < 4 || isempty(packIn)
    packIn = struct();
end

if numel(timeTrace) < 2 || numel(currentTrace) < 2
    return
end

decim = max(1, cfg.decim);
idx = 1:decim:numel(timeTrace);
if idx(end) ~= numel(timeTrace)
    idx = [idx numel(timeTrace)];
end

tEval = timeTrace(idx);
iPack = abs(currentTrace(idx));

if cfg.use_pack_sizing
    [n_series, n_parallel] = batterySizing(tEval, iPack, cfg.pack_voltage, cfg.DoD);
else
    n_series = cfg.n_series;
    n_parallel = cfg.n_parallel;
end

iCell = iPack / max(n_parallel, 1);

[~, tBatt, VBatt] = model_battery(iCell, tEval, cfg.C_nom_Ah, cfg.SOC_init, false);

N = min([numel(tBatt), numel(iCell), numel(VBatt)]);
tBatt = tBatt(1:N);
iCell = iCell(1:N);
VBatt = VBatt(1:N);

if N < 2
    battery.n_series = n_series;
    battery.n_parallel = n_parallel;
    return
end

SOC = estimateSOC(tBatt, iCell, VBatt, cfg.C_nom_Ah, cfg.SOC_init, false);
SOC = SOC(:);

soc_pct = SOC(end);
soc_pct = min(max(soc_pct, 0), 100);

battery.metric_type = 'soc';
battery.metric_value = soc_pct;
battery.margin_norm = soc_pct / 100;
battery.soc_pct = soc_pct;
battery.n_series = n_series;
battery.n_parallel = n_parallel;
battery.trace_time = tBatt;
battery.trace_current = iCell;
battery.trace_voltage = VBatt;
battery.trace_metric = SOC;
battery.bms_input = table(tBatt, iPack(1:N), iCell, VBatt, SOC, ...
    'VariableNames', {'Time', 'PackCurrent', 'CellCurrent', 'Voltage', 'SOC'});
end