function [battery,audit] = evaluateAligned(timeTrace,currentTrace,cfg)
%evaluateAligned Replay full battery history with timestamp-aligned currents.
% Full histories, not previous SOC, initialize the unchanged model and estimator.
    validateattributes(timeTrace,{'double'},{'real','finite'});
    validateattributes(currentTrace,{'double'},{'real','finite'});
    assert((isvector(timeTrace) || isempty(timeTrace)) && ...
        (isvector(currentTrace) || isempty(currentTrace)) && ...
        numel(timeTrace)==numel(currentTrace),'phase3battery:Length', ...
        'Require matching time/current vectors.');
    timeTrace = timeTrace(:);
    currentTrace = currentTrace(:);
    assert(all(diff(timeTrace)>0),'phase3battery:Order','History must be strictly ordered.');
    battery = struct('metric_type','soc','metric_value',100*cfg.SOC_init, ...
        'margin_norm',cfg.SOC_init,'soc_pct',100*cfg.SOC_init, ...
        'n_series',cfg.n_series,'n_parallel',cfg.n_parallel, ...
        'trace_time',[],'trace_current',[],'trace_voltage',[],'trace_metric',[], ...
        'bms_input',table());
    audit = struct('version','battery_feedback_timestamp_aligned_v2', ...
        'retained_decimated_indices',zeros(0,1),'retained_history_indices',zeros(0,1), ...
        'interpolation_used',false,'pack_state_used',false);
    if numel(timeTrace)<2
        return
    end
    validateattributes(cfg.decim,{'double'},{'scalar','real','finite','integer','positive'});
    indices = (1:cfg.decim:numel(timeTrace)).';
    if indices(end)~=numel(timeTrace)
        indices(end+1) = numel(timeTrace);
    end
    tEval = timeTrace(indices);
    inputPackCurrent = abs(currentTrace(indices));
    if cfg.use_pack_sizing
        [nSeries,nParallel] = batterySizing(tEval,inputPackCurrent,cfg.pack_voltage,cfg.DoD);
        validateattributes(nParallel,{'double'},{'scalar','real','finite','integer','nonnegative'});
    else
        nSeries = cfg.n_series;
        nParallel = cfg.n_parallel;
        validateattributes(nParallel,{'double'},{'scalar','real','finite','integer','positive'});
    end
    % Preserve the legacy one-cell divisor when sizing a zero-load trace returns zero.
    effectiveParallel = max(nParallel,1);
    audit.effective_parallel_divisor = effectiveParallel;
    [~,tBatt,voltage] = model_battery(inputPackCurrent/effectiveParallel,tEval,cfg.C_nom_Ah,cfg.SOC_init,false);
    tBatt = tBatt(:);
    voltage = voltage(:);
    assert(numel(tBatt)==numel(voltage),'phase3battery:Length','Battery output lengths differ.');
    validateattributes(voltage,{'double'},{'real','finite'});
    [packCurrent,cellCurrent,retained] = phase3battery.alignSamples(tEval,inputPackCurrent,tBatt,effectiveParallel);
    audit.retained_decimated_indices = retained;
    audit.retained_history_indices = indices(retained);
    battery.n_series = nSeries;
    battery.n_parallel = nParallel;
    if numel(tBatt)<2
        return
    end
    SOC = estimateSOC(tBatt,cellCurrent,voltage,cfg.C_nom_Ah,cfg.SOC_init,false);
    SOC = SOC(:);
    validateattributes(SOC,{'double'},{'real','finite','column','numel',numel(tBatt)});
    battery.soc_pct = min(max(SOC(end),0),100);
    battery.metric_value = battery.soc_pct;
    battery.margin_norm = battery.soc_pct/100;
    battery.trace_time = tBatt;
    battery.trace_current = cellCurrent;
    battery.trace_voltage = voltage;
    battery.trace_metric = SOC;
    battery.bms_input = table(tBatt,packCurrent,cellCurrent,voltage,SOC, ...
        'VariableNames',{'Time','PackCurrent','CellCurrent','Voltage','SOC'});
end
