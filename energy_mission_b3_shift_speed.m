function [speed,audit] = energy_mission_b3_shift_speed(source,s)
%ENERGY_MISSION_B3_SHIFT_SPEED Deployable charge-budget speed rule (protocol A3.3).
% [speed,audit] = energy_mission_b3_shift_speed(source,s) returns the
% constant speed (m/s) that the B3-shift baseline holds for a whole mission
% started at estimated SOC s (fraction). The rule is evaluated once per
% mission start; the R action stays at zero and gamma_a at GAMMA_A_MIN.
%
% source: a table or struct with the columns initial_soc (fraction),
% requested_speed_m_s, outcome and charge_As of fixed-speed single missions
% (sweep format), or the path (char/string) or paths (cell/string array) of
% sweep CSV files, which are concatenated. When a role column exists only
% rows with role 'benchmark' are kept; when a dataset_validated column exists
% only validated rows are kept. A (speed, start SOC) cell may appear once.
%
% Rule. Q(v,s) is the charge of completed missions at speed v, linear in
% start SOC between table starts and clamped at the table ends; the SOC drop
% is dSOC(v,s) = Q(v,s)/7200 (2 Ah pack). N(s) is the number of missions a
% shift completes from s at the frugal speed 0.75 m/s by chaining the table:
% a mission may start while s >= 0.30 and v completes at s (see (a)); a
% mission that would end below the 20% reserve fails and is not counted; the
% shift stops after the first completed mission ending below 0.30. The rule
% returns the fastest v in {1.0, 0.9, 0.85, 0.8, 0.75} such that
%   (a) v completed a mission at the nearest table start at or below s (the
%       lowest table start when no start is at or below s),
%   (b) s - dSOC(v,s) >= 0.21, and
%   (c) 1 + N(s - dSOC(v,s)) >= N(s),
% and 0.75 m/s when no speed qualifies. The Python reference is
% RL-MPC-Monitor docs/paper/figures/budget_rule.py; it differs only in (a)
% below the lowest table start, where it treats every speed as failing.
    validateattributes(s,{'double'},{'scalar','real','finite'},mfilename,'s');
    k = localConstants();
    t = localTable(source);
    need = localMissions(t,k,s);
    n = numel(k.speeds);
    completes = false(n,1); charge = NaN(n,1); endSoc = NaN(n,1);
    after = NaN(n,1); qualified = false(n,1);
    for i = 1:n
        v = k.speeds(i);
        completes(i) = localCompletes(t,v,s);
        charge(i) = localCharge(t,v,s);
        endSoc(i) = s-charge(i)/k.pack_As;
        if endSoc(i) >= 0
            after(i) = localMissions(t,k,endSoc(i));
        end
        qualified(i) = completes(i) && endSoc(i) >= k.min_end_soc && 1+after(i) >= need;
    end
    chosen = find(qualified,1);
    if isempty(chosen)
        speed = k.frugal; rule = 'fallback_frugal';
    else
        speed = k.speeds(chosen); rule = 'fastest_qualified';
    end
    audit = struct('schema','b3_shift_charge_budget_v1','start_soc_estimated',s, ...
        'speed_m_s',speed,'rule',rule,'missions_frugal',need, ...
        'candidate_speeds_m_s',k.speeds(:),'completes',completes,'charge_As',charge, ...
        'dsoc',charge/k.pack_As,'end_soc',endSoc,'missions_after',after,'qualified',qualified, ...
        'table_rows',height(t),'constants',k);
end

function k = localConstants()
k = struct('speeds',[1.0 0.9 0.85 0.8 0.75],'frugal',0.75,'pack_As',7200, ...
    'reserve_soc',0.20,'min_end_soc',0.21,'min_start_soc',0.30,'chain_cap',12);
end

function n = localMissions(t,k,s)
% Missions completed from s at the frugal speed (A3.1 shift rules). The
% chain cap matches the Python reference; with a positive charge per mission
% it is never reached from s <= 1.
n = 0;
while n < k.chain_cap && s >= k.min_start_soc && localCompletes(t,k.frugal,s)
    e = s-localCharge(t,k.frugal,s)/k.pack_As;
    if e < k.reserve_soc, return, end
    n = n+1;
    if e < k.min_start_soc, return, end
    s = e;
end
end

function tf = localCompletes(t,v,s)
rows = t(t.speed==round(v,3),:);
if isempty(rows), tf = false; return, end
below = rows.start <= s+1e-12;
if any(below)
    start = max(rows.start(below));
else
    start = min(rows.start);
end
tf = rows.completed(rows.start==start);
end

function q = localCharge(t,v,s)
rows = sortrows(t(t.speed==round(v,3) & t.completed,:),'start');
if isempty(rows)
    q = Inf;
elseif height(rows) == 1
    q = rows.charge(1);
else
    q = interp1(rows.start,rows.charge,min(max(s,rows.start(1)),rows.start(end)),'linear');
end
end

function t = localTable(source)
if ischar(source) || isstring(source) || iscell(source)
    paths = cellstr(source);
    assert(~isempty(paths),'b3Shift:Table','At least one sweep table path is required.');
    parts = cell(numel(paths),1);
    for i = 1:numel(paths)
        assert(isfile(paths{i}),'b3Shift:Table','Sweep table not found: %s',paths{i});
        opts = detectImportOptions(paths{i},'Delimiter',',');
        opts = setvartype(opts,intersect(opts.VariableNames,{'outcome','role','dataset_validated'}),'string');
        parts{i} = localColumns(readtable(paths{i},opts));
    end
    raw = vertcat(parts{:});
elseif istable(source)
    raw = localColumns(source);
elseif isstruct(source)
    columns = isscalar(source) && any(structfun(@(x)~ischar(x) && numel(x)>1,source));
    raw = localColumns(struct2table(source,'AsArray',~columns));
else
    error('b3Shift:Table','The table must be a table, struct or CSV path(s).');
end
keep = strcmp(raw.role,'benchmark') & raw.validated;
raw = raw(keep,:);
t = table(round(raw.requested_speed_m_s,3),round(raw.initial_soc,6),strcmp(raw.outcome,'mission_complete'), ...
    raw.charge_As,'VariableNames',{'speed','start','completed','charge'});
assert(~isempty(t),'b3Shift:Table','The sweep table has no benchmark rows.');
assert(all(isfinite(t.speed) & isfinite(t.start)) && all(isfinite(t.charge(t.completed)) & t.charge(t.completed)>0), ...
    'b3Shift:Table','Speeds, starts and completed-mission charges must be finite and positive.');
[~,first] = unique(t(:,{'speed','start'}),'rows');
assert(numel(first)==height(t),'b3Shift:DuplicateCell','A (speed, start SOC) cell appears more than once.');
end

function out = localColumns(t)
names = {'initial_soc','requested_speed_m_s','outcome','charge_As'};
assert(all(ismember(names,t.Properties.VariableNames)),'b3Shift:Table', ...
    'The sweep table needs initial_soc, requested_speed_m_s, outcome and charge_As.');
n = height(t);
role = repmat({'benchmark'},n,1);
if ismember('role',t.Properties.VariableNames), role = cellstr(string(t.role)); end
validated = true(n,1);
if ismember('dataset_validated',t.Properties.VariableNames)
    v = t.dataset_validated;
    if islogical(v) || isnumeric(v), validated = logical(v(:));
    else, validated = strcmpi(strtrim(cellstr(string(v))),'true');
    end
end
out = table(double(t.initial_soc(:)),double(t.requested_speed_m_s(:)),cellstr(string(t.outcome)), ...
    double(t.charge_As(:)),role,validated, ...
    'VariableNames',{'initial_soc','requested_speed_m_s','outcome','charge_As','role','validated'});
end
