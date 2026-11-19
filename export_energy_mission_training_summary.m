function summary = export_energy_mission_training_summary(runRoot, outputDir, options)
%export_energy_mission_training_summary Small, read-only learning summary of a training run.
%   summary = export_energy_mission_training_summary(runRoot, outputDir)
%   reads a finished train_energy_mission_rl run and writes, into outputDir:
%     decisions.csv   (or decisions-NNN.csv, see split_decisions below)
%                     one row per decision: action, executed speed, reward terms,
%                     SOC, pack/drive electrical state, attitude, QP health
%     episodes.csv    one row per episode: initial/end SOC, end reason, return and
%                     its components, tau/kappa, charge, energy, speed, limits,
%                     toolbox EpisodeReward/EpisodeQ0, wall-clock minutes
%     policy_probe.csv the deterministic actor action and twin-critic values of
%                     every saved checkpoint at fixed probe observations (the
%                     first observation of every episode), to follow the policy
%     transitions.mat observation, next observation, normalized action, reward
%     summary.json    run identity, counts and the files written
%   The run folder is only read. outputDir must not exist yet.
%
%   options.probe_checkpoints (default true) evaluates every checkpoint.
%   options.split_decisions (default false) writes decisions-001.csv,
%     decisions-002.csv, ... in place of decisions.csv. Each part holds whole
%     episodes and is smaller than options.max_file_bytes (default 1e6 bytes);
%     non-integer numbers are rounded to options.significant_digits (default 6)
%     significant digits. Use it when the summary is published where files must
%     stay small. An episode too large for one part is an error. Other CSV
%     files at or above the limit are reported with a warning.
    arguments
        runRoot (1,:) char
        outputDir (1,:) char
        options.probe_checkpoints (1,1) logical = true
        options.split_decisions (1,1) logical = false
        options.max_file_bytes (1,1) double {mustBePositive,mustBeInteger} = 1e6
        options.significant_digits (1,1) double {mustBePositive,mustBeInteger} = 6
    end
    assert(isfolder(fullfile(runRoot,'decisions')) && isfile(fullfile(runRoot,'training','final_agent.mat')), ...
        'export_energy_mission_training_summary:NotARun','%s is not a finished training run.',runRoot);
    assert(~exist(outputDir,'file'),'export_energy_mission_training_summary:OutputExists', ...
        'Output folder %s already exists; choose a new one.',outputDir);
    files = dir(fullfile(runRoot,'decisions','episode_*_decision_*.mat'));
    assert(~isempty(files),'export_energy_mission_training_summary:NoDecisions','No decisions recorded.');
    names = sort(string({files.name}));
    n = numel(names);
    ids = zeros(n,2);
    for k = 1:n
        ids(k,:) = sscanf(names(k),'episode_%d_decision_%d.mat')';
    end
    [ids,order] = sortrows(ids); names = names(order);
    rows = cell(n,1);
    obs = []; nextObs = []; actions = zeros(5,n); rewards = zeros(1,n); done = false(1,n);
    fileTimes = zeros(n,1);
    for k = 1:n
        path = fullfile(runRoot,'decisions',names(k));
        record = load(path,'event').event.record;
        audit = load(fullfile(runRoot,'normalized_actions',names(k)),'audit').audit;
        rows{k} = localDecisionRow(ids(k,:),record,audit);
        if isempty(obs)
            obs = zeros(numel(record.observation),n,'single'); nextObs = obs;
        end
        obs(:,k) = record.observation; nextObs(:,k) = record.next_observation;
        actions(:,k) = audit.requested_normalized(:); rewards(k) = record.reward;
        done(k) = logical(record.is_done);
        info = dir(path); fileTimes(k) = info.datenum;
    end
    decisions = struct2table(vertcat(rows{:}));
    final = load(fullfile(runRoot,'training','final_agent.mat'),'stats','agent');
    report = load(fullfile(runRoot,'training','report.mat'),'report').report;
    episodes = localEpisodes(runRoot,decisions,final.stats,fileTimes);
    mkdir(outputDir);
    if options.split_decisions
        decisionFiles = localWriteSplit(localRound(decisions,options.significant_digits), ...
            decisions.episode,outputDir,'decisions',options.max_file_bytes);
    else
        writetable(decisions,fullfile(outputDir,'decisions.csv'));
        decisionFiles = {'decisions.csv'};
    end
    writetable(episodes,fullfile(outputDir,'episodes.csv'));
    episodeIndex = ids(:,1)'; decisionIndex = ids(:,2)'; %#ok<NASGU>
    save(fullfile(outputDir,'transitions.mat'),'obs','nextObs','actions','rewards','done', ...
        'episodeIndex','decisionIndex','-v7.3');
    written = [decisionFiles,{'episodes.csv','transitions.mat'}];
    probe = table();
    if options.probe_checkpoints
        probe = localProbe(runRoot,obs(:,decisions.decision==1),episodes.episode(:)');
        if ~isempty(probe)
            writetable(probe,fullfile(outputDir,'policy_probe.csv'));
            written{end+1} = 'policy_probe.csv';
        end
    end
    summary = struct('schema','energy_mission_training_summary_v1','run_root',runRoot, ...
        'run_id',report.run_id,'source_sha',report.source_sha,'seed',report.seed, ...
        'time_weight',decisions.time_weight(1),'episodes',height(episodes), ...
        'decisions',height(decisions),'cumulative_episodes_trained',localField(report,'cumulative_episodes_trained',height(episodes)), ...
        'resumed',~isempty(localField(report,'resumed_from',[])), ...
        'end_reasons',localCounts(episodes.end_reason),'infeasible_decisions',nnz(~decisions.feasible), ...
        'probe_checkpoints',height(unique(probe(:,intersect('checkpoint',probe.Properties.VariableNames)))), ...
        'files',{written});
    written{end+1} = 'summary.json';
    summary.files = written;
    if options.split_decisions
        summary.decision_files = decisionFiles;
        summary.max_file_bytes = options.max_file_bytes;
        summary.significant_digits = options.significant_digits;
    end
    fid = fopen(fullfile(outputDir,'summary.json'),'w');
    fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true)); fclose(fid);
    if options.split_decisions
        for k = 1:numel(written)
            [~,~,ext] = fileparts(written{k});
            info = dir(fullfile(outputDir,written{k}));
            if ~strcmp(ext,'.mat') && info.bytes >= options.max_file_bytes
                warning('export_energy_mission_training_summary:LargeFile', ...
                    '%s is %d bytes, not below %d.',written{k},info.bytes,options.max_file_bytes);
            end
        end
    end
end

function T = localRound(T,digits)
% Round non-integer floating values to the given significant digits; integer
% values (counters, indices) are kept exactly.
    names = T.Properties.VariableNames;
    for k = 1:numel(names)
        v = T.(names{k});
        if isfloat(v)
            r = round(double(v),digits,'significant');
            keep = ~isfinite(v) | v == round(v);
            r(keep) = v(keep);
            T.(names{k}) = r;
        end
    end
end

function files = localWriteSplit(T,episode,outputDir,stem,maxBytes)
% Write T as stem-001.csv, stem-002.csv, ... Each part keeps whole episodes
% (rows of T are grouped by episode in order) and is below maxBytes.
    temp = [tempname '.csv'];
    cleanup = onCleanup(@() localDelete(temp));
    writetable(T,temp);
    fid = fopen(temp,'r'); bytes = fread(fid,Inf,'*uint8')'; fclose(fid);
    if isempty(bytes) || bytes(end) ~= uint8(10), bytes(end+1) = uint8(10); end
    ends = find(bytes == uint8(10));
    assert(numel(ends) == height(T)+1,'export_energy_mission_training_summary:CsvLines', ...
        'Expected one line per row in the decision table.');
    header = bytes(1:ends(1));
    rowStart = ends(1:end-1)+1; rowEnd = ends(2:end);
    [labels,~,group] = unique(episode(:),'stable');
    assert(all(diff(group) >= 0),'export_energy_mission_training_summary:EpisodeOrder', ...
        'Decision rows must be grouped by episode.');
    files = {}; part = uint8([]); partEpisodes = 0;
    for g = 1:numel(labels)
        rows = find(group == g);
        block = bytes(rowStart(rows(1)):rowEnd(rows(end)));
        assert(numel(header)+numel(block) < maxBytes,'export_energy_mission_training_summary:EpisodeTooLarge', ...
            'Episode %d alone needs %d bytes, not below %d.',labels(g),numel(header)+numel(block),maxBytes);
        if partEpisodes > 0 && numel(header)+numel(part)+numel(block) >= maxBytes
            files{end+1} = localWritePart(outputDir,stem,numel(files)+1,[header part]); %#ok<AGROW>
            part = uint8([]); partEpisodes = 0;
        end
        part = [part block]; %#ok<AGROW>
        partEpisodes = partEpisodes+1;
    end
    if partEpisodes > 0
        files{end+1} = localWritePart(outputDir,stem,numel(files)+1,[header part]);
    end
end

function name = localWritePart(outputDir,stem,index,bytes)
    name = sprintf('%s-%03d.csv',stem,index);
    fid = fopen(fullfile(outputDir,name),'w');
    fwrite(fid,bytes,'uint8'); fclose(fid);
end

function localDelete(path)
    if isfile(path), delete(path); end
end

function row = localDecisionRow(id,r,audit)
    w = r.window; b = w.battery; x = r.action_execution; ri = r.reward_info;
    att = localField(w,'attitude',struct());
    monitor = localField(ri,'standardized_monitor',struct());
    scales = localField(ri,'scales',struct());
    reason = string(localField(r,'terminal_reason',""));
    if strlength(reason) == 0, reason = ""; end
    req = audit.requested_normalized(:)';
    dR = localField(x,'dR_effective',nan(3,1));
    row = struct('episode',id(1),'decision',id(2), ...
        'time_weight',localField(scales,'time_weight',NaN), ...
        'elapsed_time_s',w.elapsed_time_s,'duration_s',w.duration_s, ...
        'distance_start_m',w.distance_start_m,'distance_end_m',w.distance_end_m, ...
        'progress_frac',w.progress_frac,'time_frac',w.time_frac,'lag_frac',w.lag_frac, ...
        'soc_start_pct',w.soc_start_pct,'soc_end_pct',w.soc_end_pct, ...
        'a1_req',req(1),'a2_req',req(2),'a3_req',req(3),'a4_req',req(4),'a5_req',req(5), ...
        'gamma_v_raw',x.gamma_v_raw,'gamma_v_applied',x.gamma_v_applied, ...
        'gamma_a_raw',x.gamma_a_raw,'gamma_a_applied',x.gamma_a_applied, ...
        'gamma_rate_limited',logical(x.gamma_rate_limited),'R_saturated',logical(x.R_saturated), ...
        'dR1',dR(1),'dR2',dR(2),'dR3',dR(3), ...
        'v_req',w.v_req,'v_exec',w.v_exec,'a_exec',w.a_exec,'delta_v_exec',w.delta_v_exec, ...
        'com_speed',w.com_speed_mag,'tracking_error_mean',w.tracking_error_mean, ...
        'control_effort_mean',w.control_effort_mean, ...
        'reward',r.reward,'r_progress',localField(ri,'progress_contribution',NaN), ...
        'r_time',-localField(ri,'time_cost',NaN),'r_charge',-localField(ri,'charge_cost',NaN), ...
        'r_terminal',localField(ri,'terminal_contribution',NaN), ...
        'booked_charge_As',localField(ri,'booked_charge_As',NaN), ...
        'tau_cumulative',localField(monitor,'tau_cumulative',NaN), ...
        'kappa_cumulative',localField(monitor,'kappa_cumulative',NaN), ...
        'Ieq_A',w.Ieq_window,'charge_As',w.charge_As,'energy_Wh',w.energy_Wh, ...
        'pack_voltage_V',localField(b,'pack_voltage_V',NaN),'pack_current_A',localField(b,'pack_current_A',NaN), ...
        'power_W',localField(b,'power_W',NaN),'drive_required_V',localField(b,'drive_required_V',NaN), ...
        'drive_headroom_V',localField(b,'drive_voltage_headroom_V',NaN), ...
        'peak_motor_torque_Nm',localField(b,'peak_motor_torque_Nm',NaN), ...
        'peak_cell_current_A',localField(b,'drive_sampled_peak_cell_current_A',NaN), ...
        'voltage_limit_steps',localField(b,'voltage_limit_steps',NaN), ...
        'torque_limit_steps',localField(b,'torque_limit_steps',NaN), ...
        'max_tilt_rad',localField(att,'max_tilt_rad',NaN),'rms_tilt_rad',localField(att,'rms_tilt_rad',NaN), ...
        'max_body_rate_rad_s',localField(att,'max_body_rate_rad_s',NaN), ...
        'qp_solves',w.qp_solve_count,'feasible',logical(w.feasible), ...
        'recovered_solver_events',sum(double(localField(w,'recovered_solver_events',0))), ...
        'chunks_completed',r.chunks_completed,'is_done',logical(r.is_done),'terminal_reason',reason);
end

function T = localEpisodes(runRoot,D,stats,fileTimes)
    list = unique(D.episode)';
    rows = cell(numel(list),1);
    for k = 1:numel(list)
        e = list(k); m = D.episode == e; d = D(m,:); last = d(end,:);
        startRecord = load(fullfile(runRoot,'episodes',sprintf('episode_%06d_start.mat',e)),'event').event.record;
        initialSoc = NaN;
        if isfield(startRecord,'initial_soc_audit'), initialSoc = 100*startRecord.initial_soc_audit.initial_soc; end
        reason = last.terminal_reason;
        if strlength(reason) == 0, reason = "step_limit"; end
        s = find(stats.EpisodeIndex == k,1);
        q0 = NaN; toolboxReward = NaN;
        if ~isempty(s), q0 = stats.EpisodeQ0(s); toolboxReward = stats.EpisodeReward(s); end
        t0 = dir(fullfile(runRoot,'episodes',sprintf('episode_%06d_start.mat',e)));
        t1 = dir(fullfile(runRoot,'episodes',sprintf('episode_%06d_end.mat',e)));
        if isempty(t1), t1 = struct('datenum',max(fileTimes(m))); end
        rows{k} = struct('episode',e,'initial_soc_pct',initialSoc,'end_soc_pct',last.soc_end_pct, ...
            'end_reason',reason,'completed',reason == "mission_complete",'decisions',height(d), ...
            'distance_m',last.distance_end_m,'sim_time_s',last.elapsed_time_s, ...
            'episode_return',sum(d.reward),'return_progress',sum(d.r_progress),'return_time',sum(d.r_time), ...
            'return_charge',sum(d.r_charge),'return_terminal',sum(d.r_terminal), ...
            'toolbox_episode_reward',toolboxReward,'episode_q0',q0, ...
            'tau_final',last.tau_cumulative,'kappa_final',last.kappa_cumulative, ...
            'charge_As',sum(d.booked_charge_As),'energy_Wh',sum(d.energy_Wh), ...
            'mean_v_exec',mean(d.v_exec),'max_v_exec',max(d.v_exec),'last_v_exec',last.v_exec, ...
            'mean_gamma_v_raw',mean(d.gamma_v_raw),'rate_limited_fraction',mean(d.gamma_rate_limited), ...
            'min_drive_headroom_V',min(d.drive_headroom_V),'max_drive_required_V',max(d.drive_required_V), ...
            'voltage_limit_steps',sum(d.voltage_limit_steps),'torque_limit_steps',sum(d.torque_limit_steps), ...
            'max_tilt_rad',max(d.max_tilt_rad),'mean_Ieq_A',mean(d.Ieq_A),'max_power_W',max(d.power_W), ...
            'infeasible_decisions',nnz(~d.feasible),'recovered_solver_events',sum(d.recovered_solver_events), ...
            'wall_clock_min',(t1.datenum-t0.datenum)*1440);
    end
    T = struct2table(vertcat(rows{:}));
end

function T = localProbe(runRoot,probeObs,probeEpisodes)
    files = dir(fullfile(runRoot,'training','checkpoints','Agent*.mat'));
    if isempty(files), T = table(); return, end
    numbers = str2double(erase(erase(string({files.name}),'Agent'),'.mat'));
    [numbers,order] = sort(numbers); files = files(order);
    p = size(probeObs,2);
    rows = cell(numel(files)*p,1); count = 0;
    for f = 1:numel(files)
        saved = load(fullfile(files(f).folder,files(f).name));
        agent = saved.(localAgentName(saved));
        actor = getActor(agent); critics = getCritic(agent);
        for j = 1:p
            o = double(probeObs(:,j));
            a = getAction(actor,{o}); a = double(a{1}(:));
            q1 = double(getValue(critics(1),{o},{a})); q2 = double(getValue(critics(2),{o},{a}));
            count = count+1;
            rows{count} = struct('checkpoint',numbers(f),'probe_episode',probeEpisodes(j), ...
                'a1',a(1),'a2',a(2),'a3',a(3),'a4',a(4),'a5',a(5),'q1',q1,'q2',q2,'q_min',min(q1,q2));
        end
    end
    T = struct2table(vertcat(rows{1:count}));
end

function name = localAgentName(saved)
    names = fieldnames(saved);
    for k = 1:numel(names)
        if isa(saved.(names{k}),'rl.agent.AbstractAgent') || isa(saved.(names{k}),'rl.agent.rlTD3Agent')
            name = names{k}; return
        end
    end
    error('export_energy_mission_training_summary:NoAgent','Checkpoint holds no agent.');
end

function counts = localCounts(values)
    [labels,~,index] = unique(values);
    counts = struct();
    for k = 1:numel(labels)
        counts.(matlab.lang.makeValidName(labels(k))) = nnz(index == k);
    end
end

function value = localField(s,name,default)
    if isstruct(s) && isfield(s,name) && ~isempty(s.(name))
        value = s.(name);
    else
        value = default;
    end
end
