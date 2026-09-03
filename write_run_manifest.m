function metadata = write_run_manifest(manifestPath, cfg, runInfo, sourceAgentPath)
%write_run_manifest Write the authoritative reproducibility manifest.

    if nargin < 3 || isempty(runInfo)
        runInfo = struct();
    elseif ~isstruct(runInfo)
        legacyMode = runInfo;
        runInfo = struct('training_mode', legacyMode);
    end
    if nargin >= 4 && ~isempty(sourceAgentPath)
        runInfo.source_agent = sourceAgentPath;
    end

    rootDir = fileparts(mfilename('fullpath'));
    metadata = localCollectMetadata(rootDir, cfg, runInfo);

    manifestFolder = fileparts(manifestPath);
    if ~isempty(manifestFolder) && ~isfolder(manifestFolder)
        mkdir(manifestFolder);
    end

    fid = fopen(manifestPath, 'w');
    if fid < 0
        error('write_run_manifest:OpenFailed', ...
            'Could not write run manifest: %s', manifestPath);
    end
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

    fields = fieldnames(metadata);
    for i = 1:numel(fields)
        localPrintValue(fid, fields{i}, metadata.(fields{i}));
    end
end

function metadata = localCollectMetadata(rootDir, cfg, runInfo)
    nowLocal = datetime('now', 'TimeZone', 'local', ...
        'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX');

    metadata = struct();
    metadata.run_id = localGet(runInfo, {'run_id'}, localGet(cfg, {'RUN','run_stamp'}, ''));
    metadata.timestamp = char(nowLocal);
    metadata.source_repository = localGit(rootDir, 'remote get-url origin');
    metadata.branch = localGit(rootDir, 'branch --show-current');
    metadata.git_sha = localGit(rootDir, 'rev-parse HEAD');
    gitStatus = localGit(rootDir, 'status --porcelain');
    metadata.git_dirty_status = localDirtyLabel(gitStatus);
    metadata.git_status_entries = strrep(gitStatus, newline, ' | ');
    metadata.matlab_version = version;
    metadata.optimization_toolbox_version = localToolboxVersion('Optimization Toolbox');
    metadata.reinforcement_learning_toolbox_version = ...
        localToolboxVersion('Reinforcement Learning Toolbox');
    metadata.hostname = getenv('COMPUTERNAME');
    metadata.working_directory = rootDir;

    metadata.training_mode = localGet(runInfo, {'training_mode'}, '');
    metadata.max_episodes = localGet(runInfo, {'max_episodes'}, NaN);
    metadata.random_seed = localGet(runInfo, {'random_seed'}, ...
        localGet(cfg, {'RNG_SEED'}, NaN));
    metadata.rng_algorithm = localGet(cfg, {'RNG_ALGORITHM'}, '');
    metadata.source_agent = localGet(runInfo, {'source_agent'}, '');
    metadata.run_label = localGet(runInfo, {'run_label'}, '');
    metadata.start_time = localGet(runInfo, {'start_time'}, '');
    metadata.end_time = localGet(runInfo, {'end_time'}, '');
    metadata.wall_clock_seconds = localGet(runInfo, {'wall_clock_seconds'}, NaN);

    metadata.actor_learning_rate = localGet(cfg, {'TRAIN','actor_learn_rate'}, NaN);
    metadata.critic_learning_rate = localGet(cfg, {'TRAIN','critic_learn_rate'}, NaN);
    metadata.discount_factor = localGet(cfg, {'TRAIN','discount_factor'}, NaN);
    metadata.target_smooth_factor = localGet(cfg, {'TRAIN','target_smooth_factor'}, NaN);
    metadata.mini_batch_size = localGet(cfg, {'TRAIN','mini_batch_size'}, NaN);
    metadata.experience_buffer_length = ...
        localGet(cfg, {'TRAIN','experience_buffer_length'}, NaN);
    metadata.noise_variance = localGet(runInfo, {'noise_variance'}, ...
        localGet(cfg, {'TRAIN','noise_variance'}, NaN));
    metadata.noise_standard_deviation = sqrt(max(metadata.noise_variance, 0));
    metadata.noise_decay = localGet(runInfo, {'noise_decay'}, ...
        localGet(cfg, {'TRAIN','noise_decay'}, NaN));

    metadata.mission_target_m = localGet(cfg, {'MISSION','D_TARGET_M'}, NaN);
    metadata.mission_duration_s = localGet(cfg, {'MISSION_DURATION'}, NaN);
    metadata.chunk_duration_s = localGet(cfg, {'CHUNK_DURATION'}, NaN);
    metadata.chunks_per_decision = localGet(cfg, {'APPLY_EVERY'}, NaN);
    metadata.decisions_per_episode = localGet(cfg, {'EP_STEPS'}, NaN);
    metadata.action_lower_bound = localActionBounds(cfg, 'lower');
    metadata.action_upper_bound = localActionBounds(cfg, 'upper');
    metadata.dr_max = localGet(cfg, {'DR_MAX'}, NaN);
    metadata.d_gamma_v_max = localGet(cfg, {'DGAMMA_V_MAX'}, NaN);

    metadata.battery_metric_type = localGet(cfg, {'BATTERY','metric_type'}, '');
    metadata.battery_metric_initial = localGet(cfg, {'BATTERY','metric_init'}, NaN);
    metadata.battery_metric_minimum = localGet(cfg, {'BATTERY','metric_min'}, NaN);
    metadata.battery_metric_maximum = localGet(cfg, {'BATTERY','metric_max'}, NaN);
    metadata.battery_initial_soc = localGet(cfg, {'BATTERY','SOC_init'}, NaN);
    metadata.battery_terminal_margin = localGet(cfg, {'BATTERY','terminal_margin'}, NaN);
    metadata.battery_nominal_capacity_Ah = localGet(cfg, {'BATTERY','C_nom_Ah'}, NaN);
    metadata.battery_pack_voltage_V = localGet(cfg, {'BATTERY','pack_voltage'}, NaN);
    metadata.battery_depth_of_discharge = localGet(cfg, {'BATTERY','DoD'}, NaN);
    metadata.battery_pack_sizing_enabled = ...
        localGet(cfg, {'BATTERY','use_pack_sizing'}, NaN);
    metadata.battery_series = localGet(cfg, {'BATTERY','n_series'}, NaN);
    metadata.battery_parallel = localGet(cfg, {'BATTERY','n_parallel'}, NaN);
    metadata.battery_decimation = localGet(cfg, {'BATTERY','decim'}, NaN);

    metadata.experiment_id = localGet(cfg, {'EXPERIMENT','id'}, '');
    metadata.experiment_description = localGet(cfg, {'EXPERIMENT','description'}, '');
    metadata.reward_version = localGet(cfg, {'REWARD','version'}, '');
    metadata.reward_w_pace = localGet(cfg, {'REWARD','w_pace'}, NaN);
    metadata.reward_w_shortfall = localGet(cfg, {'REWARD','w_shortfall'}, NaN);
    metadata.reward_w_ahead = localGet(cfg, {'REWARD','w_ahead'}, NaN);
    metadata.reward_w_lag_linear = localGet(cfg, {'REWARD','w_lag_linear'}, NaN);
    metadata.reward_w_lag_quadratic = localGet(cfg, {'REWARD','w_lag_quad'}, NaN);
    metadata.reward_w_risk = localGet(cfg, {'REWARD','w_risk'}, NaN);
    metadata.reward_complete_bonus = localGet(cfg, {'REWARD','complete_bonus'}, NaN);
    metadata.reward_early_bonus = localGet(cfg, {'REWARD','early_bonus'}, NaN);
    metadata.reward_final_soc_bonus = localGet(cfg, {'REWARD','final_soc_bonus'}, NaN);
    metadata.reward_alpha_state = localGet(cfg, {'REWARD','alpha_state'}, NaN);
    metadata.reward_alpha_speed_state = ...
        localGet(cfg, {'REWARD','alpha_speed_state'}, NaN);
    metadata.reward_mission_guard_weight = ...
        localGet(cfg, {'REWARD','w_v_shortfall'}, NaN);
    metadata.reward_adaptive_enabled = localGet(cfg, {'REWARD','ADAPT','enable'}, NaN);
end

function bounds = localActionBounds(cfg, side)
    if strcmp(side, 'lower')
        bounds = [-cfg.DR_MAX; -cfg.DR_MAX; -cfg.DR_MAX; ...
            cfg.GAMMA_V_MIN; cfg.GAMMA_A_MIN].';
    else
        bounds = [cfg.DR_MAX; cfg.DR_MAX; cfg.DR_MAX; ...
            cfg.GAMMA_V_MAX; cfg.GAMMA_A_MAX].';
    end
end

function value = localGet(s, fields, defaultValue)
    value = defaultValue;
    current = s;
    for i = 1:numel(fields)
        field = fields{i};
        if ~isstruct(current) || ~isfield(current, field)
            return
        end
        current = current.(field);
    end
    value = current;
end

function value = localGit(rootDir, arguments)
    command = sprintf('git -C "%s" %s', rootDir, arguments);
    [status, output] = system(command);
    if status == 0
        value = strtrim(output);
    else
        value = sprintf('<unavailable: %s>', strtrim(output));
    end
end

function label = localDirtyLabel(gitStatus)
    if isempty(strtrim(gitStatus))
        label = 'clean';
    else
        label = 'dirty';
    end
end

function toolboxVersion = localToolboxVersion(toolboxName)
    products = ver;
    index = find(strcmp({products.Name}, toolboxName), 1);
    if isempty(index)
        toolboxVersion = '<not installed>';
    else
        toolboxVersion = products(index).Version;
    end
end

function localPrintValue(fid, key, value)
    if isnumeric(value) || islogical(value)
        if isscalar(value)
            fprintf(fid, '%s: %.12g\n', key, double(value));
        else
            fprintf(fid, '%s: %s\n', key, mat2str(value, 12));
        end
        return
    end

    if isstring(value)
        value = char(join(value, ', '));
    elseif ~ischar(value)
        value = char(string(value));
    end
    fprintf(fid, '%s: %s\n', key, value);
end
