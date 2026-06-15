function write_run_manifest(manifestPath, cfg, trainMode, sourceAgentPath)
    if nargin < 4
        sourceAgentPath = '';
    end

    fid = fopen(manifestPath, 'w');

    if fid < 0
        warning('Could not write run manifest: %s', manifestPath);
        return
    end

    cleanupObj = onCleanup(@() fclose(fid));

    fprintf(fid, 'train_mode: %s\n', trainMode);
    fprintf(fid, 'source_agent: %s\n', sourceAgentPath);

    localPrintText(fid, 'experiment_id', ...
        localGet(cfg, {'EXPERIMENT','id'}, ''));

    localPrintText(fid, 'experiment_description', ...
        localGet(cfg, {'EXPERIMENT','description'}, ''));

    localPrintText(fid, 'reward_version', ...
        localGet(cfg, {'REWARD','version'}, ''));

    localPrintText(fid, 'reward_description', ...
        localGet(cfg, {'REWARD','description'}, ''));

    localPrintNum(fid, 'mission_target_m', ...
        localGet(cfg, {'MISSION','D_TARGET_M'}, NaN));

    localPrintNum(fid, 'mission_window_target_m', ...
        localGet(cfg, {'MISSION','WINDOW_TARGET_M'}, NaN));

    localPrintNum(fid, 'mission_duration_s', ...
        localGet(cfg, {'MISSION_DURATION'}, NaN));

    localPrintNum(fid, 'episode_steps', ...
        localGet(cfg, {'EP_STEPS'}, NaN));

    localPrintNum(fid, 'chunk_duration_s', ...
        localGet(cfg, {'CHUNK_DURATION'}, NaN));

    localPrintNum(fid, 'apply_every', ...
        localGet(cfg, {'APPLY_EVERY'}, NaN));

    localPrintNum(fid, 'gamma_v_min', ...
        localGet(cfg, {'GAMMA_V_MIN'}, NaN));

    localPrintNum(fid, 'gamma_v_max', ...
        localGet(cfg, {'GAMMA_V_MAX'}, NaN));

    localPrintNum(fid, 'v_min', ...
        localGet(cfg, {'V_MIN'}, NaN));

    localPrintNum(fid, 'v_max', ...
        localGet(cfg, {'V_MAX'}, NaN));

    if isfield(cfg, 'V_MIN') && isfield(cfg, 'V_MAX') && isfield(cfg, 'GAMMA_V_MAX')
        v_exec_cap = cfg.V_MIN + cfg.GAMMA_V_MAX * (cfg.V_MAX - cfg.V_MIN);
    else
        v_exec_cap = NaN;
    end
    localPrintNum(fid, 'v_exec_cap', v_exec_cap);

    localPrintNum(fid, 'gamma_a_min', ...
        localGet(cfg, {'GAMMA_A_MIN'}, NaN));

    localPrintNum(fid, 'gamma_a_max', ...
        localGet(cfg, {'GAMMA_A_MAX'}, NaN));

    localPrintNum(fid, 'dr_max', ...
        localGet(cfg, {'DR_MAX'}, NaN));

    localPrintNum(fid, 'battery_parallel', ...
        localGet(cfg, {'BATTERY','n_parallel'}, NaN));

    localPrintNum(fid, 'battery_terminal_margin', ...
        localGet(cfg, {'BATTERY','terminal_margin'}, NaN));

    localPrintNum(fid, 'reward_w_pace', ...
        localGet(cfg, {'REWARD','w_pace'}, NaN));

    localPrintNum(fid, 'reward_w_shortfall', ...
        localGet(cfg, {'REWARD','w_shortfall'}, NaN));

    localPrintNum(fid, 'reward_w_ahead', ...
        localGet(cfg, {'REWARD','w_ahead'}, NaN));

    localPrintNum(fid, 'reward_w_risk', ...
        localGet(cfg, {'REWARD','w_risk'}, NaN));

    localPrintNum(fid, 'reward_final_soc_bonus', ...
        localGet(cfg, {'REWARD','final_soc_bonus'}, NaN));

    localPrintNum(fid, 'reward_complete_bonus', ...
        localGet(cfg, {'REWARD','complete_bonus'}, NaN));

    localPrintNum(fid, 'reward_early_bonus', ...
        localGet(cfg, {'REWARD','early_bonus'}, NaN));

    localPrintNum(fid, 'adapt_enable', ...
        localGet(cfg, {'REWARD','ADAPT','enable'}, NaN));

    localPrintNum(fid, 'adapt_w_efficiency_bonus', ...
        localGet(cfg, {'REWARD','ADAPT','w_efficiency_bonus'}, NaN));

    localPrintNum(fid, 'adapt_w_excess_speed', ...
        localGet(cfg, {'REWARD','ADAPT','w_excess_speed'}, NaN));

    localPrintNum(fid, 'adapt_w_cap_use', ...
        localGet(cfg, {'REWARD','ADAPT','w_cap_use'}, NaN));

    localPrintNum(fid, 'adapt_w_current_conserve', ...
        localGet(cfg, {'REWARD','ADAPT','w_current_conserve'}, NaN));

    localPrintNum(fid, 'adapt_w_terminal_soc', ...
        localGet(cfg, {'REWARD','ADAPT','w_terminal_soc'}, NaN));

    localPrintNum(fid, 'adapt_I_conserve_high', ...
        localGet(cfg, {'REWARD','ADAPT','I_conserve_high'}, NaN));

    localPrintNum(fid, 'adapt_I_conserve_low', ...
        localGet(cfg, {'REWARD','ADAPT','I_conserve_low'}, NaN));

    localPrintNum(fid, 'adapt_I_conserve_scale', ...
        localGet(cfg, {'REWARD','ADAPT','I_conserve_scale'}, NaN));
end

function val = localGet(s, fields, defaultVal)
    val = defaultVal;
    cur = s;

    for i = 1:numel(fields)
        f = fields{i};

        if isstruct(cur) && isfield(cur, f)
            cur = cur.(f);
        else
            return
        end
    end

    val = cur;
end

function localPrintText(fid, key, val)
    if isstring(val)
        val = char(val);
    end

    if isempty(val)
        val = '';
    end

    if ~ischar(val)
        try
            val = char(string(val));
        catch
            val = '<unprintable>';
        end
    end

    fprintf(fid, '%s: %s\n', key, val);
end

function localPrintNum(fid, key, val)
    if isnumeric(val) || islogical(val)
        if isscalar(val)
            fprintf(fid, '%s: %.10g\n', key, double(val));
        else
            fprintf(fid, '%s: %s\n', key, mat2str(val));
        end
    else
        localPrintText(fid, key, val);
    end
end