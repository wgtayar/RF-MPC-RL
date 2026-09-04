function [failureQpId, failureQpFile] = save_qp_failure_snapshot(cfg, context, failureQP)
%save_qp_failure_snapshot Save one reconstructable QP failure artifact.

    failureQpId = "";
    failureQpFile = "";
    if ~isfield(cfg, 'DIAGNOSTICS') || ~cfg.DIAGNOSTICS.enable || ...
            ~cfg.DIAGNOSTICS.save_full_qp_on_failure || ...
            ~isfield(cfg, 'RUN') || ~cfg.RUN.enabled || ...
            ~isfield(cfg.RUN, 'qp_failure_dir')
        return
    end

    outputFolder = cfg.RUN.qp_failure_dir;
    if ~isfolder(outputFolder)
        mkdir(outputFolder);
    end

    episode = localGet(context, 'episode_idx', 0);
    decision = localGet(context, 'decision_idx', 0);
    chunk = localGet(context, 'chunk_in_decision', 0);
    iteration = localGet(failureQP, 'fail_iter', 0);
    failureQpId = string(sprintf('ep%04d_dec%02d_chunk%02d_iter%04d', ...
        episode, decision, chunk, iteration));
    failureQpFile = string(fullfile(outputFolder, failureQpId + ".mat"));

    if isfield(failureQP, 'fsm_internal_state')
        failureQP.schema_version = 'phase2_qp_failure_v2';
    else
        failureQP.schema_version = 'phase1_qp_failure_v1';
    end
    failureQP.failure_qp_id = failureQpId;
    failureQP.saved_at = char(datetime('now', 'TimeZone', 'local', ...
        'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX'));
    failureQP.context = context;
    save(failureQpFile, 'failureQP', '-v7.3');
end

function value = localGet(s, field, defaultValue)
    if isstruct(s) && isfield(s, field) && ~isempty(s.(field)) && isfinite(s.(field))
        value = s.(field);
    else
        value = defaultValue;
    end
end
