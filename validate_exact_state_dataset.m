function report = validate_exact_state_dataset(root,reconstructWindows)
%validate_exact_state_dataset Validate hashes, provenance and MPC continuity.
% Candidate datasets always reconstruct windows; pass true to audit legacy
% Phase-3 supervisory captures without rewriting their historical evidence.
    if nargin<2
        reconstructWindows = false;
    end
    validateattributes(reconstructWindows,{'logical'},{'scalar'});
    [exists, attributes] = fileattrib(root);
    assert(exists, 'validate_exact_state_dataset:MissingRoot', 'Dataset root is absent.');
    root = attributes.Name;
    files = readtable(fullfile(root,'checksums','files.csv'),'TextType','string');
    verifiedHashes = containers.Map('KeyType','char','ValueType','char');
    for k = 1:height(files)
        target = localArtifact(root,files.file(k));
        assert(strcmp(sha256_file(target),files.sha256(k)), ...
            'validate_exact_state_dataset:Checksum', 'Artifact checksum mismatch.');
        verifiedHashes(char(target)) = char(files.sha256(k));
    end
    saved = load(fullfile(root,'manifest','run.mat'),'manifest');
    manifest = saved.manifest;
    assert(strcmp(sha256_file(fullfile(root,'manifest','configuration.mat')), ...
        manifest.configuration_sha256), 'validate_exact_state_dataset:ConfigHash', ...
        'Configuration bytes do not match manifest.');
    candidate = strcmp(manifest.reward_version,'reward_phase3_coverage_candidate_v1');
    reconstructWindows = reconstructWindows || candidate;
    cfg = struct();
    if reconstructWindows
        configuration = load(fullfile(root,'manifest','configuration.mat'),'cfg');
        cfg = configuration.cfg;
        assert(strcmp(cfg.PHASE3.reward_version,manifest.reward_version), ...
            'validate_exact_state_dataset:RewardVersion','Reward configuration and manifest differ.');
        if candidate
            phase3reward.validatePolicy(cfg.PHASE3.options.reward_policy,cfg);
        end
    end
    index = readtable(fullfile(root,'mpc_steps','index.csv'),'TextType','string');
    previous = [];
    count = 0;
    qpCount = 0;
    decisionRanges = containers.Map('KeyType','char','ValueType','any');
    for k = 1:height(index)
        target = localArtifact(root,index.file(k));
        assert(strcmp(sha256_file(target),index.sha256(k)), ...
            'validate_exact_state_dataset:IndexHash','Index hash mismatch.');
        rows = read_exact_state_rows(target);
        assert(numel(rows) == index.row_count(k) && ...
            rows{1}.row_id == index.first_row(k) && ...
            rows{end}.row_id == index.last_row(k), ...
            'validate_exact_state_dataset:IndexBounds','Index row bounds differ.');
        for j = 1:numel(rows)
            row = rows{j};
            key = sprintf('%d_%d',row.context.episode,row.context.decision);
            if ~isKey(decisionRanges,key)
                decisionRanges(key) = struct('first',row.row_id,'last',row.row_id, ...
                    'action',row.action_execution,'Xt',row.state_after.Xt,'Ut',row.state_after.Ut);
            else
                range = decisionRanges(key);
                assert(isequaln(range.action,row.action_execution), ...
                    'validate_exact_state_dataset:DecisionAction', ...
                    'Supervisory action changed inside one recorded decision.');
                range.last = row.row_id;
                range.Xt = row.state_after.Xt;
                range.Ut = row.state_after.Ut;
                decisionRanges(key) = range;
            end
            validate_exact_state_row(row,previous);
            assert(row.row_id == count+1, 'validate_exact_state_dataset:RowOrder', ...
                'Dataset row order is discontinuous.');
            assert(strcmp(row.source_sha,manifest.source_sha) && ...
                strcmp(row.configuration_sha256,manifest.configuration_sha256), ...
                'validate_exact_state_dataset:Provenance','Row provenance differs from manifest.');
            localReference(root,row.snapshot_reference,verifiedHashes);
            if strlength(string(row.qp_reference.file)) > 0
                target = localReference(root,row.qp_reference,verifiedHashes);
                qp = load(target,'exactQP');
                assert(qp.exactQP.row_id == row.row_id && ...
                    strcmp(qp.exactQP.source_sha,row.source_sha) && ...
                    strcmp(qp.exactQP.configuration_sha256,row.configuration_sha256) && ...
                    isequaln(qp.exactQP.Xt,row.Xt_qp) && ...
                    isequaln(qp.exactQP.Ut,row.Ut_qp) && ...
                    isequaln(qp.exactQP.Xd,row.Xd) && ...
                    isequaln(qp.exactQP.Ud,row.Ud), ...
                    'validate_exact_state_dataset:QpLink','Exact QP does not match its state row.');
                qpCount = qpCount+1;
            end
            previous = row;
            if reconstructWindows
                range = decisionRanges(key);
                if row.row_id==range.first
                    initial = load(localReference(root,row.snapshot_reference,verifiedHashes),'snapshot');
                    range.before = initial.snapshot.state;
                    range.dt = initial.snapshot.parameters.simTimeStep;
                    range.exposure = [];
                    range.sample_times = zeros(0,1);
                    range.sample_currents = zeros(0,1);
                    range.chunks = cell(0,1);
                    assert(abs(range.before.t-row.time_before)<1e-10, ...
                        'validate_exact_state_dataset:RewardStart','First row is not the decision snapshot time.');
                end
                if candidate
                    range.exposure = phase3reward.accumulate(range.exposure,row,range.dt);
                end
                if isempty(range.chunks) || range.chunks{end}.segment~=row.segment
                    initial = load(localReference(root,row.snapshot_reference,verifiedHashes),'snapshot');
                    assert(strcmp(initial.snapshot.configuration_sha256,manifest.configuration_sha256), ...
                        'validate_exact_state_dataset:ChunkConfig','Chunk snapshot configuration differs.');
                    range.chunks{end+1} = phase3reward.accumulateChunk([],row,initial.snapshot,cfg);
                else
                    range.chunks{end} = phase3reward.accumulateChunk(range.chunks{end},row,[],cfg);
                end
                range.time_after = row.time_after;
                if row.current_sample_committed
                    range.sample_times(end+1,1) = row.current_sample_time;
                    range.sample_currents(end+1,1) = row.current_sample_A;
                end
                decisionRanges(key) = range;
            end
            count = count+1;
        end
    end
    saved = load(fullfile(root,'manifest','completion.mat'),'completion');
    assert(count == saved.completion.rows, ...
        'validate_exact_state_dataset:Completion','Completion row count differs.');
    report = struct('valid',true,'rows',count,'full_qps',qpCount, ...
        'files',height(files),'source_sha',manifest.source_sha, ...
        'run_status',saved.completion.status, ...
        'restore_equivalence_tested',false);
    [report.supervisory_records,report.supervisory_decisions_complete,report.rewards_reconstructed, ...
        report.windows_reconstructed] = localValidateEvents(root,manifest,decisionRanges,verifiedHashes,cfg,reconstructWindows);
    report.window_reconstruction_version = 'captured_decision_window_reconstruction_v1';
    if isfield(manifest,'supervisory_records_required') && manifest.supervisory_records_required && ...
            ~strcmp(saved.completion.status,'environment_exception')
        assert(report.supervisory_decisions_complete, ...
            'validate_exact_state_dataset:MissingDecision','A required supervisory transition is absent.');
    end
end

function [count,complete,reconstructed,windowCount] = localValidateEvents(root, manifest, ranges, hashes,cfg,reconstructWindows)
    files = [dir(fullfile(root,'episodes','*.mat'));dir(fullfile(root,'decisions','*.mat'))];
    count = numel(files);
    reconstructed = 0;
    windowCount = 0;
    seen = containers.Map('KeyType','char','ValueType','logical');
    for k = 1:count
        path = fullfile(files(k).folder,files(k).name);
        assert(isKey(hashes,path), ...
            'validate_exact_state_dataset:EventHash','Supervisory record is not checksummed.');
        saved = load(path,'event');
        event = saved.event;
        assert(strcmp(event.source_sha,manifest.source_sha) && ...
            strcmp(event.configuration_sha256,manifest.configuration_sha256), ...
            'validate_exact_state_dataset:EventProvenance','Supervisory provenance differs.');
        if isfield(event.context,'decision')
            key = sprintf('%d_%d',event.context.episode,event.context.decision);
            assert(~isKey(seen,key), ...
                'validate_exact_state_dataset:DuplicateDecision','A supervisory decision is duplicated.');
            seen(key) = true;
            assert(isKey(ranges,key), ...
                'validate_exact_state_dataset:DecisionRows','Decision has no linked MPC rows.');
            range = ranges(key);
            record = event.record;
            assert(record.first_mpc_row == range.first && record.last_mpc_row == range.last && ...
                isequaln(record.action_execution,range.action) && ...
                isequaln(record.state.Xt,range.Xt) && isequaln(record.state.Ut,range.Ut), ...
                'validate_exact_state_dataset:DecisionLink','Decision does not match its MPC rows.');
            assert(isequaln(record.next_observation,record.state.decision_bookkeeping.observation) && ...
                isfinite(record.reward) && all(isfinite(record.observation)) && ...
                all(isfinite(record.next_observation)), ...
                'validate_exact_state_dataset:DecisionObservation','Decision observation/reward is inconsistent.');
            if reconstructWindows
                chunks = cell(size(range.chunks));
                for j = 1:numel(chunks)
                    relative = sprintf('snapshots/segment_%06d_after.mat',range.chunks{j}.segment);
                    target = localArtifact(root,relative);
                    assert(isKey(hashes,target), ...
                        'validate_exact_state_dataset:ChunkHash','Post-chunk snapshot is not checksummed.');
                    post = load(target,'snapshot');
                    assert(isequal(post.snapshot.context,range.chunks{j}.context), ...
                        'validate_exact_state_dataset:ChunkContext','Post-chunk context differs.');
                    chunks{j} = phase3reward.finishChunk(range.chunks{j},post.snapshot.state);
                end
                [window,terminal,audit] = phase3reward.reconstructWindow(chunks,cfg);
                assert(isequaln(window,record.window) && strcmp(terminal,record.terminal_reason) && ...
                    isequal(audit.is_done,record.is_done) && ...
                    isequaln(record.state.battery,chunks{end}.after.battery) && ...
                    record.state.decision_bookkeeping.distance_m==window.distance_end_m, ...
                    'validate_exact_state_dataset:WindowReconstruction','Window, terminal or endpoint bookkeeping differs from captured evidence.');
                windowCount = windowCount+1;
            end
            if strcmp(manifest.reward_version,'reward_phase3_coverage_candidate_v1')
                assert(isequal(record.state.current_time(:),[range.before.current_time(:);range.sample_times]) && ...
                    isequal(record.state.current_total(:),[range.before.current_total(:);range.sample_currents]) && ...
                    abs(record.state.t-range.time_after)<1e-10 && ...
                    strcmp(record.terminal_reason,record.window.terminal_reason), ...
                    'validate_exact_state_dataset:RewardHistory','Decision differs from captured current/time/terminal history.');
                exposure = phase3reward.finishExposure(range.exposure);
                [reward,info] = compute_phase3_reward_candidate(window,record.action_execution, ...
                    exposure,range.before,record.state,cfg,cfg.PHASE3.options.reward_policy);
                assert(isequaln(reward,record.reward) && isequaln(info,record.reward_info), ...
                    'validate_exact_state_dataset:RewardReconstruction','Candidate reward or audit differs from captured evidence.');
                reconstructed = reconstructed+1;
            end
        end
    end
    complete = seen.Count == ranges.Count;
end

function target = localReference(root, reference, verifiedHashes)
    target = localArtifact(root,reference.file);
    assert(isKey(verifiedHashes,char(target)) && ...
        strcmp(verifiedHashes(char(target)),reference.sha256), ...
        'validate_exact_state_dataset:ReferenceHash','Referenced artifact hash mismatch.');
end

function target = localArtifact(root, relative)
    [exists, attributes] = fileattrib(fullfile(root,relative));
    assert(exists, 'validate_exact_state_dataset:MissingArtifact','Referenced artifact is missing.');
    target = attributes.Name;
    assert(startsWith(string(target),string(root)+filesep,'IgnoreCase',ispc), ...
        'validate_exact_state_dataset:ExternalPath','References must stay inside the dataset.');
end
