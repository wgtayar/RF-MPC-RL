function report = validate_exact_state_dataset(root)
%validate_exact_state_dataset Validate hashes, provenance and MPC continuity.
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
    [report.supervisory_records,report.supervisory_decisions_complete] = ...
        localValidateEvents(root,manifest,decisionRanges,verifiedHashes);
    if isfield(manifest,'supervisory_records_required') && manifest.supervisory_records_required && ...
            ~strcmp(saved.completion.status,'environment_exception')
        assert(report.supervisory_decisions_complete, ...
            'validate_exact_state_dataset:MissingDecision','A required supervisory transition is absent.');
    end
end

function [count,complete] = localValidateEvents(root, manifest, ranges, hashes)
    files = [dir(fullfile(root,'episodes','*.mat'));dir(fullfile(root,'decisions','*.mat'))];
    count = numel(files);
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
