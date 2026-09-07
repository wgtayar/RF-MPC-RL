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
