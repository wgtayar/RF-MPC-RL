classdef ExactStateDataset < handle
    %ExactStateDataset Append-only, chunked MAT v7.3 controller evidence.
    % A segment begins with a full replay state and ends with a full state.
    % Intermediate rows omit growing current arrays; recorded samples and the
    % segment snapshot reconstruct them without quadratic per-step storage.
    properties (SetAccess = private)
        Root
        Manifest
        RowCount = 0
        LoggingSeconds = 0
        Closed = false
    end
    properties (Access = private)
        Buffer = cell(256,1)
        Count = 0
        Segment = 0
        Context
        SnapshotReference
        Index = cell(0,6)
        PreviousRow = []
        AuditInterval = 100
    end
    methods
        function obj = ExactStateDataset(root, cfg, metadata)
            required = {'run_id','run_type','monitor_root','seed', ...
                'observation_schema_version','reward_version','solver_strategy','source_policy'};
            if ~all(isfield(metadata, required))
                error('ExactStateDataset:MissingProvenance', 'Explicit run/schema/policy provenance is required.');
            end
            if isfolder(root) || isfile(root)
                error('ExactStateDataset:Exists', 'Refusing to overwrite dataset %s.', root);
            end
            obj.Root = char(root);
            mkdir(obj.Root);
            [~, attributes] = fileattrib(obj.Root);
            obj.Root = attributes.Name;
            folders = {'manifest','mpc_steps','snapshots','failures','checksums'};
            for k = 1:numel(folders)
                mkdir(fullfile(obj.Root, folders{k}));
            end
            configPath = fullfile(obj.Root, 'manifest', 'configuration.mat');
            save(configPath, 'cfg', '-v7.3');
            source = fileparts(mfilename('fullpath'));
            manifest = metadata;
            manifest.dataset_schema_version = 'dynamic_state_v2';
            manifest.snapshot_schema_version = 'state_snapshot_v2_exact';
            manifest.action_schema_version = 'action_v2_rate_limited';
            manifest.timestamp_utc = char(datetime('now','TimeZone','UTC', ...
                'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX'));
            manifest.source_branch = localGit(source, 'branch --show-current');
            manifest.source_sha = localGit(source, 'rev-parse HEAD');
            manifest.source_status = localGit(source, 'status --porcelain');
            manifest.source_dirty = ~isempty(manifest.source_status);
            manifest.monitor_sha = localGit(metadata.monitor_root, 'rev-parse HEAD');
            manifest.matlab_version = version;
            manifest.toolboxes = ver;
            manifest.hostname = getenv('COMPUTERNAME');
            manifest.configuration_sha256 = sha256_file(configPath);
            manifest.qp_audit_interval_steps = obj.AuditInterval;
            manifest.buffer_capacity_rows = numel(obj.Buffer);
            manifest.configuration_hash_kind = 'sha256_of_archived_mat_file_bytes';
            obj.Manifest = manifest;
            save(fullfile(obj.Root,'manifest','run.mat'), 'manifest', '-v7.3');
        end
        function beginSegment(obj, state, context, parameters, control)
            obj.requireOpen();
            if ~all(isfield(context, {'episode','decision','chunk'}))
                error('ExactStateDataset:MissingIndex', 'Episode, decision and chunk are required.');
            end
            validateattributes([context.episode,context.decision,context.chunk], ...
                {'numeric'}, {'integer','positive','numel',3});
            if ~isfield(control, 'action_execution')
                error('ExactStateDataset:MissingAction', 'Capture requires the complete candidate/applied transform.');
            end
            obj.flush();
            obj.Segment = obj.Segment+1;
            obj.Context = context;
            relative = sprintf('snapshots/segment_%06d_before.mat', obj.Segment);
            snapshot = struct('state',state,'context',context,'parameters',parameters, ...
                'control',control,'configuration_sha256',obj.Manifest.configuration_sha256);
            save(fullfile(obj.Root,relative), 'snapshot', '-v7.3');
            obj.SnapshotReference = struct('file',relative, ...
                'sha256',sha256_file(fullfile(obj.Root,relative)));
        end
        function append(obj, row, problem)
            obj.requireOpen();
            timer = tic;
            if obj.Segment == 0
                error('ExactStateDataset:NoSegment', 'Call beginSegment before append.');
            end
            row.schema_version = 'dynamic_state_v2';
            row.row_id = obj.RowCount+1;
            row.segment = obj.Segment;
            row.context = obj.Context;
            row.snapshot_reference = obj.SnapshotReference;
            row.configuration_sha256 = obj.Manifest.configuration_sha256;
            row.source_sha = obj.Manifest.source_sha;
            validate_exact_state_row(row, obj.PreviousRow);
            saveQP = ~row.solver.success || row.dynamic_event || ...
                row.iteration == 1 || mod(row.row_id-1,obj.AuditInterval) == 0;
            row.qp_reference = struct('file','','sha256','');
            if saveQP
                relative = sprintf('failures/qp_row_%09d.mat', row.row_id);
                exactQP = struct('problem',problem,'solver',row.solver, ...
                    'row_id',row.row_id,'source_sha',row.source_sha, ...
                    'configuration_sha256',row.configuration_sha256, ...
                    'Xt',row.Xt_qp,'Ut',row.Ut_qp,'Xd',row.Xd,'Ud',row.Ud);
                save(fullfile(obj.Root,relative), 'exactQP', '-v7.3');
                row.qp_reference = struct('file',relative, ...
                    'sha256',sha256_file(fullfile(obj.Root,relative)));
            end
            obj.Count = obj.Count+1;
            obj.Buffer{obj.Count} = row;
            obj.RowCount = row.row_id;
            obj.PreviousRow = row;
            if obj.Count == numel(obj.Buffer)
                obj.flush();
            end
            obj.LoggingSeconds = obj.LoggingSeconds+toc(timer);
        end
        function endSegment(obj, state, outcome)
            obj.requireOpen();
            obj.flush();
            snapshot = struct('state',state,'outcome',outcome,'context',obj.Context);
            filename = sprintf('segment_%06d_after.mat',obj.Segment);
            save(fullfile(obj.Root,'snapshots',filename), 'snapshot', '-v7.3');
        end
        function flush(obj)
            if obj.Count == 0
                return
            end
            rows = obj.Buffer(1:obj.Count);
            first = rows{1}.row_id;
            last = rows{end}.row_id;
            relative = sprintf('mpc_steps/rows_%09d_%09d.mat',first,last);
            target = fullfile(obj.Root,relative);
            if isfile(target)
                error('ExactStateDataset:Exists', 'Refusing to overwrite %s.',target);
            end
            save(target,'rows','-v7.3');
            info = dir(target);
            obj.Index(end+1,:) = {relative,first,last,obj.Count,info.bytes,sha256_file(target)};
            index = cell2table(obj.Index,'VariableNames', ...
                {'file','first_row','last_row','row_count','bytes','sha256'});
            writetable(index,fullfile(obj.Root,'mpc_steps','index.csv'));
            obj.Buffer(:) = {[]};
            obj.Count = 0;
        end
        function finish(obj, status)
            obj.requireOpen();
            obj.flush();
            completion = struct('status',status,'rows',obj.RowCount, ...
                'segments',obj.Segment,'append_logging_seconds',obj.LoggingSeconds);
            save(fullfile(obj.Root,'manifest','completion.mat'),'completion');
            files = dir(fullfile(obj.Root,'**','*'));
            checksums = cell(0,3);
            for k = 1:numel(files)
                if ~files(k).isdir
                    filename = fullfile(files(k).folder,files(k).name);
                    relative = filename(numel(obj.Root)+2:end);
                    checksums(end+1,:) = {relative,files(k).bytes,sha256_file(filename)}; %#ok<AGROW>
                end
            end
            writetable(cell2table(checksums,'VariableNames',{'file','bytes','sha256'}), ...
                fullfile(obj.Root,'checksums','files.csv'));
            obj.Closed = true;
        end
    end
    methods (Access = private)
        function requireOpen(obj)
            if obj.Closed
                error('ExactStateDataset:Closed', 'This dataset is finalized.');
            end
        end
    end
end

function value = localGit(root, command)
    [status, value] = system(sprintf('git -C "%s" %s',root,command));
    if status ~= 0
        error('ExactStateDataset:GitUnavailable','Cannot collect Git provenance: %s',value);
    end
    value = strtrim(value);
end
