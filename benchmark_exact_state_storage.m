function result = benchmark_exact_state_storage(sourceFile)
%benchmark_exact_state_storage Compare lossless layouts on identical saved rows.
% The large block repeats eight recorded rows for serialization measurement;
% it is not a new trajectory or additional independent experimental evidence.
    original = read_exact_state_rows(sourceFile);
    folder = tempname;
    mkdir(folder);
    cleanup = onCleanup(@() rmdir(folder,'s'));
    counts = [numel(original),256];
    measurements = zeros(6,6);
    index = 0;
    for count = counts
        rows = original(mod(0:count-1,numel(original))+1);
        rows = rows(:);
        for repetition = 1:3
            plainFile = fullfile(folder,sprintf('plain_%d_%d.mat',count,repetition));
            packedFile = fullfile(folder,sprintf('packed_%d_%d.mat',count,repetition));
            timer = tic;
            save(plainFile,'rows','-v7.3');
            plainSeconds = toc(timer);
            timer = tic;
            packed_rows = pack_exact_state_rows(rows);
            save(packedFile,'packed_rows','-v7.3');
            packedSeconds = toc(timer);
            restored = read_exact_state_rows(packedFile);
            assert(isequaln(localComparable(restored),localComparable(rows)), ...
                'benchmark_exact_state_storage:Mismatch','Packed file changed a row field.');
            plainInfo = dir(plainFile);
            packedInfo = dir(packedFile);
            index = index+1;
            measurements(index,:) = [count,repetition,plainSeconds,packedSeconds, ...
                plainInfo.bytes,packedInfo.bytes];
        end
    end
    result = array2table(measurements,'VariableNames', ...
        {'rows','repetition','plain_write_s','pack_and_write_s','plain_bytes','packed_bytes'});
    disp(result);
end

function rows = localComparable(rows)
    for k = 1:numel(rows)
        rows{k}.solver = localSolverValues(rows{k}.solver);
    end
end

function solver = localSolverValues(solver)
    options = solver.solver_options;
    if isobject(options)
        names = properties(options);
        values = struct('class',class(options));
        for k = 1:numel(names)
            values.(names{k}) = options.(names{k});
        end
        solver.solver_options = values;
    end
    if isfield(solver,'primary_attempt') && ~isempty(fieldnames(solver.primary_attempt))
        solver.primary_attempt = localSolverValues(solver.primary_attempt);
    end
end
