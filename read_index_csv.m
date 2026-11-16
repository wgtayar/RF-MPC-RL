function table = read_index_csv(path, names, types)
%READ_INDEX_CSV Read a comma-separated index with a known header.
% readtable's automatic detection misread a large checksums/files.csv with
% Windows paths (lab PC pilot, 2026-11-13: variables Var1..Var4). The
% delimiter, header and column types are therefore fixed here, and the
% header line must match the expected names exactly.
fid = fopen(path, 'r');
assert(fid > 0, 'read_index_csv:Missing', 'Cannot open %s.', path);
header = strtrim(fgetl(fid));
fclose(fid);
assert(strcmp(header, strjoin(names, ',')), 'read_index_csv:Header', ...
    'Unexpected header in %s: %s', path, header);
options = delimitedTextImportOptions('NumVariables', numel(names), ...
    'VariableNames', names, 'VariableTypes', types, 'Delimiter', ',', ...
    'DataLines', [2 Inf], 'ExtraColumnsRule', 'error', 'EmptyLineRule', 'skip');
table = readtable(path, options);
end
