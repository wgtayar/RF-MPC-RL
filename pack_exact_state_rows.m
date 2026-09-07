function packed = pack_exact_state_rows(rows)
%pack_exact_state_rows Store nested numeric fields as columns rather than objects.
    assert(iscell(rows) && ~isempty(rows),'pack_exact_state_rows:Rows', ...
        'Supply a nonempty cell array of row records.');
    packed = struct('layout','packed_recursive_columns_v1', ...
        'count',numel(rows),'tree',localPack(rows(:)));
end

function node = localPack(values)
    first = values{1};
    if (isnumeric(first) || islogical(first)) && ...
            all(cellfun(@(v) strcmp(class(v),class(first)) && isequal(size(v),size(first)),values))
        columns = cellfun(@(v) v(:),values,'UniformOutput',false);
        node = struct('kind','numeric','shape',size(first),'data',cat(2,columns{:}));
    elseif isstruct(first) && isscalar(first) && ...
            all(cellfun(@(v) isstruct(v) && isscalar(v) && ...
            isequal(fieldnames(v),fieldnames(first)),values))
        fields = fieldnames(first);
        children = struct();
        for k = 1:numel(fields)
            name = fields{k};
            childValues = cellfun(@(v) v.(name),values,'UniformOutput',false);
            children.(name) = localPack(childValues);
        end
        node = struct('kind','struct','children',children);
    elseif all(cellfun(@(v) isequaln(v,first),values))
        node = struct('kind','constant','value',{first});
    else
        node = struct('kind','cell','values',{values});
    end
end
