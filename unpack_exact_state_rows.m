function rows = unpack_exact_state_rows(packed)
%unpack_exact_state_rows Invert packed_recursive_columns_v1 without rounding.
    assert(strcmp(packed.layout,'packed_recursive_columns_v1'), ...
        'unpack_exact_state_rows:Layout','Unsupported exact-state storage layout.');
    rows = localUnpack(packed.tree,packed.count);
end

function values = localUnpack(node,count)
    switch node.kind
        case 'numeric'
            values = cell(count,1);
            for k = 1:count
                values{k} = reshape(node.data(:,k),node.shape);
            end
        case 'struct'
            values = repmat({struct()},count,1);
            fields = fieldnames(node.children);
            for k = 1:numel(fields)
                name = fields{k};
                children = localUnpack(node.children.(name),count);
                for j = 1:count
                    values{j}.(name) = children{j};
                end
            end
        case 'constant'
            values = repmat({node.value},count,1);
        case 'cell'
            values = node.values;
        otherwise
            error('unpack_exact_state_rows:Node','Unsupported packed field type.');
    end
end
