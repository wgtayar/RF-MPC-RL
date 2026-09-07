function [state, control, parameters] = restore_exact_dataset_state(root, rowId)
%restore_exact_dataset_state Restore the state BEFORE FSM processing at rowId.
    validateattributes(rowId,{'numeric'},{'scalar','positive','integer'});
    validate_exact_state_dataset(root);
    index = readtable(fullfile(root,'mpc_steps','index.csv'),'TextType','string');
    fileIndex = find(index.first_row <= rowId & index.last_row >= rowId,1);
    if isempty(fileIndex)
        error('restore_exact_dataset_state:MissingRow','Requested MPC row does not exist.');
    end
    data = load(fullfile(root,index.file(fileIndex)),'rows');
    target = data.rows{rowId-index.first_row(fileIndex)+1};
    saved = load(fullfile(root,target.snapshot_reference.file),'snapshot');
    state = saved.snapshot.state;
    control = saved.snapshot.control;
    parameters = saved.snapshot.parameters;
    for k = 1:fileIndex
        data = load(fullfile(root,index.file(k)),'rows');
        for j = 1:numel(data.rows)
            row = data.rows{j};
            if row.segment == target.segment && row.row_id < rowId && row.current_sample_committed
                state.current_time(end+1,1) = row.current_sample_time;
                state.current_total(end+1,1) = row.current_sample_A;
            end
        end
    end
    fields = setdiff(fieldnames(target.state_before),{'current_sample_count'});
    for k = 1:numel(fields)
        state.(fields{k}) = target.state_before.(fields{k});
    end
    assert(numel(state.current_time) == target.state_before.current_sample_count, ...
        'restore_exact_dataset_state:CurrentHistory','Accumulated current history is incomplete.');
end
