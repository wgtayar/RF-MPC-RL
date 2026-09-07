function rows = read_exact_state_rows(filename)
%read_exact_state_rows Read original or packed v2 artifacts without migration.
    data = load(filename);
    if isfield(data,'packed_rows')
        rows = unpack_exact_state_rows(data.packed_rows);
    elseif isfield(data,'rows')
        rows = data.rows;
    else
        error('read_exact_state_rows:MissingRows','No recognized MPC rows in %s.',filename);
    end
end
