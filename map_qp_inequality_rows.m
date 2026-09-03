function mapping = map_qp_inequality_rows(rowIndices)
%map_qp_inequality_rows Map RF-MPC inequality rows to horizon, leg, and force side.

    rowIndices = rowIndices(:);
    validateattributes(rowIndices, {'numeric'}, {'integer', 'positive'});
    rowsPerLeg = 6;
    legsPerHorizon = 4;
    rowsPerHorizon = rowsPerLeg * legsPerHorizon;

    zeroBased = rowIndices - 1;
    horizonStep = floor(zeroBased / rowsPerHorizon) + 1;
    withinHorizon = mod(zeroBased, rowsPerHorizon);
    leg = floor(withinHorizon / rowsPerLeg) + 1;
    constraintIndex = mod(withinHorizon, rowsPerLeg) + 1;

    names = [
        "Fx - mu*Fz <= 0"
        "-Fx - mu*Fz <= 0"
        "Fy - mu*Fz <= 0"
        "-Fy - mu*Fz <= 0"
        "Fz <= Ud_z + ub"
        "Fz >= Ud_z + lb"
    ];
    constraint = names(constraintIndex);
    mapping = table(rowIndices, horizonStep, leg, constraintIndex, constraint, ...
        'VariableNames', {'row', 'horizon_step', 'leg', ...
        'constraint_index', 'physical_constraint'});
end
