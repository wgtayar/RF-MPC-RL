function diagnostics = compute_qp_diagnostics( ...
        z, Aineq, bineq, Aeq, beq, activeTolerance, nearActiveTolerance)
%compute_qp_diagnostics Compute residuals and physical force margins for a QP.

    if nargin < 6 || isempty(activeTolerance)
        activeTolerance = 1e-7;
    end
    if nargin < 7 || isempty(nearActiveTolerance)
        nearActiveTolerance = 1e-5;
    end

    z = z(:);
    inequalityMargin = bineq - Aineq*z;
    equalityResidual = Aeq*z - beq;

    diagnostics = struct();
    diagnostics.inequality_margin_min = min(inequalityMargin, [], 'omitnan');
    diagnostics.inequality_margin_max = max(inequalityMargin, [], 'omitnan');
    diagnostics.active_inequality_count = nnz(inequalityMargin <= activeTolerance);
    diagnostics.near_active_inequality_count = nnz(inequalityMargin <= nearActiveTolerance);
    diagnostics.violated_inequality_count = nnz(inequalityMargin < -nearActiveTolerance);
    diagnostics.equality_residual_norm = norm(equalityResidual);
    diagnostics.equality_residual_max_abs = max(abs(equalityResidual), [], 'omitnan');
    diagnostics.inequality_margins = inequalityMargin;
    diagnostics.equality_residual = equalityResidual;
    diagnostics.constraint_type_names = [
        "Fx_minus_muFz"
        "minusFx_minus_muFz"
        "Fy_minus_muFz"
        "minusFy_minus_muFz"
        "Fz_upper"
        "Fz_lower"
    ];

    rowsPerHorizon = 4 * 6;
    if mod(numel(inequalityMargin), rowsPerHorizon) == 0
        predictionHorizon = numel(inequalityMargin) / rowsPerHorizon;
        marginCube = reshape(inequalityMargin, 6, 4, predictionHorizon);
        diagnostics.margin_by_type_leg_horizon = marginCube;
        diagnostics.minimum_margin_by_leg_type = ...
            squeeze(min(marginCube, [], 3)).';
    else
        diagnostics.margin_by_type_leg_horizon = [];
        diagnostics.minimum_margin_by_leg_type = nan(4, 6);
    end
end
