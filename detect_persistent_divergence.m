function result = detect_persistent_divergence(time, values, baseline, thresholds, persistenceSamples)
%detect_persistent_divergence Find the first sustained multivariable separation.

    time = time(:);
    if size(values, 1) ~= numel(time)
        error('detect_persistent_divergence:RowCount', ...
            'Values must have one row per timestamp.');
    end
    baseline = reshape(baseline, 1, []);
    thresholds = reshape(thresholds, 1, []);
    if size(values, 2) ~= numel(baseline) || ...
            numel(baseline) ~= numel(thresholds)
        error('detect_persistent_divergence:ColumnCount', ...
            'Baseline and threshold widths must match values.');
    end
    if nargin < 5 || isempty(persistenceSamples)
        persistenceSamples = 1;
    end

    standardizedExcess = (values - baseline) ./ thresholds;
    separated = standardizedExcess > 1;
    anySeparated = any(separated, 2);
    runStarts = find(conv(double(anySeparated), ...
        ones(persistenceSamples, 1), 'valid') >= persistenceSamples, 1);

    result = struct('found', ~isempty(runStarts), ...
        'index', NaN, 'time', NaN, 'variable_index', NaN, ...
        'standardized_excess', standardizedExcess, ...
        'separated', separated, 'left_censored', false);
    if isempty(runStarts)
        return
    end
    result.index = runStarts;
    result.time = time(runStarts);
    [~, result.variable_index] = max(standardizedExcess(runStarts, :));
    result.left_censored = runStarts == 1;
end
