function [parameters,noise] = resolveModelOptions(options,defaultPath)
%RESOLVEMODELOPTIONS Load replaceable cell data and configurable EKF settings.
% An absent options struct preserves the historical EKF noise constants.
% Reported RMSE is metadata, not a covariance or an injected sensor error.
    if nargin < 1 || isempty(options)
        options = struct();
    end
    assert(isstruct(options) && isscalar(options),'phase3battery:ModelOptions', ...
        'Battery model options must be a scalar struct.');
    allowed = {'parameter_source','process_covariance','measurement_variance','initial_covariance','voltage_filter'};
    assert(all(ismember(fieldnames(options),allowed)), ...
        'phase3battery:ModelOptions','Unknown battery model option.');
    source = defaultPath;
    if isfield(options,'parameter_source')
        source = options.parameter_source;
    end
    if istable(source)
        parameters = source;
    else
        assert((ischar(source) && isrow(source) && ~isempty(source)) || ...
            (isstring(source) && isscalar(source) && ~ismissing(source) && strlength(source)>0), ...
            'phase3battery:ParameterSource','Use a cell table or a nonempty workbook path.');
        parameters = readtable(source);
    end
    required = {'SOC','OCV','R0','R1','C1','R2','C2','R3','C3'};
    assert(all(ismember(required,parameters.Properties.VariableNames)), ...
        'phase3battery:Parameters','Cell table is missing required parameters.');
    values = parameters{:,required};
    assert(isnumeric(values) && isreal(values) && size(values,1)>=2 && ...
        all(isfinite(values),'all') && all(diff(values(:,1))>0) && ...
        all(values(:,1)>=0 & values(:,1)<=1) && all(values(:,2:end)>0,'all'), ...
        'phase3battery:Parameters','Require ordered SOC fractions and finite positive electrical parameters.');
    noise = struct('process_covariance',diag([1e-7 1e-3 1e-3 1e-3]), ...
        'measurement_variance',1e-3,'initial_covariance',diag([1e-4 1e-3 1e-3 1e-3]), ...
        'voltage_filter','legacy_lowpass');
    fields = fieldnames(noise);
    for k = 1:numel(fields)
        name = fields{k};
        if isfield(options,name)
            noise.(name) = options.(name);
        end
    end
    for name = {'process_covariance','initial_covariance'}
        matrix = noise.(name{1});
        assert(isnumeric(matrix) && isreal(matrix) && isequal(size(matrix),[4 4]) && ...
            all(isfinite(matrix),'all') && isequal(matrix,matrix.') && min(eig(matrix))>=0, ...
            'phase3battery:Covariance','Covariance must be finite, symmetric, positive semidefinite and 4-by-4.');
    end
    validateattributes(noise.measurement_variance,{'numeric'}, ...
        {'scalar','real','finite','positive'});
    assert((ischar(noise.voltage_filter) && isrow(noise.voltage_filter)) || ...
        (isstring(noise.voltage_filter) && isscalar(noise.voltage_filter)), ...
        'phase3battery:VoltageFilter','Voltage filter must be a scalar name.');
    assert(any(strcmp(noise.voltage_filter,{'none','legacy_lowpass'})), ...
        'phase3battery:VoltageFilter','Unknown voltage filter.');
end
