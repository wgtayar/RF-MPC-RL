function [packCurrent,cellCurrent,indices] = alignSamples(inputTime,inputPackCurrent,returnedTime,nParallel)
%alignSamples Join the model's returned timestamps to original current samples.
% Exact subset matching only: no interpolation, snapping or prefix truncation.
    validateattributes(inputTime,{'double'},{'real','finite'});
    validateattributes(inputPackCurrent,{'double'},{'real','finite','nonnegative'});
    validateattributes(returnedTime,{'double'},{'real','finite'});
    validateattributes(nParallel,{'double'},{'scalar','real','finite','integer','positive'});
    assert((isvector(inputTime) || isempty(inputTime)) && ...
        (isvector(inputPackCurrent) || isempty(inputPackCurrent)) && ...
        (isvector(returnedTime) || isempty(returnedTime)) && ...
        numel(inputTime)==numel(inputPackCurrent), ...
        'phase3battery:Length','Require matching input time/current vectors.');
    inputTime = inputTime(:);
    inputPackCurrent = inputPackCurrent(:);
    returnedTime = returnedTime(:);
    assert(all(diff(inputTime)>0) && all(diff(returnedTime)>0), ...
        'phase3battery:Order','Sample timestamps must be unique and increasing.');
    [present,indices] = ismember(returnedTime,inputTime);
    assert(all(present),'phase3battery:Subset', ...
        'Returned times must be exact members of the original time vector.');
    packCurrent = inputPackCurrent(indices);
    cellCurrent = packCurrent/nParallel;
end
