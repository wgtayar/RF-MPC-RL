function q = estimatorView(s)
%ESTIMATORVIEW Cell model used by the SOC estimator.
% Without a declared estimator re-identification the state itself is
% returned, so the nominal estimator arithmetic is unchanged. With
% s.plant.estimator_resistance_scale (see robotenergy.plantMismatch) the
% estimator's R0..R3 are scaled by that factor: the battery management
% system has re-identified the pack's resistance (protocol A4 control). The
% estimator capacity, OCV and RC capacitances stay nominal.
q = s;
if ~isfield(s,'plant') || ~isfield(s.plant,'estimator_resistance_scale'), return; end
q.values(:,1:4) = s.values(:,1:4)*s.plant.estimator_resistance_scale;
end
