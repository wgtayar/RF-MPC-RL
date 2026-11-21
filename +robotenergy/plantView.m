function q = plantView(s)
%PLANTVIEW Cell state seen by the physical plant.
% Without a declared mismatch (no s.plant) the state itself is returned, so
% the nominal plant arithmetic is unchanged. With s.plant (see
% robotenergy.plantMismatch) the plant resistances R0..R3 are scaled and the
% plant capacity replaces the nominal one. The estimator never calls this.
q = s;
if ~isfield(s,'plant'), return; end
q.values(:,1:4) = s.values(:,1:4)*s.plant.resistance_scale;
q.capacity = s.plant.capacity_Ah;
end
