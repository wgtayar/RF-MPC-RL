function [observation,audit] = build_phase3_observation_v2(state,legacyObservation,lastRow,cfg,version)
%build_phase3_observation_v2 Versioned health observation without global state norm.
% lastRow is the last already executed MPC row, never a future/predicted label.
% An empty lastRow is allowed only at reset. Legacy observations are not changed.
    validateattributes(legacyObservation,{'double'},{'numel',19,'real','finite'});
    legacyObservation = legacyObservation(:);
    if nargin < 5
        version = 'observation_v2_dynamic_health_candidate_v1';
    end
    schema = phase3_observation_v2_schema(cfg,version);
    hasSolver = ~isempty(fieldnames(lastRow));
    assert(hasSolver || state.t==0,'observationV2:MissingHistory', ...
        'Nonzero-time observations require the actual previous MPC row.');
    if hasSolver
        assert(isfield(lastRow,'time_after') && lastRow.time_after==state.t && ...
            isfield(lastRow,'state_after') && isequaln(lastRow.state_after.Xt,state.Xt) && ...
            isequaln(lastRow.state_after.Ut,state.Ut), ...
            'observationV2:Endpoint','Previous MPC row must match this exact endpoint, not a future sample.');
    end
    book = state.decision_bookkeeping;
    progress = NaN;
    if isreal(state.Xt(1)) && isfinite(state.Xt(1))
        progress = max(0,state.Xt(1)-book.initial_position_x)/cfg.MISSION.D_TARGET_M;
    end
    timeFraction = state.t/cfg.MISSION_DURATION;
    base = [legacyObservation(schema.legacy_indices);progress-timeFraction;1-timeFraction];
    physical = nan(22,1);
    validState = isreal(state.Xt) && isreal(state.Ut) && all(isfinite([state.Xt(:);state.Ut(:)]));
    if hasSolver
        validState = validState && isreal(lastRow.Xd) && all(isfinite(lastRow.Xd(:)));
    end
    if validState
        rotation = reshape(state.Xt(7:15),3,3);
        desiredRotation = eye(3);
        velocityError = zeros(3,1);
        if hasSolver
            desiredRotation = reshape(lastRow.Xd(7:15,1),3,3);
            velocityError = state.Xt(4:6)-lastRow.Xd(4:6,1);
        end
        relative = desiredRotation.'*rotation;
        angle = acos(min(max((trace(relative)-1)/2,-1),1));
        skew = 0.5*[relative(3,2)-relative(2,3);relative(1,3)-relative(3,1);relative(2,1)-relative(1,2)];
        feet = reshape(state.Xt(19:30),3,4)-state.Xt(1:3);
        forces = reshape(state.Ut,3,4);
        physical = [state.Xt(4:6);velocityError;angle;skew;state.Xt(16:18); ...
            norm(rotation.'*rotation-eye(3),'fro');vecnorm(feet,2,1).';vecnorm(forces,2,1).'];
        validState = all(isfinite(physical));
    end
    if ~validState
        physical(:) = NaN;
    elseif ~hasSolver
        physical(4:6) = NaN;
    end
    fsm = nan(16,1);
    internal = state.fsm_internal_state;
    hasFsm = ~isempty(fieldnames(internal));
    if hasFsm
        assert(all(isfield(internal,{'FSM','Ta','Tb'})), ...
            'observationV2:FSM','All-leg FSM timers are required.');
        duration = internal.Tb(:)-internal.Ta(:);
        assert(numel(duration)==4 && all(isfinite(duration)) && all(duration>0), ...
            'observationV2:FSM','This observation schema supports positive-duration locomotion phases.');
        fsm = [double(internal.FSM(:)==1);double(internal.FSM(:)==2); ...
            (state.t-internal.Ta(:))./duration;duration];
    end
    actions = nan(10,1);
    solver = nan(3,1);
    classes = zeros(numel(schema.solver_classes),1);
    hasAction = false;
    hasMargin = false;
    if hasSolver
        execution = lastRow.action_execution;
        actions = [execution.candidate_action(:);execution.applied_action(:)];
        assert(numel(actions)==10 && all(isfinite(actions)), ...
            'observationV2:Action','Complete finite candidate/applied history is required.');
        hasAction = true;
        previousSolver = lastRow.solver;
        solver(1) = previousSolver.iterations;
        solver(3) = previousSolver.wall_time_s;
        if isfield(previousSolver.diagnostics,'inequality_margin_min')
            solver(2) = previousSolver.diagnostics.inequality_margin_min;
            hasMargin = isfinite(solver(2));
        end
        match = find(schema.solver_classes==string(previousSolver.classification),1);
        if isempty(match)
            match = numel(classes);
        end
        classes(match) = 1;
    end
    raw = [base;physical;fsm;actions;solver;classes; ...
        validState;hasSolver;hasFsm;hasAction;hasMargin];
    raw = raw(schema.raw_indices);
    assert(numel(raw)==schema.dimension,'observationV2:Dimension','Feature dimension mismatch.');
    available = isfinite(raw);
    encoded = raw;
    encoded(~available) = 0;
    observation = asinh(encoded./schema.scales);
    observation(schema.identity_transform) = encoded(schema.identity_transform);
    audit = struct('schema',schema,'raw',raw,'available',available, ...
        'clipped',false(size(raw)),'legacy_observation',legacyObservation, ...
        'previous_solver_wall_seconds',solver(3), ...
        'clipping_scope','No additional clipping in v2; retained legacy features keep their historical normalization.', ...
        'reference_semantics', ...
        'Previous executed QP reference; reset orientation uses world identity and velocity error is unavailable.');
end
