function [state,audit] = apply_phase3_initial_condition(state,condition)
%apply_phase3_initial_condition Perturb a fresh reset, never a resumed state.
% Right-multiply body rotation; keep CoM position, world feet and carried forces.
    condition = validate_phase3_initial_condition(condition);
    assert(state.t==0 && isempty(fieldnames(state.fsm_internal_state)) && ...
        isempty(fieldnames(state.decision_bookkeeping)) && ...
        isempty(state.current_time) && isempty(state.current_total) && isempty(state.mpc_warm_start), ...
        'phase3InitialCondition:NotFresh','Perturbations require a fresh pre-FSM reset.');
    validateattributes(state.Xt,{'double'},{'size',[30,1],'finite','real'});
    before = state.Xt;
    rotation = reshape(before(7:15),3,3);
    assert(norm(rotation.'*rotation-eye(3),'fro')<1e-10 && abs(det(rotation)-1)<1e-10, ...
        'phase3InitialCondition:Rotation','Initial rotation must already be in SO(3).');
    eta = condition.rotation_vector_body_rad;
    if any(eta~=0)
        rotation = rotation*expm([0,-eta(3),eta(2);eta(3),0,-eta(1);-eta(2),eta(1),0]);
        state.Xt(7:15) = rotation(:);
    end
    state.Xt(4:6) = before(4:6)+condition.linear_velocity_world_delta_m_s;
    state.Xt(16:18) = before(16:18)+condition.angular_velocity_body_delta_rad_s;
    audit = struct('condition',condition,'Xt_before',before,'Xt_after',state.Xt, ...
        'world_feet_unchanged',isequal(before(19:30),state.Xt(19:30)), ...
        'scope','Synthetic initial state only; no impulse, equilibrium or safety claim.');
end
