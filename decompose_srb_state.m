function components = decompose_srb_state(Xt, Xd)
%decompose_srb_state Decompose the implemented 30-state SRB representation.

    validateattributes(Xt, {'numeric'}, {'vector', 'numel', 30, 'finite'});
    Xt = Xt(:);
    if nargin < 2 || isempty(Xd)
        Xd = [zeros(6,1); reshape(eye(3), 9, 1); zeros(15,1)];
    elseif size(Xd, 2) > 1
        Xd = Xd(:,1);
    end
    validateattributes(Xd, {'numeric'}, {'vector', 'numel', 30, 'finite'});
    Xd = Xd(:);

    position = Xt(1:3);
    linearVelocity = Xt(4:6);
    rotation = reshape(Xt(7:15), 3, 3);
    desiredRotation = reshape(Xd(7:15), 3, 3);
    angularVelocity = Xt(16:18);
    footPositions = reshape(Xt(19:30), 3, 4);
    footPositionsRelative = footPositions - position;

    relativeRotation = desiredRotation.' * rotation;
    cosineAngle = min(max((trace(relativeRotation) - 1) / 2, -1), 1);
    orientationErrorRad = acos(cosineAngle);
    orientationSkew = 0.5 * [
        relativeRotation(3,2) - relativeRotation(2,3)
        relativeRotation(1,3) - relativeRotation(3,1)
        relativeRotation(2,1) - relativeRotation(1,2)
    ];

    components = struct();
    components.full_state_norm = norm(Xt);
    components.global_position_norm = norm(position);
    components.global_position_x = position(1);
    components.global_position_y = position(2);
    components.global_position_z = position(3);
    components.linear_velocity_norm = norm(linearVelocity);
    components.orientation_error_rad = orientationErrorRad;
    components.orientation_error_vector_norm = norm(orientationSkew);
    components.rotation_orthogonality_error = ...
        norm(rotation.' * rotation - eye(3), 'fro');
    components.angular_velocity_norm = norm(angularVelocity);
    components.foot_position_global_norm = norm(footPositions, 'fro');
    components.foot_position_relative_norm = norm(footPositionsRelative, 'fro');
    components.position_invariant_state_norm = norm([
        linearVelocity
        orientationSkew
        angularVelocity
        footPositionsRelative(:)
    ]);
end
