function quat = AlignYaw(...
    quat, ... % quaternion
    initYaw... % initial yaw angle (rad)
    )
    R_to_earth = Quat2Tbn(quat);
	if (abs(R_to_earth(3, 1)) < abs(R_to_earth(3, 2)))
        euler_init = QuatToEul(quat);
        euler_init(3) = initYaw;
        quat = EulToQuat(euler_init);
        
    else
        % Calculate the 312 Tait-Bryan sequence euler angles that rotate from earth to body frame
        % http://www.atacolorado.com/eulersequences.doc
        % euler312(0) = atan2(-R_to_earth(1, 2), R_to_earth(2, 2));  % first rotation (yaw)
        euler312(1) = initYaw;
        euler312(2) = asin(R_to_earth(2, 1)); % second rotation (roll)
        euler312(3) = atan2(-R_to_earth(3, 1), R_to_earth(3, 3));  % third rotation (pitch)

        % convert back to rotaton matrix quaternion
        c2 = cos(euler312(3));
        s2 = sin(euler312(3));
        s1 = sin(euler312(2));
        c1 = cos(euler312(2));
        s0 = sin(euler312(1));
        c0 = cos(euler312(1));
        
        R_to_earth(1, 1) = c0 * c2 - s0 * s1 * s2;
        R_to_earth(2, 2) = c0 * c1;
        R_to_earth(3, 3) = c2 * c1;
        R_to_earth(1, 2) = -c1 * s0;
        R_to_earth(1, 3) = s2 * c0 + c2 * s1 * s0;
        R_to_earth(2, 1) = c2 * s0 + s2 * s1 * c0;
        R_to_earth(2, 3) = s0 * s2 - s1 * c0 * c2;
        R_to_earth(3, 1) = -s2 * c1;
        R_to_earth(3, 2) = s1;

        % then convert back to a quaternion
        quat = Tbn2Quat(R_to_earth);
    end
end