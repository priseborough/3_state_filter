function quat = AlignYaw(...
    quat, ... % quaternion
    initYaw... % initial yaw angle (rad)
    )

    euler_init = QuatToEul(quat);
    euler_init(3) = initYaw;
    quat = EulToQuat(euler_init);
    
end