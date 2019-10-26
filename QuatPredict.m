function [quat_out,initialised] = QuatPredict(quat_in,delAng,delAngDt,delVel,delVelDt,initialised)

	% generate attitude solution using simple complementary filter

	% check for excessive acceleration.
    
	accel = delVel ./ delVelDt;
	accel_norm_sq = sum(accel.*accel);
    accel_norm = sqrt(accel_norm_sq);
    CONSTANTS_ONE_G = 9.80665;
	upper_accel_limit = CONSTANTS_ONE_G * 1.1;
	lower_accel_limit = CONSTANTS_ONE_G * 0.9;

	% Angular rate of correction
	ok_to_align = boolean((accel_norm > lower_accel_limit && accel_norm < upper_accel_limit));

	% Iniitialise quaternion first time
	if (~initialised)
		if (ok_to_align)
			quat_out = AlignTilt(delVel);
			initialised = boolean(true);
        else
            initialised = boolean(false);
        end
	else
		% Accelerometer correction
		% Project 'k' unit vector of earth frame to body frame
		k = [
            2 * (quat_in(2) * quat_in(4) - quat_in(1) * quat_in(3));
			2 * (quat_in(3) * quat_in(4) + quat_in(1) * quat_in(2));
			(quat_in(1) * quat_in(1) - quat_in(2) * quat_in(2) - quat_in(3) * quat_in(3) + quat_in(4) * quat_in(4))
            ];

		% correct using accel data only if its norm is close to 1 g (reduces drift when vehicle picked up and moved).
		accel_fusion_gain = 0.0;
        if (accel_norm > 0.5 * CONSTANTS_ONE_G && accel_norm < 1.5 * CONSTANTS_ONE_G)
            accel_fusion_gain = 0.4 * (1.0 - 2.0 * abs(accel_norm - CONSTANTS_ONE_G)/CONSTANTS_ONE_G)^2;
        end
        
        % misalignment between gravity vector and measured accel unit
        % vectors is used to calculate a correction.
		correction = [0;0;0];
		if (ok_to_align)
			correction = cross(k , accel ./ accel_norm) * accel_fusion_gain * delAngDt;
        end

		% rotate attitude forward by gyro and correction delta angle 
		quat_out = QuatMult(quat_in , RotToQuat(correction + delAng));

		% Normalize quaternion
		quat_out = NormQuat(quat_out);
end