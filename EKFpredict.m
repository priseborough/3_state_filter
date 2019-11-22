function [x,P,quat,initialised] = EKFpredict(quat,initialised,x,P,IMU_data,IMU_noise_param)

accelNoise      = IMU_noise_param(1);
gyrNoise        = IMU_noise_param(2);

% generate an attitude reference using IMU data
[quat, initialised] = QuatPredict(x, quat, initialised, IMU_data(1:3), IMU_data(4), IMU_data(5:7), IMU_data(8));

% calculate delta velocity in a horizontal front-right frame
R = Quat2Tbn(quat);
del_vel_NED = R * [IMU_data(5);IMU_data(6);IMU_data(7)];
if (abs(R(3, 1)) < abs(R(3, 2)))
    % use 321 Tait-Bryan rotation
    yaw = atan2(R(2, 1), R(1, 1));
else
    % use 312 Tait-Bryan rotation
    yaw = atan2(-R(1, 2), R(2, 2)); % first rotation (yaw)
end
del_vel_FR(1) =   del_vel_NED(1) * cos(yaw) + del_vel_NED(2) * sin(yaw);
del_vel_FR(2) = - del_vel_NED(1) * sin(yaw) + del_vel_NED(2) * cos(yaw);

% sum delta velocties in earth frame:
x(1) = x(1) + del_vel_NED(1);
x(2) = x(2) + del_vel_NED(2);

% take yaw angle from quaternion solution
x(3) = yaw;

% predict covariance
% optimised version of
% F = calcFmat(del_vel_FR(1),del_vel_FR(2),x(3));
% Q = calcQmat((gyrNoise*IMU_data(4))^2,(accelNoise*IMU_data(8))^2,(accelNoise*IMU_data(8))^2,x(3));
% P = F'*P*F + Q;

dazVar = (gyrNoise*IMU_data(4))^2;
dvyVar = (accelNoise*IMU_data(8))^2;
dvxVar = dvyVar;
P = calcPmat(P(1,1),P(1,2),P(1,3),P(2,1),P(2,2),P(2,3),P(3,1),P(3,2),P(3,3),dazVar,del_vel_FR(1),del_vel_FR(2),dvxVar,dvyVar,x(3));