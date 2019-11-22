function [quat,x,P,S, innov] = EKFupdate(quat,x,P,obs,obs_noise_var)

%Calculate Innovation
innov = obs(1:2) - x(1:2);

%Calculate innovation covariance
% H     = [1,0,0;0,1,0]; 
% R     = diag(obs_noise_var); 
% S     = H * P * H' + R;
S = calcS(P(1,1),P(1,2),P(2,1),P(2,2),obs_noise_var(1));

%Calculate Kalman gain
% K     = P * H' * inv(S);
K = calcK(P(1,1),P(1,2),P(2,1),P(2,2),P(3,1),P(3,2),obs_noise_var(1));

%Update
correction = K * innov;
x     = x + correction;
P     = P - K*S*K';

% Apply yaw correction to AHRS quaternion
delta_rot_ef = [0;0;correction(3)];
R = transpose(Quat2Tbn(quat));
delta_rot_bf = R * delta_rot_ef;
quat = QuatMult(quat, RotToQuat(delta_rot_bf));