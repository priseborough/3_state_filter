function [x,P,S, innov] = EKFupdate(x,P,obs,GPS_noise)

%Caclulate Innovation
innov = obs(2:3)' - x(1:2);

%Calculate innovation covariance
H     = [1,0,0;0,1,0]; 
R     = diag(GPS_noise); 
S     = H * P * H' + R;

%Calculate Kalman gain
K     = P * H' * inv(S);  

%Update
x     = x + K * innov;
P     = P - K*S*K';