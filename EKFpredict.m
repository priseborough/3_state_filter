function [x,P] = EKFpredict(x,P,dt,IMU_meas_prev,IMU_meas_current,IMU_noise_param)

%Extract states
Vn              = x(1);
Ve              = x(2);
psi             = x(3);

ax_prev         = IMU_meas_prev(2);
ay_prev         = IMU_meas_prev(3);
yawRate_prev    = IMU_meas_prev(4);

ax_current      = IMU_meas_current(2);
ay_current      = IMU_meas_current(3);
yawRate_current = IMU_meas_current(4);

accelNoise      = IMU_noise_param(1);
gyrNoise        = IMU_noise_param(2);

%Predict states forward
psi_current = psi +  yawRate_current * dt;

%Convert velocities to body frame:
% Cbn = [cos(psi) , -sin(psi) ;...
%        sin(psi) ,  cos(psi)];

Vx = Vn*cos(psi) + Ve*sin(psi);
Vy = Vn*(-sin(psi)) + Ve*cos(psi);

dvx = 0.5*dt*(ax_prev + ax_current);
Vx  = Vx + dvx;

dvy = 0.5*dt*(ay_prev + ay_current);
Vy  = Vy + dvy;

Vn = Vx * cos(psi_current) - Vy * sin(psi_current); 
Ve = Vx * sin(psi_current) - Vy * cos(psi_current); 

F            = calcFmat(dvx,dvy,psi);
Q            = calcQmat((gyrNoise*dt)^2,(accelNoise*dt)^2,(accelNoise*dt)^2,psi);

P            = F*P*F' + Q;
x            = [Vn;Ve;psi_current];
IMUData      = IMU_meas_current;