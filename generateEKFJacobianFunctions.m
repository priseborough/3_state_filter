syms daz 'real' % IMU delta angle yaw measurement in body axes - rad
syms dvx dvy 'real' % IMU delta velocity measurements in front-right body axes - m/sec
syms psi 'real' % yaw angle of X body axis wrt north
syms vn ve 'real' % NE velocity - m/sec
syms dt 'real' % IMU time step - sec
syms dazVar dvxVar dvyVar  'real'; % IMU delta angle and delta velocity measurement variances

% derive the body to nav direction transformation matrix
Tbn = [ cos(psi) , -sin(psi) ;...
        sin(psi) ,  cos(psi)];

% attitude update equation
psiNew = psi + daz;

% velocity update equations
velNew = [vn;ve] + Tbn*[dvx;dvy];

% Define the state vector & number of states
stateVector = [vn;ve;psi];
nStates=numel(stateVector);

% Define vector of process equations
newStateVector = [velNew;psiNew];
F = jacobian(newStateVector,stateVector);
matlabFunction(F,'file','calcFmat.m');

%% derive the covariance prediction equations
% This reduces the number of floating point operations by a factor of 6 or
% more compared to using the standard matrix operations in code

% Error growth in the inertial solution is assumed to be driven by 'noise' in the delta angles and
% velocities, after bias effects have been removed.

% derive the control(disturbance) influence matrix from IMU noise to state
% noise
G = jacobian(newStateVector, [dvx;dvy;daz]);

% derive the state error matrix
distMatrix = diag([dvxVar dvyVar dazVar]);
Q = G*distMatrix*transpose(G);
matlabFunction(Q,'file','calcQmat.m');

% derive expressions for the covariance prediction
syms P00 P01 P02 P10 P11 P12 P20 P21 P22 real;
P = [P00 P01 P02;P10 P11 P12;P20 P21 P22];
Pnext = F'*P*F + Q;
matlabFunction(Pnext,'file','calcPmat.m');
ccode(Pnext,'file','calcPmat.txt');

%% derive the covariance update equation
H = [1,0,0;0,1,0]; 
syms velObsVar real;
R = [velObsVar 0 ; 0 velObsVar]; 
S = H * P * H' + R;

matlabFunction(S,'file','calcS.m');
ccode(S,'file','calcS.txt');

%Calculate Kalman gain
K = P * H' * inv(S);

matlabFunction(K,'file','calcK.m');
ccode(K,'file','calcK.txt');

% Update
Pnew = P - K*S*K';

matlabFunction(Pnew,'file','calcPupdate.m');
ccode(Pnew,'file','calcPupdate.txt');