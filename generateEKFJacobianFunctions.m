syms daz 'real' % IMU delta angle measurements in body axes - rad
syms dvx dvy 'real' % IMU delta velocity measurements in body axes - m/sec
syms psi 'real'
syms vn ve 'real' % NED velocity - m/sec
syms dt 'real' % IMU time step - sec
syms gravity 'real' % gravity  - m/sec^2
syms dazVar dvxVar dvyVar  'real'; % IMU delta angle and delta velocity measurement variances

% derive the body to nav direction transformation matrix
Tbn = [cos(psi) , -sin(psi) ;...
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
Pnext = F*P*F' +Q;
matlabFunction(Pnext,'file','calcPmat.m');
ccode(Pnext,'file','calcPmat.txt');