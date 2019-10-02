clear all, close all, clc

%Define times
t0              = 0;
tf              = 30;
dt              = 0.01; %Prediction update rate
timeVec         = [t0:dt:tf];
GPSUpdateRate   = 10; %10Hz
N               = 20; % Number of models for GSF 
noParticles     = 500; 

%Define noise parameters
IMU_noise = [0.35;0.003]*10;
GPS_noise = [0.5;0.5];

%Generate truth and sensor data 
[truthBodyRates, truthDataBody, truthDataNav, IMU_data, GPS_data] = simulateTruthIMUandGPS(timeVec,dt,GPSUpdateRate);

%Initialise filters
deg2rad             = pi/180;
state_error_init    = [-0.2;0.2;45*deg2rad];
x_init              = [truthDataNav(1);truthDataNav(2);truthDataNav(3)] + state_error_init;
P_init              = diag(state_error_init.^2);
Q0                  = diag([dt*IMU_noise(1);dt*IMU_noise(1);dt*IMU_noise(2)].^2);

%Flags
plotStates          = 1;

%*********Single Filters*********
%EKF
simulateEKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise,GPS_data,GPS_noise,truthDataNav,plotStates);

%UKF
simulateUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise,GPS_data,Q0,truthDataNav,plotStates);

%*********Gaussian Sum Filters*********
%GSF-EKF
simulateGSFEKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise,GPS_data,GPS_noise,truthDataNav,N,plotStates);

%GSF-UKF
simulateGSFUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise,GPS_data,GPS_noise,Q0,truthDataNav,N,plotStates);

%*********Interacting Multiple Model Filters*********
%IMM-EKF
simulateIMMEKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise,GPS_data,GPS_noise,truthDataNav,N,plotStates);

%IMM-UKF
simulateIMMUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise,GPS_data,GPS_noise,Q0,truthDataNav,N,plotStates);

%*****************Particle Filter*******************
simulateParticleFilter(x_init,P_init,timeVec,dt,IMU_data,IMU_noise,GPS_data,GPS_noise,Q0,truthDataNav,noParticles,plotStates);

               