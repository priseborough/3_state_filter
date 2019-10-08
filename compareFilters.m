clear all, close all, clc

%Define times
t0              = 0;
tf              = 30;
dt              = 0.01; %Prediction update rate
timeVec         = [t0:dt:tf];
GPSUpdateRate   = 10; % Hz
N               = 20; % Number of models for GSF 
noParticles     = 500; 

%Define noise parameters
IMU_noise_param = [0.35;0.015]*10; % Accel/gyro noise [m/s^2, rad/s]
GPS_noise_param = [0.5;0.5]; % Velocity north/east noise
initial_state_uncertainty = [0.5;0.5;deg2rad(20)]; % [m/s, m/s, rad]

%Generate truth and sensor data 
[truthBodyRates, truthDataBody, truthDataNav, IMU_data, GPS_data] = simulateTruthIMUandGPS(timeVec,dt,GPSUpdateRate);

%Initialise filters
deg2rad             = pi/180;
state_error_init    = [-0.5;0.5;90*deg2rad];
x_init              = [truthDataNav(1);truthDataNav(2);truthDataNav(3)] + state_error_init;
P_init              = diag(initial_state_uncertainty.^2);
Q0                  = diag([dt*IMU_noise_param(1);dt*IMU_noise_param(1);dt*IMU_noise_param(2)].^2);

%Flags
plotStates          = 1;

%*********Single Filters*********
%EKF
simulateEKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,truthDataNav,plotStates);

%UKF
simulateUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,Q0,truthDataNav,plotStates);

%*********Gaussian Sum Filters*********
%GSF-EKF
simulateGSFEKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,truthDataNav,N,plotStates);

%GSF-UKF
simulateGSFUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,Q0,truthDataNav,N,plotStates);

%*********Interacting Multiple Model Filters*********
%IMM-EKF
simulateIMMEKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,truthDataNav,N,plotStates);

%IMM-UKF
simulateIMMUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,Q0,truthDataNav,N,plotStates);

%*****************Particle Filter*******************
simulateParticleFilter(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,Q0,truthDataNav,noParticles,plotStates);
               