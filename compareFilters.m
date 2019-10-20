clear all, close all, clc

%load test data
% sample file here https://drive.google.com/open?id=1pO8e1G0lPnGlIkAwjPafn4Y1HuDTgPYm
load test_data/08_52_44_yaw_est_test_data_0.mat; 

N               = 7; % Number of models for GSF and IMM - enables initial yaw values 60 degrees apart which provides fast initial convergence.
noParticles     = 500; 

%Define noise parameters
IMU_noise_param = [0.35;0.015]*10; % Accel/gyro noise [m/s^2, rad/s]
GPS_noise_param = [0.3;0.3]; % Velocity north/east noise
initial_state_uncertainty = [0.5;0.5;deg2rad(0.5*360/(N-1))]; % [m/s, m/s, rad]

%  sensor data 
%          IMU_data       - [time, ax[m/s^2], ay[m/s^2], psi_dot[rads/s]]
%          GPS_data       - [time[secs], Vn_GPS[m/s], Ve_GPS[m/s]]
start_index = min(find(fuse_vel));
end_index = max(find(fuse_vel));
timeVec = timestamp(start_index:end_index)*1E-6;
IMU_data(:,1) = timeVec;
IMU_data(:,2) = accel_fr0(start_index:end_index);
IMU_data(:,3) = accel_fr1(start_index:end_index);
IMU_data(:,4) = yaw_rate(start_index:end_index);
GPS_data(:,1) = timeVec;
GPS_data(:,2) = vel_ne0(start_index:end_index);
GPS_data(:,3) = vel_ne1(start_index:end_index);

%Initialise filters
deg2rad             = pi/180;
x_init              = [0;0;0];
P_init              = diag(initial_state_uncertainty.^2);
Q0                  = diag([dt*IMU_noise_param(1);dt*IMU_noise_param(1);dt*IMU_noise_param(2)].^2);

%Flags
plotStates          = 1;

%*********Single Filters*********
%EKF
simulateEKF(x_init, P_init, timeVec, dt, IMU_data, IMU_noise_param, GPS_data, fuse_vel, GPS_noise_param, plotStates);

%UKF
%simulateUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,Q0,truthDataNav,plotStates);

%*********Gaussian Sum Filters*********
%GSF-EKF
simulateGSFEKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data, fuse_vel, GPS_noise_param,N,plotStates);

%GSF-UKF
%simulateGSFUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,Q0,truthDataNav,N,plotStates);

%*********Interacting Multiple Model Filters*********
%IMM-EKF
%simulateIMMEKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,truthDataNav,N,plotStates);

%IMM-UKF
%simulateIMMUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,Q0,truthDataNav,N,plotStates);

%*****************Particle Filter*******************
%simulateParticleFilter(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,Q0,truthDataNav,noParticles,plotStates);
               