clear all, close all, clc

%load test data
% sample file here https://drive.google.com/drive/u/0/folders/0By4v2BuLAaCfU1FSYUQ5aVhBT2c
load test_data/yaw_est_input_data_03.mat; 

N_models    = 7; % Number of models for GSF and IMM - enables initial yaw values 60 degrees apart which provides fast initial convergence.
noParticles = 500; 

%Define noise parameters
IMU_noise_param = [0.35;0.015]*10; % Accel/gyro noise [m/s^2, rad/s]
GPS_noise_param = [0.3;0.3]; % Velocity north/east noise
initial_state_uncertainty = [0.5;0.5;deg2rad(0.5*360/(N_models-1))]; % [m/s, m/s, rad]

%  sensor data 
%          IMU_data       - [delAngX,delAngY,delAngZ,delAngDt,delVelX,delVelY,delVelZ,delVelDt]
%          GPS_data       - [velN,velE,velD]
timeVec = timestamp*1E-6;
IMU_data(:,1) = del_ang0;
IMU_data(:,2) = del_ang1;
IMU_data(:,3) = del_ang2;
IMU_data(:,4) = del_ang_dt;
IMU_data(:,5) = del_vel0;
IMU_data(:,6) = del_vel1;
IMU_data(:,7) = del_vel2;
IMU_data(:,8) = del_vel_dt;
GPS_data(:,1) = vel0;
GPS_data(:,2) = vel1;
GPS_data(:,3) = vel2;

%Initialise filters
deg2rad             = pi/180;
x_init              = [0;0;0];
P_init              = diag(initial_state_uncertainty.^2);
Q0                  = diag([median(del_vel_dt)*IMU_noise_param(1);median(del_vel_dt)*IMU_noise_param(1);median(del_ang_dt)*IMU_noise_param(2)].^2);

%Flags
plotStates          = 1;

%*********Single Filters*********
%EKF
simulateEKF(x_init, P_init, timeVec, IMU_data, IMU_noise_param, GPS_data, fuse_vel, vel_err, plotStates);

%UKF
%simulateUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,Q0,truthDataNav,plotStates);

%*********Gaussian Sum Filters*********
%GSF-EKF
simulateGSFEKF(x_init,P_init,timeVec, IMU_data, IMU_noise_param, GPS_data, fuse_vel, vel_err, N_models, plotStates);

%GSF-UKF
%simulateGSFUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,Q0,truthDataNav,N,plotStates);

%*********Interacting Multiple Model Filters*********
%IMM-EKF
%simulateIMMEKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,truthDataNav,N,plotStates);

%IMM-UKF
%simulateIMMUKF(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,Q0,truthDataNav,N,plotStates);

%*****************Particle Filter*******************
%simulateParticleFilter(x_init,P_init,timeVec,dt,IMU_data,IMU_noise_param,GPS_data,GPS_noise_param,Q0,truthDataNav,noParticles,plotStates);
               