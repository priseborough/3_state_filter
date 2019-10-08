function [truthBodyRates, truthDataBody, truthDataNav, IMU_data, GPS_data] = simulateTruthIMUandGPS(timeVec,dt,GPSUpdateRate)
% Generate reference (or truth) trajectory
% Vector = [ax, ay, psiDot]
%
% Inputs: timeVec       - secs
%         dt            - secs
%         GPSUpdateRate - Hz
%
% Outputs: truthBodyRates - [time, ax_truth[m/s^2], ay_truth[m/s^2], psiDot_truth[rads/s]]
%          truthDataBody  - [time, vx_truth[m/s], vy_truth[m/s], psi_truth[rads]]
%          truthDataNav   - [time, Vn_truth[m/s], Ve_truth[m/s]]
%          IMU_data       - [time, ax[m/s^2], ay[m/s^2], psi_dot[rads/s]]
%          GPS_data       - [time[secs], Vn_GPS[m/s], Ve_GPS[m/s]]

truthBodyRates        = zeros(length(timeVec),4);
truthDataBody         = zeros(length(timeVec),4);
truthDataNav          = zeros(length(timeVec),3);

IMU_data              = zeros(length(timeVec),4); %IMU recived at 100Hz time, ax, ay, psiDot
GPS_data              = zeros(ceil(length(timeVec)/GPSUpdateRate),3); %GPS recived at 10Hz, time, Vn, Ve

%Generate body axis data
for i = 1:length(timeVec)
    if timeVec(i) < 5 % Stationary for 5 seconds
        truthBodyRates(i,1) = timeVec(i);
        truthBodyRates(i,2) = 0;
        truthBodyRates(i,3) = 0;
        truthBodyRates(i,4) = 0;
    elseif timeVec(i) > 5 && timeVec(i) < 10 % Accelerate for 5 seconds in a straight line
        truthBodyRates(i,1) = timeVec(i);
        truthBodyRates(i,2) = 5;
        truthBodyRates(i,3) = 0;
        truthBodyRates(i,4) = 0;
    elseif timeVec(i) > 10 && timeVec(i) < 15 % Turn right at 0.5rads/sec
        truthBodyRates(i,1) = timeVec(i);
        truthBodyRates(i,2) = 0;
        truthBodyRates(i,3) = 0;
        truthBodyRates(i,4) = 0.5;        
    elseif timeVec(i) > 15 && timeVec(i) < 20 % Turn left at 0.5rads/sec
        truthBodyRates(i,1) = timeVec(i);
        truthBodyRates(i,2) = 0;
        truthBodyRates(i,3) = 0;
        truthBodyRates(i,4) = -0.5;        
    elseif timeVec(i) > 20 && timeVec(i) < 25 % Deccelerate for 5 seconds in a straight line
        truthBodyRates(i,1) = timeVec(i);
        truthBodyRates(i,2) = -5;
        truthBodyRates(i,3) = 0;
        truthBodyRates(i,4) = 0;
    end
end

%Trapezoidal integration for trajectory
for i = 2:length(timeVec)
    truthDataBody(i,1) = timeVec(i);
    truthDataBody(i,2) = truthDataBody(i-1,2) + 0.5*dt*(truthBodyRates(i-1,2) + truthBodyRates(i,2)); %Vx_body
    truthDataBody(i,3) = truthDataBody(i-1,3) + 0.5*dt*(truthBodyRates(i-1,3) + truthBodyRates(i,3)); %Vy_body
    truthDataBody(i,4) = truthDataBody(i-1,4) + 0.5*dt*(truthBodyRates(i-1,4) + truthBodyRates(i,4)); %psi  
end

%Convert to NED frame
truthDataNav(:,1) = timeVec;
truthDataNav(:,2) = truthDataBody(:,2).*cos(truthDataBody(:,4)) - truthDataBody(:,3).*sin(truthDataBody(:,4)); %Vn
truthDataNav(:,3) = truthDataBody(:,2).*sin(truthDataBody(:,4)) + truthDataBody(:,3).*cos(truthDataBody(:,4)); %Ve
truthDataNav(:,4) = truthDataBody(:,4);

%Simulate IMU
ax_noise      = 0.35; % m/sec^2
ay_noise      = 0.35; % m/sec^2
psiDot_noise  = 0.015; % rad/sec
IMU_data(:,1) = timeVec';
IMU_data(:,2) = truthBodyRates(:,2) + ax_noise*randn(length(timeVec),1); %ax
IMU_data(:,3) = truthBodyRates(:,3) + ay_noise*randn(length(timeVec),1); %ay
IMU_data(:,4) = truthBodyRates(:,4) + psiDot_noise*randn(length(timeVec),1); %psi_dot

%Simulate GPS
velNoise      = 0.5; %m/sec
GPS_data(:,1) = timeVec(1,1:GPSUpdateRate:end)';
GPS_data(:,2) = truthDataNav(1:GPSUpdateRate:end,2) +  velNoise*randn(length(GPS_data),1); %Vn
GPS_data(:,3) = truthDataNav(1:GPSUpdateRate:end,3) +  velNoise*randn(length(GPS_data),1); %Ve

end

