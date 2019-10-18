function simulateEKF(x_init,P_init,timeVec,dt,IMU_meas,IMU_noise_param,GPS_data,fuse_vel,GPS_noise,plotStates)
Ndata       = length(timeVec);
x_EKF       = x_init;
P_EKF       = P_init;
state_out   = zeros(3,Ndata);
cov_out     = zeros(3,Ndata);
idx         = 2;

state_out(:,1) = x_init;
cov_out(:,1) = diag(P_init);

tic;
for i = 2:Ndata
    %Predict
    [x_EKF,P_EKF] = EKFpredict(x_EKF,P_EKF,dt(i),IMU_meas(i-1,:),IMU_meas(i,:),IMU_noise_param);
    
    %Update
    if fuse_vel(i) == 1
        [x_EKF,P_EKF,S,nu] = EKFupdate(x_EKF,P_EKF,GPS_data(idx,:),GPS_noise);
        S_mat(:,idx-1)    = diag(S);
        nu_mat(:,idx-1)   = nu;
        obsTime(idx-1)    = timeVec(i);
        idx               = idx + 1;
    end
    
    state_out(:,i) = x_EKF;
    cov_out(:,i) = diag(P_EKF);
    
end
elapsedTime = toc;

if plotStates == 1
    plotCovs = 1;
    plotFilterStates(state_out,timeVec,'EKF',plotCovs,S_mat,nu_mat,obsTime)
end

save('EKF_Data','timeVec','state_out','cov_out','obsTime','S_mat','nu_mat','elapsedTime')