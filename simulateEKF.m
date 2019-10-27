function simulateEKF(x_init,P_init,timeVec,IMU_meas,IMU_noise_param,GPS_data,fuse_vel,vel_err,plotStates)

% don't start running until we are getting velocity measurements
start_index = min(find(fuse_vel));
end_index = max(find(fuse_vel));

Ndata       = end_index - start_index +1;
x_EKF       = x_init;
P_EKF       = P_init;
state_out   = zeros(3,Ndata);
cov_out     = zeros(3,Ndata);
quat        = [1;0;0;0];
initialised = boolean(false);

state_out(:,1) = x_init;
cov_out(:,1) = diag(P_init);

idx = 1;


tic;
for i = start_index:end_index
    % Predict
    [x_EKF,P_EKF,quat,initialised] = EKFpredict(quat,initialised,x_EKF, P_EKF, IMU_meas(i,:), IMU_noise_param);
    
    %Update
    if fuse_vel(i) == 1
        [quat,x_EKF,P_EKF,S,nu] = EKFupdate(quat,x_EKF,P_EKF,GPS_data(i,:),[vel_err(i)^2,vel_err(i)^2]);
        S_mat(:,idx)    = diag(S);
        nu_mat(:,idx)   = nu;
        obsTime(idx)    = timeVec(i);
        idx               = idx + 1;
    end
    
    state_out(:,i-start_index+1) = x_EKF;
    cov_out(:,i-start_index+1) = diag(P_EKF);
    
end
elapsedTime = toc;

if plotStates == 1
    plotCovs = 1;
    plotFilterStates(state_out,timeVec(start_index:end_index),'EKF',plotCovs,S_mat,nu_mat,obsTime)
end

save('EKF_Data','timeVec','state_out','cov_out','obsTime','S_mat','nu_mat','elapsedTime')