function simulateIMMEKF(x_init,P_init,timeVec,IMU_meas,IMU_noise,GPS_meas,fuse_vel,vel_err,N_models,plotStates)

% don't start running until we are getting velocity measurements
start_index = min(find(fuse_vel));
end_index = max(find(fuse_vel));

Ndata       = end_index - start_index +1;
Nobs        = sum(fuse_vel == 1);
idx         = 1;
S_mat       = zeros(2,Nobs,N_models);
nu_mat      = zeros(2,Nobs,N_models);
obsTime     = zeros(Nobs,1);

state_out(:,1) = x_init;
cov_out(:,1) = diag(P_init);

[X_Filter, P_Filter, X_IMM, P_IMM, transMatrix, modeProbs] = initialiseIMMEKF(x_init,P_init,N_models);

tic;
for i = start_index:end_index
    
    if fuse_vel(i) == 1
        EKF_update = 1;
    else
        EKF_update  = 0;
    end
    measurement = GPS_meas(i,:);
    
    [X_Filter,P_Filter, innovation_store, S_store, X_IMM, P_IMM, modeProbs] = IMMEKFUpdate(X_IMM,P_IMM,IMU_meas(i,:), IMU_noise, measurement, vel_err(i), transMatrix, modeProbs, EKF_update);
    
    state_out(:,i-start_index+1) = X_Filter;
    cov_out(:,i) = diag(P_Filter);
    
    if EKF_update == 1
        for j = 1:N_models
            S_mat(:,idx,j) = S_store(:,j);
            nu_mat(:,idx,j) = innovation_store(:,j);
        end
        obsTime(idx)      = timeVec(i);
        idx                 = idx + 1;
    end
    
end
elapsedTime = toc;

if plotStates ==  1
    plotCovs = 1;
    plotFilterStates(state_out,timeVec(start_index:end_index),'IMM - EKF',plotCovs,S_mat,nu_mat,obsTime)
end

save('IMM_EKF_Data','timeVec','state_out','cov_out','obsTime','S_mat','nu_mat','elapsedTime')