function simulateIMMEKF(x_init,P_init,timeVec,dt,IMU_meas,IMU_noise,GPS_meas,GPS_noise,truthDataNav,N,plotStates)

Ndata       = length(timeVec);
idx         = 2;
IMU_noise   = IMU_noise;

state_out(:,1) = x_init;
cov_out(:,1) = diag(P_init);

[X_Filter, P_Filter, X_IMM, P_IMM, transMatrix, modeProbs] = initialiseIMMEKF(x_init,P_init,N);

tic;
for i = 2:Ndata
    
    if timeVec(i) == GPS_meas(idx,1)
        EKF_update = 1;
        measurement = GPS_meas(idx,:);
    else
        EKF_update  = 0;
        measurement = [];
    end
    
    
    [X_Filter,P_Filter, innovation_store, S_store, X_IMM, P_IMM, modeProbs] = IMMEKFUpdate(X_IMM,P_IMM,IMU_meas(i-1:i,:),IMU_noise,measurement,GPS_noise,transMatrix,modeProbs,EKF_update,dt);
    
    state_out(:,i) = X_Filter;
    cov_out(:,i) = diag(P_Filter);
    
    if EKF_update == 1
        S_mat(:,idx-1,:)    = S_store;
        nu_mat(:,idx-1,:)   = innovation_store;
        obsTime(idx-1)      = timeVec(i);
        idx                 = idx + 1;
    end
    
end
elapsedTime = toc;

if plotStates ==  1
    plotCovs = 1;
    plotFilterStates(state_out,timeVec,truthDataNav,'IMM - EKF',plotCovs,S_mat,nu_mat,obsTime)
end

save('IMM_EKF_Data','timeVec','truthDataNav','state_out','cov_out','obsTime','S_mat','nu_mat','elapsedTime')