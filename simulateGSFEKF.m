function simulateGSFEKF(x_init,P_init,timeVec,IMU_meas,IMU_noise,GPS_meas,fuse_vel,vel_err,N_models,plotStates)

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
P_GSFEKF_out(:,1) = diag(P_init);
quat        = zeros(4,N_models);
quat(1,:)   = ones(1,N_models);
initialised = boolean(zeros(1,N_models));


[X_filter,P_filter,w] = initialiseGSFEKF(x_init,P_init,N_models);

tic;
for i = start_index:end_index
    for j = 1:N_models
        %Predict
        [X_filter(:,j),P_filter(:,:,j),quat(:,j),initialised(j)] = EKFpredict(quat(:,j),initialised(j),X_filter(:,j),P_filter(:,:,j),IMU_meas(i,:),IMU_noise);
    end
    
    %Update
    if fuse_vel(i) == 1
        obsTime(idx) = timeVec(i);
        for j = 1:N_models
            [quat(:,j),X_filter(:,j),P_filter(:,:,j),S(:,:,j),nu(:,j)] = EKFupdate(quat(:,j),X_filter(:,j),P_filter(:,:,j),GPS_meas(i,:)',[vel_err(i)^2,vel_err(i)^2]);
            S_mat(:,idx,j) = diag(S(:,:,j));
            nu_mat(:,idx,j) = nu(:,j);
        end
        
        
        total_w = 0;
        for j = 1:N_models
            %Normal distributions N(nu,0,S)
            normDist(j)     = GaussianDensity(nu(:,j), 0, S(:,:,j));
            newWeight(j)    = normDist(j)*w(j);
            total_w         = newWeight(j) + total_w;
        end
        w                   = newWeight/total_w;
        idx                 = idx + 1;
    end
    
    
    X_GSF       = zeros(3,1);
    P_GSF       = zeros(3,3);
    for j = 1:N_models
        X_GSF   = X_GSF + w(j)*X_filter(:,j);
    end
    
    for j = 1:N_models
        P_GSF   = P_GSF + w(j)*(P_filter(:,:,j)+((X_filter(:,j)-X_GSF)*(X_filter(:,j)-X_GSF)'));
    end
    
    state_out(:,i-start_index+1) = X_GSF;
    cov_out(:,i-start_index+1) = diag(P_GSF);
    weights_out(:,i-start_index+1)  = w;
    
end
elapsedTime = toc;

if plotStates == 1
    plotCovs = 1;
    plotFilterStates(state_out,timeVec(start_index:end_index),'GSF - EKF',plotCovs,S_mat,nu_mat,obsTime)
end

save('GSF_EKF_Data','timeVec','state_out','cov_out','obsTime','S_mat','nu_mat','elapsedTime','weights_out')