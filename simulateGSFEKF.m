function simulateGSFEKF(x_init,P_init,timeVec,dt,IMU_meas,IMU_noise,GPS_meas,GPS_noise,truthDataNav,N,plotStates)

Ndata       = length(timeVec);
idx         = 2;
IMU_noise   = IMU_noise;
S_mat       = zeros(2,length(GPS_meas)-1,N);
nu_mat      = zeros(2,length(GPS_meas)-1,N);
obsTime     = zeros(length(GPS_meas)-1,1);

state_out(:,1) = x_init;
P_GSFEKF_out(:,1) = diag(P_init);

[X_filter,P_filter,w] = initialiseGSFEKF(x_init,P_init,N);

tic;
for i = 2:Ndata
    for j = 1:N
        %Predict
        [X_filter(:,j),P_filter(:,:,j)] = EKFpredict(X_filter(:,j),P_filter(:,:,j),dt,IMU_meas(i-1,:),IMU_meas(i,:),IMU_noise);
    end
    
    %Update
    if timeVec(i) == GPS_meas(idx,1)
        obsTime(idx-1) = timeVec(i);
        for j = 1:N
            [X_filter(:,j),P_filter(:,:,j),S(:,:,j),nu(:,j)] = EKFupdate(X_filter(:,j),P_filter(:,:,j),GPS_meas(idx,:),GPS_noise);
            S_mat(:,idx-1,j) = diag(S(:,:,j));
            nu_mat(:,idx-1,j) = nu(:,j);
        end
        
        
        total_w = 0;
        for j = 1:N
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
    for j = 1:N
        X_GSF   = X_GSF + w(j)*X_filter(:,j);
    end
    
    for j = 1:N
        P_GSF   = P_GSF + w(j)*(P_filter(:,:,j)+((X_filter(:,j)-X_GSF)*(X_filter(:,j)-X_GSF)'));
    end
    
    state_out(:,i) = X_GSF;
    cov_out(:,i) = diag(P_GSF);
    weights_out(:,i)  = w;
    
end
elapsedTime = toc;

if plotStates == 1
    plotCovs = 1;
    plotFilterStates(state_out,timeVec,truthDataNav,'GSF - EKF',plotCovs,S_mat,nu_mat,obsTime)
end

save('GSF_EKF_Data','timeVec','truthDataNav','state_out','cov_out','obsTime','S_mat','nu_mat','elapsedTime','weights_out')