function simulateGSFUKF(x_init,P_init,timeVec,dt,IMU_meas,IMU_noise,GPS_meas,GPS_noise,Q,truthDataNav,N,plotStates)

Ndata       = length(timeVec);
idx         = 2;
IMU_noise   = IMU_noise;
S_mat       = zeros(2,length(GPS_meas)-1,N);
nu_mat      = zeros(2,length(GPS_meas)-1,N);
obsTime     = zeros(length(GPS_meas)-1,1);


na          = length(x_init);
noMeas      = 2;
noGSFmodels = N;

R           = diag(GPS_noise);

state_out(:,1) = x_init;
cov_out(:,1) = diag(P_init);

[X_filter,P_filter,W_GSF] = initialiseGSFUKF(x_init,P_init,N);

tic;
for i = 2:Ndata
    for k = 1:noGSFmodels
        [W_UKF(:,k),sigma(:,:,k)] = unscentedTransform(X_filter(:,k),P_filter(:,:,k));
    end
    
    for k = 1:noGSFmodels
        %Predict
        sigma_meas = zeros(noMeas,2*na+1,noGSFmodels);
        for j = 1:2*na+1
            [sigma(:,j,k)]    = UKFpredict(sigma(:,j,k),dt,IMU_meas(i-1,:),IMU_meas(i,:),IMU_noise);
            sigma_meas(:,j,k) = sigma(1:2,j,k);
        end
        
        x_UKF     = zeros(na,1);
        P_UKF     = zeros(na,na);
        
        for j = 1:2*na+1
            x_UKF = x_UKF + W_UKF(j,k)*sigma(:,j,k);
        end
        
        for j = 1:2*na+1
            P_UKF = P_UKF + W_UKF(j,k)*[sigma(:,j,k) - x_UKF]*[sigma(:,j,k) - x_UKF]';
        end
        
        P_UKF = P_UKF + Q;
        
        X_filter(:,k)   = x_UKF;
        P_filter(:,:,k) = P_UKF;
    end
    
    %Update
    if timeVec(i) == GPS_meas(idx,1)
        obsTime(idx-1) = timeVec(i);
        for k = 1:N
            Pxz     = zeros(na,noMeas);
            Pzz     = zeros(noMeas,noMeas);
            ymeas   = zeros(noMeas,1);
            
            for j = 1:2*na+1
                ymeas = ymeas + W_UKF(j,k)*sigma_meas(:,j,k);
            end
            
            for j = 1:2*na+1
                Pxz = Pxz + W_UKF(j,k)*[sigma(:,j,k)- x_UKF]*[sigma_meas(:,j,k)-ymeas]';
                Pzz = Pzz + W_UKF(j,k)*[sigma_meas(:,j,k)-ymeas]*[sigma_meas(:,j,k)-ymeas]';
            end
            
            Scov(:,:,k)             = R + Pzz;
            k_UKF                   = Pxz*inv(Scov(:,:,k));
            measurement             = GPS_meas(idx,2:3)';
            innovation(:,k)         = measurement - ymeas;
            correction              = k_UKF*innovation(:,k);
            x_UKF                   = x_UKF + correction;
            P_UKF                   = P_UKF - k_UKF*Scov(:,:,k)*k_UKF';
            
            S_mat(:,idx-1,k)        = diag(Scov(:,:,k));
            nu_mat(:,idx-1,k)       = innovation(:,k);
            
            X_filter(:,k)           = x_UKF;
            P_filter(:,:,k)         = P_UKF;
            
        end
        
        total_w = 0;
        
        for k = 1:N
            %Normal distributions N(nu,0,S)
            normDist(k)     = GaussianDensity(innovation(:,k), 0, Scov(:,:,k));
            newWeight(k)    = normDist(k)*W_GSF(k);
            total_w         = newWeight(k) + total_w;
        end
        W_GSF               = newWeight/total_w;
        idx                 = idx + 1;
        
    end
    
    X_GSF       = zeros(3,1);
    P_GSF       = zeros(3,3);
    for k = 1:N
        X_GSF   = X_GSF + W_GSF(k)*X_filter(:,k);
    end
    
    for k = 1:N
        P_GSF   = P_GSF + W_GSF(k)*(P_filter(:,:,k)+((X_filter(:,k)-X_GSF)*(X_filter(:,k)-X_GSF)'));
    end
    
    state_out(:,i) = X_GSF;
    cov_out(:,i) = diag(P_GSF);
    weights_out(:,i)  = W_GSF;
end
elapsedTime = toc;

if plotStates == 1
    plotCovs = 1;
    plotFilterStates(state_out,timeVec,truthDataNav,'GSF - UKF',plotCovs,S_mat,nu_mat,obsTime)
end

save('GSF_UKF_Data','timeVec','truthDataNav','state_out','cov_out','obsTime','S_mat','nu_mat','elapsedTime')