function simulateUKF(x_init,P_init,timeVec,dt,IMU_meas,IMU_noise,GPS_meas,Q,truthDataNav,plotStates)

Ndata       = length(timeVec);
x_UKF       = x_init;
P_UKF       = P_init;
state_out   = zeros(3,Ndata);
cov_out     = zeros(3,Ndata);
idx         = 2;

state_out(:,1) = x_init;
cov_out(:,1) = diag(P_init);

%Noise matrices
R           = diag(IMU_noise);
Q           = Q*1;

%Lengths
N           = 2; %no of measurements
na          = length(x_init); %no of states

%Define matries
obsTime     = zeros(1,length(GPS_meas)-1);
S_mat       = zeros(2,length(GPS_meas)-1);
nu_mat      = zeros(2,length(GPS_meas)-1);

tic;
for i = 2:Ndata
    [W,sigma] = unscentedTransform(x_UKF,P_UKF);
    %Predict
    sigma_meas = zeros(N,2*na+1);
    for j = 1:2*na+1
        [sigma(:,j)]    = UKFpredict(sigma(:,j),dt,IMU_meas(i-1,:),IMU_meas(i,:),IMU_noise);
        sigma_meas(:,j) = sigma(1:2,j);
    end
    
    x_UKF     = zeros(na,1);
    P_UKF     = zeros(na,na);
    
    for j = 1:2*na+1
        x_UKF = x_UKF + W(j)*sigma(:,j);
    end
    
    for j = 1:2*na+1
        P_UKF = P_UKF + W(j)*[sigma(:,j) - x_UKF]*[sigma(:,j) - x_UKF]';
    end
    
    P_UKF = P_UKF + Q;
    
    %Update
    if timeVec(i) == GPS_meas(idx,1)
        Pxz         = zeros(na,N);
        Pzz         = zeros(N,N);
        ymeas       = 0;
        
        for j = 1:2*na+1
            ymeas = ymeas + W(j)*sigma_meas(:,j);
        end
        
        for j = 1:2*na+1
            Pxz = Pxz + W(j)*[sigma(:,j)- x_UKF]*[sigma_meas(:,j)-ymeas]';
            Pzz = Pzz + W(j)*[sigma_meas(:,j)-ymeas]*[sigma_meas(:,j)-ymeas]';
        end
        
        measurement         = GPS_meas(idx,2:3)';
        obsTime(idx-1)      = timeVec(i);
        Scov                = R + Pzz;
        innovation          = measurement - ymeas;
        k                   = Pxz*inv(Scov);
        correction          = k*innovation;
        x_UKF               = x_UKF + correction;
        P_UKF               = P_UKF - k*Scov*k';
        
        S_mat(:,idx-1)      = diag(Scov);
        nu_mat(:,idx-1)     = innovation;
        
        idx = idx + 1;
        
    end
    
    state_out(:,i) = x_UKF;
    cov_out(:,i) = diag(P_UKF);
    
end
elapsedTime = toc;

if plotStates == 1

plotCovs = 1;
plotFilterStates(state_out,timeVec,truthDataNav,'UKF',plotCovs,S_mat,nu_mat,obsTime)

end

save('UKF_Data','timeVec','truthDataNav','state_out','cov_out','obsTime','S_mat','nu_mat','elapsedTime')