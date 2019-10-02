function [X_Filter,P_Filter, innovation_store, S_store, X_IMM, P_IMM, modeProbs]= IMMUKFUpdate(X_IMM,P_IMM,IMU_meas,IMU_noise,GPS_meas,GPS_noise,transMatrix,modeProbs,UKF_update,dt,Q)
                                                                                                     
%Determine number of models
m = size(X_IMM,2);

%Number of states
n = size(X_IMM,1);

%Number of measurements
if ~isempty(GPS_meas)
    measurement = GPS_meas(2:3);
end

innovation_store = [];
S_store = [];

%*****Interacting/Mixing*****
c_j = zeros(1,m);
for j = 1:m
    for i = 1:m
        c_j(j) = c_j(j) + transMatrix(i,j)*modeProbs(i);
    end
end

modeProbs_ij = zeros(m,m);
for i = 1:m
    for j = 1:m
        modeProbs_ij(i,j) = 1/c_j(j)*transMatrix(i,j)*modeProbs(i);
    end
end

%Calculate mixed state and covariance
X0j = zeros(n,m);
for j = 1:m
    for i = 1:m
        X0j(:,j) = X0j(:,j) + X_IMM(:,i)*modeProbs_ij(i,j);
    end
end

P0j = zeros(n,n,m);
for j = 1:m
    for i = 1:m
        P0j(:,:,j) = P0j(:,:,j) + modeProbs_ij(i,j)*(P_IMM(:,:,i) + (X_IMM(:,i)-X0j(:,j))*(X_IMM(:,i)-X0j(:,j))');
    end
end

for i = 1:m
    [W_UKF,sigma] = unscentedTransform(X0j(:,i),P0j(:,:,i));
        
%     sigma_meas = zeros(noMeas,2*na+1,noGSFmodels);
    for j = 1:2*n+1
        [sigma(:,j)]    = UKFpredict(sigma(:,j),dt,IMU_meas(1,:),IMU_meas(2,:),IMU_noise);
        sigma_meas(:,j,i) = sigma(1:2,j);
    end
    
    x_UKF   = zeros(n,1);
    P_UKF   = zeros(n,n);
    
    for j = 1:2*n+1
        x_UKF = x_UKF + W_UKF(j)*sigma(:,j);
    end
    
    for j = 1:2*n+1
        P_UKF = P_UKF + W_UKF(j)*[sigma(:,j) - x_UKF]*[sigma(:,j) - x_UKF]';
    end
    
    P_UKF = P_UKF + Q;
    
    X0j(:,i)   = x_UKF;
    P0j(:,:,i) = P_UKF; 
    
    if UKF_update == 1
        %Perform Kalman filter updates on each filter and calculate likelihoods
        obsStates           = length(measurement);
        S_store             = zeros(obsStates,m);
        innovation_store    = zeros(obsStates,m);
        
        Pxz = zeros(n,obsStates);
        Pzz = zeros(obsStates,obsStates);
        ymeas     = zeros(obsStates,1);
        
        for j = 1:2*n+1
            ymeas = ymeas + W_UKF(j)*sigma_meas(:,j);
        end
        
        for j = 1:2*n+1
            Pxz = Pxz + W_UKF(j)*[sigma(:,j)- x_UKF]*[sigma_meas(:,j)-ymeas]';
            Pzz = Pzz + W_UKF(j)*[sigma_meas(:,j)-ymeas]*[sigma_meas(:,j)-ymeas]';
        end
        R           = diag(GPS_noise);
        Scov        = R + Pzz;
        k           = Pxz*inv(Scov);
        measurement = GPS_meas(2:3)';
        innovation  = measurement - ymeas;
        correction  = k*innovation;
        X0j(:,i)    = X0j(:,i) + correction;
        P0j(:,:,i)  = P0j(:,:,i) - k*Scov*k';
        
        lamda(i)        = GaussianDensity(measurement, ymeas, Scov);
        
        S_store(:,i)            = diag(Scov);
        innovation_store(:,i)   = innovation;
    end
    
end

if UKF_update == 1
    %Mode probability update
    c = 0;
    for j = 1:m
        c = c + lamda(j) * c_j(j);
    end
    
    for j = 1:m
        modeProbs(j) = 1/c * lamda(j) * c_j(j);
    end
end

X_IMM = X0j;
P_IMM = P0j;

%Estimate and Covariance Combination
X_Filter = zeros(n,1);
for j = 1:m
    X_Filter = X_Filter + X_IMM(:,j) * modeProbs(j);
end

P_Filter = zeros(n,n);
for j = 1:m
    P_Filter = P_Filter +  modeProbs(j) * (P_IMM(:,:,j) + (X_IMM(:,j)-X_Filter)*(X_IMM(:,j)-X_Filter)');
end