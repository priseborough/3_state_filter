function simulateParticleFilter(x_init,P_init,timeVec,dt,IMU_data,IMU_noise,GPS_data,GPS_noise,Q,truthDataNav,N,plotStates)

%Define constants
Ndata       = length(timeVec);
idx         = 2;
IMU_noise   = IMU_noise;
nx          = length(x_init);
Nthr        = 0.7*N;
w           = 1/N*ones(N,1);
R           = diag(GPS_noise);

state_out(:,1) = x_init;
P_PF_out(:,1) = diag(P_init);

%Create particles
stdDevPsi  = 2;
stdDevNE   = 1.5;
x_P        = zeros(nx,N);
x_P(1:2,:) = repmat(x_init(1:2),1,N) + stdDevNE*randn(2,N);
x_P(3,:)   = stdDevPsi*randn(1,N);

tic;
%Simulate
for i = 2:Ndata
    prev_x_P = x_P;
    %---Filter via SIS---
    %Draw samples
    for j = 1:N
        x_P(:,j) = PFpredict(prev_x_P(:,j),dt,IMU_data(i-1,:),IMU_data(i,:),IMU_noise);
    end
    
    %Update
    if timeVec(i) == GPS_data(idx,1)
        for j = 1:N
            z_P(:,j) = x_P(1:2,j);
            p_z_x    = GaussianDensity(GPS_data(idx,2:end)', z_P(:,j), R); %likelihood
            w(j)     = w(j) * p_z_x;
        end
        
        w = w/sum(w);
        
        %---Calculate Neff---
        w_sqrd  = w.^2;
        Neff    = 1/sum(w_sqrd);
        
        %---Resample---
        if Neff < Nthr
            [x_P,w,indices] = resamplePF(x_P,w,N);
        end
        idx = idx + 1;
    end
    
    state_out(:,i) = zeros(3,1);
    %
    %     for j = 1:N
    %           state_out(:,i) = state_out(:,i) + w(j)*x_P(:,j);
    %     end
    
    state_out(:,i) = mean(x_P,2);
end
elapsedTime = toc;

if plotStates == 1
    plotCovs = 0;
    plotFilterStates(state_out,timeVec,truthDataNav,'Particle Filter',plotCovs)
end

save('PF_Data','timeVec','truthDataNav','state_out','elapsedTime')