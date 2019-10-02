function [X_GSF, P_GSF, W_GSF, W_UKF, sigma] = initialiseGSFUKF(x_init,P_init,N)

numberOfModels  = N;
D2R             = pi/180;
Vn_0            = x_init(1);
Ve_0            = x_init(2);

P_Filter        = P_init;

X_GSF           = zeros(3,numberOfModels);
increment       = (1*pi)/(N-1);

P_GSF           = zeros(3,3,numberOfModels);

for i = 1:N
    X_GSF(:,i)      = [Vn_0;Ve_0;(i-1)*increment];
    P_GSF(:,:,i)    = P_Filter;
    W_GSF(i)        = 1/(numberOfModels);
end