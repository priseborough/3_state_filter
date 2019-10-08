function [X_GSF, P_GSF, w_GSF] = initialiseGSFEKF(x_init,P_init,N)

numberOfModels  = N;
D2R             = pi/180;
Vn_0            = x_init(1);
Ve_0            = x_init(2);

P_Filter        = P_init;

X_GSF           = zeros(3,numberOfModels);
increment       = (2*pi)/(N-1); % Changed to 0 -> 360 degrees. This change breaks something

P_GSF           = zeros(3,3,numberOfModels);

for i = 1:N
    X_GSF(:,i)      = [Vn_0;Ve_0;(i-1)*increment];
    P_GSF(:,:,1)    = P_Filter;
    w_GSF(i)        = 1/(numberOfModels);
end