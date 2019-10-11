function [X_Filter, P_Filter, X_IMM, P_IMM, transMatrix, modeProbs] = initialiseIMMUKF(x_init,P_init,N)

%X_Filter and P_Filter represent the overall filter output,
%X_IMM and P_IMM represent the individual filter output

numberOfModels  = N;
D2R             = pi/180;
Vn_0            = x_init(1);
Ve_0            = x_init(2);

increment       = (2*pi)/N;

na              = length(x_init);

X_Filter        = x_init;
P_Filter        = P_init;

X_IMM           = zeros(na,numberOfModels);

for i = 1:N
    X_IMM(:,i)      = [Vn_0;Ve_0;-pi+increment/2 + (i-1)*increment];
    P_IMM(:,:,i)    = P_Filter;
end


transMatrix_1     = N/1000*ones(numberOfModels,numberOfModels);
transMatrix_2     = diag(1 - N*(N/1000))*eye(numberOfModels,numberOfModels);
transMatrix       = transMatrix_1 - N/1000*eye(numberOfModels,numberOfModels) + transMatrix_2;

modeProbs        = 1/numberOfModels*ones(numberOfModels,1);

% transMatrix     = [1];
% modeProbs       = [1];
