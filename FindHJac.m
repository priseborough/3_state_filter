function [X_Filter, P_Filter, X_IMM, P_IMM, R_Noise, transMatrix, modeProbs] = initialiseIMMEKF(y0,RR,RL)

%X_Filter and P_Filter represent the overall filter output,
%X_IMM and P_IMM represent the individual filter output

numberOfModels  = 4; 

X_Filter        = [y0;RR;RL];
P_Filter        = eye(5);
R_Noise         = 1;%eye(3); %There'll only ever be one measurement in terms of number of models

X_IMM           = zeros(5,numberOfModels); 

X_IMM(:,1)      = [y0;RR;RL];
X_IMM(:,2)      = [y0;0.5*RR;RL];
X_IMM(:,3)      = [y0;RR;0.5*RL];
X_IMM(:,4)      = [y0;0.5*RR;0.5*RL];

P_Filter(1,1)   = P_Filter(1,1)*(0.5)^2; 
P_Filter(2,2)   = P_Filter(2,2)*(0.5)^2;
P_Filter(3,3)   = P_Filter(3,3)*(1/180*pi)^2;
P_Filter(4,4)   = P_Filter(4,4)*(0.5)^2;
P_Filter(5,5)   = P_Filter(5,5)*(0.5)^2;

P_IMM           = zeros(5,5,numberOfModels); 
P_IMM(:,:,1)    = P_Filter;
P_IMM(:,:,2)    = P_Filter;
P_IMM(:,:,3)    = P_Filter;
P_IMM(:,:,4)    = P_Filter;

R_Noise(1,1)    = R_Noise(1,1)*(0.5)^2; 

transMatrix     = [0.97, 0.01, 0.01, 0.01; 0.01, 0.97, 0.01, 0.01; 0.01, 0.01, 0.97, 0.01; 0.01, 0.01, 0.01, 0.97];
modeProbs       = [1/4, 1/4, 1/4, 1/4];

% transMatrix     = [1];
% modeProbs       = [1];
