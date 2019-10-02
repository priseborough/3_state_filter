function [W,sigma] = unscentedTransform(x_init,P_init) 

%Unscented Transform
na          = length(x_init);
kappa       = 1e-3;
sigma       = zeros(na,2*na+1);
W           = zeros(2*na+1,1);
sigma(:,1)  = x_init;
W(1,1)      = kappa / (na+kappa);
matSqrt     = chol((na+kappa)*P_init);

for i = 2:2*na+1
    if i <= (na + 1)
        sigma(:,i) = sigma(:,1) + matSqrt(i-1,:)';
    else
        sigma(:,i) = sigma(:,1) - matSqrt(i-1-na,:)';
    end
    W(i,1)      = 1 / (2*(na+kappa));
end