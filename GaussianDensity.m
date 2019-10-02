function normDist = GaussianDensity(x, m, P)

normDist = (det(2*pi*P))^(-0.5)*exp(-0.5*(x-m)'*inv(P)*(x-m));