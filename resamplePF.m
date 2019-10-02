function [x,w,ind] = resamplePF(x,w,N)

u = ([0:N-1]+rand(1))/N;
% u = ([0:N-1]+rand(1,N))/N;
wc = cumsum(w);
wc = wc/wc(N);
[dum,ind1] = sort([u,wc']);
ind2 = find(ind1<=N);
ind = ind2 - (0:N-1);
x = x(:,ind);
w = ones(N,1)./N;


% c       = zeros(N,1);
% c(1)    = w(1);
% 
% indices   = 1:N; 
% 
% for i = 2:N
%     c(i) = c(i-1) + w(i);
% end
% 
% u       = zeros(N,1);
% u(1)    = 1/N*rand(1);
% 
% i       = 1;
% 
% for j = 1:N
%     u(j) = u(1) + 1/N * (j-1);
%     
%     while u(j) > c(i)
%         i = i + 1;
%     end
%     
%     x(j) = x(i);
%     w(j) = 1/N;
%     ind(j) = i;   
% end

