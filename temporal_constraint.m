function [T,R, R_inv, M,eps_t] = temporal_constraint(T,N,A)

% N = 20;
% t_in = 1;
% T = zeros(N-1,N-1);
% for i=1:N-1
%     T(i,i) = 1/t_in^2;
% end
% A = zeros(N-1,N-1); % Finite differencing matrix 
% %should be symmetric for the covariance to be symmetric
% for i=2:N-2
%     A(i, i-1) = 1;
%     A(i,i) = -2;
%     A(i, i+1) = 1;
% end
% A(1,1) = -2; A(1,2) = 1;
% A(N-1,N-2) = -2;  A(N-1,N-1) = 1;

eps_t = zeros(N-1,1);
eps_t(:,1) = mvnrnd(zeros(N-1,1),inv(T))';
for n = 1:N-1
    T(n,n) = 1/(abs(1/(sqrt(T(n,n))) + 0.1*eps_t(n,1)))^2;
end

D = T*A;
R = transpose(D)*D;
R = nearestSPD(R);
R_inv = inv(R);
M = R_inv./((N-1)*repmat(max(R_inv),N-1,1));

end