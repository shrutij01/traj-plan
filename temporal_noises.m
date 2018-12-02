function [T_noisy,R,R_inv,M,eps_t] = temporal_noises(T,N,K,A)
% K = 10;
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

D = zeros(N-1,N-1,K);
E = zeros(N-1,N-1,K);
R = zeros(N-1,N-1,K);

R_inv = zeros(N-1,N-1,K);
T_noisy = zeros(N-1,N-1,K);
eps_t = zeros(N-1,K);
U = zeros(N-1,N-1,K);
% clu=zeros(K,1);
for k=1:K
    eps_t(:,k) = mvnrnd(zeros(N-1,1),inv(T))';
    for n = 1:N-1
        T_noisy(n,n,k) = 1/(abs(1/(sqrt(T(n,n))) + 0.1*eps_t(n,k)))^2;
    end
    D(:,:,k) = T_noisy(:,:,k)*A;
    E(:,:,k) = transpose(D(:,:,k));
%     E = permute(D,[2 1 3]);
    R(:,:,k) = E(:,:,k)*D(:,:,k); % PSD matrix that defines control costs 
    U(:,:,k) = inv(R(:,:,k)); %covariance mat E(:,:,k)*D(:,:,k)
    R_inv(:,:,k) = triu(U(:,:,k));
    R_inv(:,:,k) = R_inv(:,:,k)'+R_inv(:,:,k);

    M = R_inv./((N-1)*repmat(max(R_inv),N-1,1)); % Smoothing the noisy updates
end
% t=zeros(K,1);
% for k=1:K
%     t(k) = issymmetric(R_inv(:,:,k));
% end
% t
 end





    


