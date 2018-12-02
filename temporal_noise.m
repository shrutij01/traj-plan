% function [T_noisy,R,R_inv,M] = temporal_noise(T,N,K,A)

K = 10;
N = 20;
t_in = 1;
T = zeros(N-1,N-1,K);
for i=1:N-1
    T(i,i,:) = 1/t_in^2;
end
tt=zeros(N-1,K);
t = zeros(N-1,1);
t(:) = t_in;
for k=1:K
    tt(:,k) = abs(t(:) + sqrt(inv(T(:,:,k)))*randn(N-1,1)).^2;
    for n=1:N-1
        T(n,n,k) = 1/tt(n,k);
    end
end
A = zeros(N-1,N-1); % Finite differencing matrix 
%should be symmetric for the covariance to be symmetric
for i=2:N-2
    A(i, i-1) = 1;
    A(i,i) = -2;
    A(i, i+1) = 1;
end
A(1,1) = -2; A(1,2) = 1;
A(N-1,N-2) = -2;  A(N-1,N-1) = 1;

D = zeros(N-1,N-1,K);
E = zeros(N-1,N-1,K);
R = zeros(N-1,N-1,K);
R_inv = zeros(N-1,N-1,K);
for k=1:K
    D(:,:,k) = T(:,:,k)*A;
    E(:,:,k) = transpose(D(:,:,k));
    R(:,:,k) = E(:,:,k)*D(:,:,k); % PSD matrix that defines control costs 
    R_inv(:,:,k) = inv(R(:,:,k)); %covariance mat E(:,:,k)*D(:,:,k)
    M = R_inv./((N-1)*repmat(max(R_inv),N-1,1)); % Smoothing the noisy updates
end
%  end





    


