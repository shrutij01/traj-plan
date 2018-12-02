function [T_updated,R] = T_update(T,N,eps_t,P_theta,A)

delta_t = zeros(N-1,1);
T_updated = zeros(N-1,N-1);
for n=1:N-1
    delta_t(n) = eps_t(n,:)*P_theta(n,:)';
    T_updated(n,n) = 1/(abs(1/(sqrt(T(n,n))) + delta_t(n)))^2;
end
D = T_updated*A;
R = D'*D;
end
