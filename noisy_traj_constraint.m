% Function for generating noisy trajectories around the optimal trajectory 

%% Direct sampling
% function [loc_x,loc_y] = noisy_traj_K(R_inv,N,K,opt_x,opt_y)
% loc_x = zeros(N,K);
% loc_y = zeros(N,K);
% for i = 1:K
%     loc_x(:,i) = mvnrnd(opt_x,R_inv);
%     loc_y(:,i) = mvnrnd(opt_y,R_inv);
% end
% end

%% Addition
function [loc_x,loc_y,eps_x,eps_y] = noisy_traj_constraint(R_inv,N,K,opt_x,opt_y)
 
loc_x = zeros(N,K);
loc_y = zeros(N,K);
eps_x = zeros(N,K);
eps_y = zeros(N,K);
for i = 1:K
    eps_x(1:N-1,i) = mvnrnd(zeros(N-1,1),R_inv)'.*10^(floor(log10(opt_x(1,1)))-3);
    eps_y(1:N-1,i) = mvnrnd(zeros(N-1,1),R_inv)'.*10^(floor(log10(opt_y(1,1)))-3);
    eps_x(1,i) = 0;
    eps_y(1,i) = 0;
%     eps_x(N,i) = 0;
%     eps_y(N,i) = 0;
    loc_x(:,i) = opt_x + eps_x(:,i);
    loc_y(:,i) = opt_y + eps_y(:,i);
    
%     loc_x(loc_x(:,i)>= max(X_loc),i) = max(X_loc);
%     loc_x(loc_x(:,i)<= min(X_loc),i) = min(X_loc);
%     
%     loc_y(loc_y(:,i)>= max(Y_loc),i) = max(Y_loc);
%     loc_y(loc_y(:,i)<= min(Y_loc),i) = min(Y_loc);
%     
    
end








end



% for i=5:2*(N-1)
%         D(i,i) = 1;
%         D(i, i-4) = 1;
%         D(i, i-2) = -2;
% end
%     D(1,1) = 1; D(2,2) = 1;
%     D(3,1) = -2; D(3,3) = 1;D(4,2) = -2; D(4,4) = 1;
