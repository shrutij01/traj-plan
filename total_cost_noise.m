% Function for evaluating the cost for each waypoint for a trajectory 
function [S_theta,P_theta,C_e] = total_cost_noise(theta_x,theta_y,U,V,X_loc,Y_loc,w_e,w_s,s_p,c_d,T,vel_max,h)

N = size(theta_x,1);
K = size(theta_x,2);
P_theta = zeros(N,K);

% Assigning the velocities to new waypoints 
[vel_rel,vel_abs,vel_rel_wo] = find_vel_noise(theta_x,theta_y,U,V,X_loc,Y_loc,T);
% Note : a) 'vel_rel' and 'vel_abs' are of size (N-1 x K).              
%        b) 'vel_rel_wo' is of size (N-2 x K) 

%% Energy cost

% defining the energy asssociated with each waypoint excluding the start and goal loc
E = zeros(N,K); 
for i = 2:N-1
    for j = 1:K
        E(i,j) = (c_d*vel_rel(i-1,j)^3*sqrt(1/T(i,i,j))) + (c_d*vel_rel(i,j)^3*(1/sqrt(T(i,i,j))));
    end
end
 
% defining the energy asssociated with the segement w/o each waypoint, (one at a time)
E_wo = zeros(N,K);
for i = 2:N-1
    for j = 1:K
        E_wo(i,j) = c_d*vel_rel_wo(i-1,j)^3*2*(1/sqrt(T(i,i,j)));
    end
end

% Removing zero rows correspoding to start and goal locations

E([1,N],:) = [];
E_wo([1,N],:) = [];

% Defining the energy cost function
C_e = zeros(N-2,K);
for i = 1:N-2
    for j = 1:K     
%         C_e(i,j) = exp((10^(floor(log10(1/E_wo(1,1))))*(E_wo(i,j) - E(i,j))));
          C_e(i,j) = abs(E_wo(i,j) - E(i,j));
    end
end
% % 

%% Speed cost
C_s = zeros(N-2,K);
for i = 1:N-2
    for j = 1:K     
        if (vel_rel(i+1,j) <= vel_max && vel_abs(i+1,j) >= vel_max )
            C_s(i,j) = 0 ;
        elseif  (vel_rel(i+1,j) <= vel_max)
            C_s(i,j) = exp(vel_max - vel_rel(i+1,j));
        elseif  (vel_rel(i+1,j) >= vel_max)
            C_s(i,j) = s_p + (vel_rel(i+1,j) - vel_max)*(s_p^2);
        end
    end
end

 

%% weighted average of total cost

S_theta = w_e.*C_e + w_s.*C_s ; 
for i = 2:N-1
    for j = 1:K
        temp = (S_theta(i-1,j) - min(S_theta(i-1,:)))/(max(S_theta(i-1,:)) - min(S_theta(i-1,:)));
        P_theta(i,j) =  exp(-h*temp);
    end
end


end
 

