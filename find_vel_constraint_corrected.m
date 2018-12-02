function [vel_rel,vel_abs,vel_rel_wo] = find_vel_constraint(theta_x,theta_y,U,V,X_loc,Y_loc,T)
% function to assign the velocities to new way points
%
N = size(theta_x,1);
K = size(theta_y,2);

vel_u = zeros(N,K);
vel_v = zeros(N,K);

for i = 1:N
    for j = 1:K

             way_point = [theta_x(i,j);theta_y(i,j)];
             x_limit = [max(find(X_loc <= way_point(1))), min(find(X_loc >= way_point(1)))];
             y_limit = [max(find(Y_loc <= way_point(2))), min(find(Y_loc >= way_point(2)))];
             if (length(x_limit)==2 && length(y_limit)==2) 
                   vel_u(i,j) = 0.25*(U(x_limit(1),y_limit(1)) + U(x_limit(1),y_limit(2)) + U(x_limit(2),y_limit(1)) + U(x_limit(2),y_limit(2))) ;
                   vel_v(i,j) = 0.25*(V(x_limit(1),y_limit(1)) + V(x_limit(1),y_limit(2)) + V(x_limit(2),y_limit(1)) + V(x_limit(2),y_limit(2))) ;
                   
             elseif (length(x_limit)==1 && length(y_limit)==2)
                    vel_u(i,j) = 0.5*(U(x_limit(1),y_limit(1)) + U(x_limit(1),y_limit(2))) ;
                    vel_v(i,j) = 0.5*(V(x_limit(1),y_limit(1)) + V(x_limit(1),y_limit(2))) ;
             elseif (length(x_limit)==2 && length(y_limit)==1)            
                 try
                    vel_u(i,j) = 0.5*(U(x_limit(1),y_limit(1)) + U(x_limit(1),y_limit(1))) ;
                    vel_v(i,j) = 0.5*(V(x_limit(1),y_limit(1)) + V(x_limit(1),y_limit(1))) ;
                 catch 
                     keyboard
                 end
             elseif (way_point(1) > max(X_loc) || way_point(1) < min(X_loc) || way_point(2) > max(y_loc) || way_point(2) < min(y_loc))
                 vel_u(i,j) = Inf;
                 vel_v(i,j) = Inf;
             end
    end
end

vel_rel = zeros(N-1,K,2); % relative velocity at each waypoint over all trajectories
vel_abs = zeros(N-1,K,2); % absolute velocity of the vehicle 
%these velocities are defined as that required to travel to next waypoint
for i = 1:K
    for j = 1:N-1
        rel_dist = ([theta_x(j,i);theta_y(j,i)] - [theta_x(j+1,i);theta_y(j+1,i)]);
        vel_abs(j,i,1) = rel_dist(1)*sqrt(T(j,j));
        vel_abs(j,i,2) = rel_dist(2)*sqrt(T(j,j)); 
        vel_rel(j,i,1) = vel_abs(j,i,1) - vel_u(j,i);
        vel_rel(j,i,2) = vel_abs(j,i,1) - vel_v(j,i);
    end
end

vel_rel_wo = zeros(N-2,K,2); % relative velocity at each waypoint over all trajectories excluding 1 at a time
vel_abs_wo = zeros(N-2,K,2); % absolute velocity of the vehicle 
for i = 1:K
    for j = 1:N-2
        rel_dist_wo  = ([theta_x(j,i);theta_y(j,i)] - [theta_x(j+2,i);theta_y(j+2,i)]); 
        vel_abs_wo(j,i,1) = rel_dist_wo(1)*2*sqrt(T(j,j)); 
        vel_abs_wo(j,i,2) = rel_dist_wo(2)*2*sqrt(T(j,j)); 
        vel_rel_wo(j,i,1) = vel_abs_wo(j,i,1) - vel_u(j,i); 
        vel_rel_wo(j,i,1) = vel_abs_wo(j,i,2) - vel_v(j,i);
    end
end

VR = zeros(N-1, K); VA = zeros(N-1, K);
VR_O = zeros(N-2,K); VA_O = zeros(N-2,K);

for i = 1:N-1
    for j = 1:K
        VR(i,j) = sqrt(vel_rel(i,j,1)^2 + vel_rel(i,j,2)^2);
        VA(i,j) = sqrt(vel_abs(i,j,1)^2 + vel_abs(i,j,2)^2);
    end
end

for i = 1:N-2
    for j = 1:K
        VR_O(i,j) = sqrt(vel_rel_wo(i,j,1)^2 + vel_rel_wo(i,j,2)^2);
        VA_O(i,j) = sqrt(vel_abs_wo(i,j,1)^2 + vel_abs_wo(i,j,2)^2);
    end
end
        


% end