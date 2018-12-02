function [energy_EESTO, opt_traj] = EESTO
%% Synthetic Data
clc 
% Parameters
conv_factor = 0.05; % Use 0.05 for real, and 5 for synthetic
N = 1548 ; % number of waypoints
tm_init = 1;  % initial constant temporal spacing b/w waypoints in seconds 
K = 20; % number of noisy trajectories around the current best estimate of trajectory
conv_cost = 3*10^7; % inital condition on convergence criteria for the cost function
temp = 1; % temporary variable
w_e = 0.5; % weightage of energy cost
w_s = 0.5; % weightage of speed cost
w_o = 0; % weightage of obstacle cost
energy_penalty = 1000; %energy penalty
speed_pen = 1000; % penalty introduced when the motors are required to provide a speed they cannot achieve
obstacle_penalty = 10000; % penalty imposed when a waypoint belongs to region of obstracles  
c_d = 0.11; % drag coefficient
vel_max = 8.5; % in m/s - this is the maximum velocity the motors can provide
h = 10 ; % this regulates the sensitivity of the exponetiated cost
envi_type = 'real'; % specify the type of environment 'real'
count = 0; count_var = 0; %  to know the status of the code
conv = 1;

switch envi_type 
    case 'synthetic' 
        % Grid points
        [X,Y,U,V] = test;
        % specifying the start and goal locations
        X_loc = X(1,:)';
        Y_loc = Y(:,1);
        start_loc = [X_loc(10),Y_loc(10)];
        goal_loc = [X_loc(40),Y_loc(45)];
    case 'real'
        [X,Y, U,V] = environment;
        X = conv_factor *X;
        Y = conv_factor *Y;
        X_loc = X(:,1); % x cordinate of all grid points 
        Y_loc = Y(1,:)';  % y cordinate of all grid points
        start_loc = [X_loc(6),Y_loc(6)];
        goal_loc = [X_loc(40),Y_loc(34)];
        
end           

%% Generating the initial optimal trajectory
% dist_init = norm(start_loc - goal_loc);
% temp1 = ['The length of initial trajectory is : ',num2str(dist_init),' m'];
% disp(temp1)
% temp2 = ['Number of waypoints = ',num2str(N)];
% disp(temp2)

% sampling initial waypoints on the line (joining start and goal location) that are equidistant 
opt_traj = zeros(N,2);
[opt_traj(:,1),opt_traj(:,2)] = sample_line(start_loc,goal_loc,N);    % Function defined by me
opt_traj_init = opt_traj; 

%% Generating the initial time matrix
T = zeros(N-1,N-1);
for i=1:N-1
    T(i,i) = 1/tm_init^2;
end
  
%% Finding the optimal trajectory 
A = zeros(N-1,N-1); % Finite differencing matrix 
%should be symmetric for the covariance to be symmetric
for i=2:N-2
    A(i, i-1) = 1;
    A(i,i) = -2;
    A(i, i+1) = 1;
end
A(1,1) = -2; A(1,2) = 1;
A(N-1,N-2) = -2;  A(N-1,N-1) = 1;

D = T*A;
D_new = inv(D'*D);
    
R_in = D'*D; % PSD matrix that defines control costs 
R_in_inv = D_new; % Covariance matrix 
% R = R_in; R_inv = R_in_inv; %To run STOMP
[total_cost_prev,energy_cost] = total_cost_scalar(opt_traj_init,R_in,U,V,X_loc,Y_loc,w_e,w_s,speed_pen,c_d,T,vel_max,h,N); % Function defined by me

while ~(abs(conv_cost) < .5*10^0)  
    count = count+1;    %print
    temp = ['Steps towards convergence ',num2str(count)]; disp(temp);
    % 1) Generating K noisy trajcetories around the optimal trajectory 
    [T,R,R_inv,M,eps_t] = temporal_constraint(T,N,A); 
    R_inv = nearestSPD(R_inv);
%     [theta_x,theta_y,eps_x,eps_y] = noisy_traj_constraint(R_inv,N,K,opt_traj(:,1),opt_traj(:,2));  % Function defined by me
theta_x = zeros(N,K);
theta_y = zeros(N,K);
eps_x = zeros(N,K);
eps_y = zeros(N,K);
for i = 1:K
    try 
        eps_x(1:N-1,i) = mvnrnd(zeros(N-1,1),R_inv)'.*10^(floor(log10(opt_traj(1,1)))-3.8);
    catch
        keyboard
    end
    eps_y(1:N-1,i) = mvnrnd(zeros(N-1,1),R_inv)'.*10^(floor(log10(opt_traj(1,2)))-3.8);
    eps_x(1,i) = 0;
    eps_y(1,i) = 0;
%     eps_x(N,i) = 0;
%     eps_y(N,i) = 0;
    theta_x(:,i) = opt_traj(:,1) + eps_x(:,i);
    theta_y(:,i) = opt_traj(:,2) + eps_y(:,i);
end

%     if temp == 1
%         % viusalizing the sampled trajectories just for the first iteration
%         figure
%         plot(opt_traj_init(:,1),opt_traj_init(:,2));
%         hold on
%         plot(theta_x,theta_y );
%         title('K = 10 : Noisy trajectory samples')
%         temp = temp+1;
%     end    
%     
    % 2) finding the cost at each waypoint over all trajectories and the value of the objective function
    [S_theta,P_theta,~] = total_cost_constraint(theta_x,theta_y,U,V,X_loc,Y_loc,w_e,w_s,speed_pen,c_d,T,vel_max,h); % Function defined by me  
    % Note the dimensions : S_theta -- (N-2 x K) , P_theta -- (N x K) 
    
% % 3) finding the change in each waypoint for STOMP
%     delta_theta_x = zeros(N,1);
%     delta_theta_y = zeros(N,1);
%     for j = 1:N
%         delta_theta_x(j) =  eps_x(j,:)*P_theta(j,:)';
%         delta_theta_y(j) =  eps_y(j,:)*P_theta(j,:)';
%     end
%     
%     % 4) computing the update 
%     delta_theta_x = M*delta_theta_x;
%     delta_theta_y = M*delta_theta_y;
%     delta_theta_x([1,N]) = 0;
%     delta_theta_y([1,N]) = 0; 

%Use this part when using temporal_constraint for EESTO
    delta_theta_x = zeros(N,1);
    delta_theta_y = zeros(N,1);
    for j = 1:N
        delta_theta_x(j) =  eps_x(j,:)*P_theta(j,:)';
        delta_theta_y(j) =  eps_y(j,:)*P_theta(j,:)';
    end
    
    % 4) computing the update
    delta_theta_x(1:N-1,1) = M*delta_theta_x(1:N-1,1);
    delta_theta_y(1:N-1,1) = M*delta_theta_y(1:N-1,1);
    delta_theta_x([1,N]) = 0;
    delta_theta_y([1,N]) = 0;
    
    % 5) perform the update
    opt_traj_temp = zeros(size(opt_traj));
    opt_traj_temp(:,1) = opt_traj(:,1) + delta_theta_x;
    opt_traj_temp(:,2) = opt_traj(:,2) + delta_theta_y;
     
    % 6) computing the difference in successive trajectory cost 
    [total_cost_curr,energy_cost] = total_cost_scalar(opt_traj_temp,R,U,V,X_loc,Y_loc,w_e,w_s,speed_pen,c_d,T,vel_max,h,N);
    conv_cost = total_cost_prev - total_cost_curr;
   
    if conv_cost >=0
        opt_traj = opt_traj_temp;
        total_cost_prev = total_cost_curr;
        count_var = 0;
    else
        count_var = count_var+1;  %print
        temp2 = ['No improvement :(.. Reworking :) .. ', num2str(count_var)]; disp(temp2);
        if count_var > 20
            break
        end
    end
end

[ocean_x, ocean_y] = vel_ocean_opt(opt_traj(:,1), opt_traj(:,2), U, V, X_loc, Y_loc);
E_c = zeros(N,1);
for i = 1:N-1
    E_c(i) =  0.01*(c_d *norm(([opt_traj(i,1);opt_traj(i,2)] - [opt_traj(i+1,1);opt_traj(i+1,2)])/T(i,i) - [ocean_x(i);ocean_y(i)])^3)*T(i,i);
end
energy_EESTO = sum(E_c);

%% Visulaizing the resultant energy efficient trajectory : using STOMP 
figure
quiver(X,Y,U,V);
hold on;
plot([start_loc(1),goal_loc(1)],[start_loc(2),goal_loc(2)],'-r','LineWidth',2)
hold on;
plot(opt_traj(:,1),opt_traj(:,2),'-g','LineWidth',2);
hold on; 
scatter(opt_traj([1,end],1),opt_traj([1,end],2),'o','k','filled');
hold on;
[energy_STOMP, opt_STOMP] = STOMP;
plot(opt_STOMP(:,1),opt_STOMP(:,2),'-y','LineWidth',2);
hold on; 
scatter(opt_STOMP([1,end],1),opt_STOMP([1,end],2),'o','k','filled');
temp1 = ['Convergence criteria : ',num2str(abs(conv_cost))];
disp(temp1)
temp1 = ['Energy comparison : EESTO = ',num2str(energy_cost)];
disp(temp1)
temp2 = ['Energy comparison : STOMP = ',num2str(energy_STOMP)];
disp(temp2)

end
% 









