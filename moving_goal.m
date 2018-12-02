clc 
% Parameters
conv_factor = 0.05; % Use 0.05 for real, and 5 for synthetic
N = 1548 ; % number of waypoints
K = 10; % number of noisy trajectories around the current best estimate of trajectory
c_d = 0.11; % drag coefficient
vel_max = 3.5 ; % in m/s - this is the maximum velocity the motors can provide
envi_type = 'real'; % specify the type of environment 'real'
count = 0; count_var = 0; %  to know the status of the code
conv = 1;
s_p = 1000;

switch envi_type 
    case 'synthetic' 
        % Grid points
        [X,Y,U,V] = test;
        % specifying the start and goal locations
        X_loc = X(1,:)';
        Y_loc = Y(:,1);
        start_loc = [X_loc(10);Y_loc(10)];
        goal_loc = [X_loc(40);Y_loc(45)];
    case 'real'
        [X,Y, U,V] = environment; % gives a 45x1 X_loc and 39x1 Y_loc
        X = conv_factor *X;
        Y = conv_factor *Y;
        X_loc = X(:,1); % x cordinate of all grid points 
        Y_loc = Y(1,:)';  % y cordinate of all grid points
        start_loc = [X_loc(6);Y_loc(6)];
        goal_loc = [X_loc(40);Y_loc(34)];
end

% displaing information realted to trajectories
dist_init = norm(start_loc - goal_loc);
temp1 = ['The length of initial trajectory is : ',num2str(dist_init),' m'];
disp(temp1)
temp2 = ['Number of waypoints = ',num2str(N)];
disp(temp2)


% visualizing the environment
figure
quiver(X,Y,U,V);
hold on;
% plot([start_loc(1),goal_loc(1)],[start_loc(2),goal_loc(2)],'-r','LineWidth',2)
% hold on
% scatter([start_loc(1),goal_loc(1)],[start_loc(2),goal_loc(2)],'o','k','filled');
% hold on;

vel_ocean_max = max(max(sqrt(U.^2+V.^2)));
X_start = start_loc;
X_traj = zeros(2,N);
X_traj(:,1) = X_start;
scatter(X_traj(1,1), X_traj(2,1), 'o', 'filled');
hold on;
scatter(goal_loc(1), goal_loc(2), '*');
hold on;

temp4 = 5;
energy_cost = zeros(N,1);
h = animatedline;
g = animatedline;
for i = 2:N
    vel_ocean = find_ocean_vel(X_start(1),X_start(2),U,V,X_loc,Y_loc); % finding velocity at previous waypoint 
    ss = (norm(vel_ocean)+vel_max)/temp4; % step-size -- requires tunining
    lamda_t = (i/N);%*(exp(-norm(X_traj(:,i-1) - goal_loc))/(1 + exp(-norm(X_traj(:,i-1) - goal_loc)))); % lamda(t) 
    if i == 2
        X_grad = (lamda_t*(X_start-goal_loc)) + ((1-lamda_t)*(-vel_ocean)); 
    else
        vel_abs_norm = norm(X_start-X_traj(:,i-2));
        vel_abs = X_start-X_traj(:,i-2);
%         X_grad = (2*lamda_t*(X_start-goal_loc))+((1-lamda_t)*(-vel_ocean+(exp(vel_max-temp)/temp)*temp3)); %  initial 
%         X_grad = (2*lamda_t*(X_start-goal_loc))+((1-lamda_t)*(-vel_ocean+(exp(vel_max-temp)/(temp*(1+exp(vel_max-temp))))*temp3)); % soft-max
%         X_grad = (lamda_t*(X_start-goal_loc))+((1-lamda_t)*(-vel_ocean + (-((cd*norm(vel_abs-vel_ocean)^3)*(vel_abs - vel_ocean)/norm(vel_abs - vel_ocean))-(exp(vel_max-vel_abs_norm)/(vel_abs_norm*(1+exp(vel_max-vel_abs_norm))))/norm(exp(vel_max-vel_abs_norm)/(vel_abs_norm*(1+exp(vel_max-vel_abs_norm))))*vel_abs))); % soft-max
% X_grad = (lamda_t*(X_start-goal_loc)/norm(X_start-goal_loc))+((1-lamda_t)*(-vel_ocean-(exp(vel_max-vel_abs_norm)/(vel_abs_norm*(1+exp(vel_max-vel_abs_norm))))*vel_abs)/norm((-vel_ocean-(exp(vel_max-vel_abs_norm)/(vel_abs_norm*(1+exp(vel_max-vel_abs_norm))))*vel_abs))); % soft-max
        X_grad = (lamda_t*(X_start-goal_loc)/norm(X_start-goal_loc))+((1-lamda_t)*(-vel_ocean-(exp(vel_max-vel_abs_norm)/(vel_abs_norm*(1+exp(vel_max-vel_abs_norm))))*vel_abs)/norm((-vel_ocean-(exp(vel_max-vel_abs_norm)/(vel_abs_norm*(1+exp(vel_max-vel_abs_norm))))*vel_abs))); % soft-max
    if (norm(vel_abs - vel_ocean) <= vel_max && vel_abs_norm >= vel_max)
        c_s = 0;
    elseif (norm(vel_abs - vel_ocean) <= vel_max)
        c_s = exp(vel_max - norm(vel_abs - vel_ocean));
    elseif (norm(vel_abs - vel_ocean) >= vel_max)
        c_s = s_p + (norm(vel_abs - vel_ocean) - vel_max)*(s_p^2);
    end
    energy_cost(i) = 0.01*(c_d*norm(vel_abs-vel_ocean)^3);
    end
    X_start = X_start - ss*X_grad;
    X_traj(:,i) = X_start;
    temp4 = norm(X_grad);
    addpoints(g, goal_loc(1), goal_loc(2));
    drawnow;
    addpoints(h, X_traj(1,i), X_traj(2,i));
    drawnow;
    goal_loc = goal_loc + ((-1)^i)*50*rand(2,1);
    
end

energy_OGD = sum(energy_cost(:))
% scatter([X_traj(1,1),X_traj(1,N)],[X_traj(2,1),X_traj(2,N)],'o','g','filled');
% hold on;
% plot(X_traj(1,:),X_traj(2,:),'-g','LineWidth',2);
% hold on;
% [energy_EESTO, opt_EESTO] = EESTO;
% plot(opt_EESTO(:,1),opt_EESTO(:,2),'-y','LineWidth',2);
% hold on; 
% % scatter(opt_EESTO([1,end],1),opt_EESTO([1,end],2),'o','k','filled');
% % hold on;
% [energy_STOMP, opt_STOMP] = STOMP;
% plot(opt_STOMP(:,1),opt_STOMP(:,2),'-c','LineWidth',2);
% hold on; 
% % scatter(opt_STOMP([1,end],1),opt_STOMP([1,end],2),'o','k','filled');
% % hold on;
% temp1 = ['Energy comparison : EESTO = ',num2str(energy_EESTO)];
% disp(temp1)
% temp2 = ['Energy comparison : STOMP = ',num2str(energy_STOMP)];
% disp(temp2)
% temp3 = ['Energy comparison : OGD = ',num2str(energy_OGD)];
% disp(temp3)
% legend('Curents', 'Initial', 'OGD', 'EESTO', 'STOMP');


















