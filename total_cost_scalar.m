function [tot_cost,C_e] = total_cost_scalar(opt_traj,R,U,V,X_loc,Y_loc,w_e,w_s,speed_pen,c_d,T,vel_max,h,N)
[S_theta,~,C_e] = total_cost_constraint(opt_traj(:,1),opt_traj(:,2),U,V,X_loc,Y_loc,w_e,w_s,speed_pen,c_d,T,vel_max,h); % Function defined by me  

tot_cost = sum(S_theta) + (opt_traj(1:N-1,1)'*R*opt_traj(1:N-1,1)) + (opt_traj(1:N-1,2)'*R*opt_traj(1:N-1,2)); 
C_e = sum(C_e);

end