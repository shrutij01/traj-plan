function [v_curr] = ocean_information(location_x, location_y, papa,U,V)
if (floor(location_x) ~= location_x)
    nx = [ceil(location_x), ceil(location_x-1)];
else nx = [location_x, location_x];
end
if (floor(location_y) ~= location_y)
    ny = [ceil(location_y), ceil(location_y-1)];
else ny = [location_y, location_y];
end
%The current location may be off-grid. 
 
v_curr(papa,1) = 0.25*(U(nx(1,1),ny(1,1)) + U(nx(1,1),ny(1,2)) + U(nx(1,2),ny(1,1)) + U(nx(1,2),ny(1,2)));
v_curr(papa,2) = 0.25*(V(nx(1,1),ny(1,1)) + V(nx(1,1),ny(1,2)) + V(nx(1,2),ny(1,1)) + V(nx(1,2),ny(1,2)));
%defined the current velocities at the present location 

end
