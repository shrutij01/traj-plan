function [X,Y, U,V] = environment

%converting the distances between every consecutive pair of points in the
%dataset to UTM  

% ncfile = 'ca_subSCB_das_2017121621.nc';
% ncdisp(ncfile)

ncfile = 'ca_subSCB_das_2017121621.nc';
% ncdisp(ncfile)
lat = ncread(ncfile, 'lat');
lon = ncread(ncfile, 'lon');
u = ncread(ncfile, 'u');
v = ncread(ncfile, 'v');

L_lat = length(lat); 
L_lon = length(lon);

dlat = zeros(L_lat-1,1);
alat = zeros(L_lat-1,1);
rlat = zeros(L_lat-1,1);
latutm = zeros(1,L_lat);
latutm(1,1) = 0;

% indexing such that current stores diff b/w next and curr. CORRECT!

for i =2:L_lat
    dlat(i-1,1) = abs(lat(i)-lat(i-1));
    alat(i-1,1) = (lat(i)+lat(i-1))/2;
    rlat(i-1,1) = alat(i-1,1) * pi/180;
    m = 111132.09  - 566.05 * cos(2 * rlat(i-1,1)) + 1.2 * cos(4 * rlat(i-1,1));
    dy = dlat(i-1,1)*m;
    latutm(1,i) = latutm(1,i-1) + dy;
end
%decide if want to store the dlat, rlat values globally
lonutm = zeros(L_lon,L_lat-1);
lonutm(1,:) = 0;
dlon = zeros(L_lon-1,1);
for i=2:L_lon
    for j=1:L_lat-1 
        p = 111415.13 * cos(rlat(j,1)) - 94.55 * cos(3 * rlat(j,1));
        dlon(i-1,1) = abs(lon(i)-lon(i-1));
        dx = dlon(i-1,1)*p;
        lonutm(i,j) = lonutm(i-1,j) + dx;
    end
end
%points very far away, need to scale down the values of lonutm and latutm
%to form a reasonably sized grid
m = 45;
nu = 74;
nl = 36;

y = zeros(m,nu-nl+1);
for i = 1: m
    y(i,:) = latutm(:,nl:nu);
end    
    
x = zeros(m,nu-nl+1);
for i = nl:nu
    x(:,i-nl+1) = lonutm(1:m,i);
end
X = x;
Y = y;
% [X,Y] = meshgrid(x,y);
U = -u(1:m,nl:nu,2);
V = -v(1:m,nl:nu,2);
% figure
% quiver(x,y,U,V);

% figure
% quiver(repmat(lon,1,110),repmat(lat(1:110)',211,1),u(:,1:110,2),v(:,1:110,2))
