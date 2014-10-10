function [t_traj,x_traj,y_traj,z_traj] = elip_gen(p,sa,si,n_points,traj_speed,yaw_rot,pitch_rot,roll_rot)
% Inputs:
% P - Ellipse Center Coordinates in North, East, Up m (1x3)
% sa - Ellispe major axis in m
% si - Ellipse minor axis in m
% n_points - number of points in the trajectory unitless
% traj_speed - velocity of trajectory in m/s
% yaw_rot - yaw rotation of the ellipse in rads
% pitch_rot - pitch rotation of the ellipse in rads 
% roll_rot - roll rotation of the ellipse in rads
%
% Outputs:
% t_traj - time hisotyr for trajectory in s
% x_traj - North trajectory coordinates in m 
% y_traj - East trajectory coordinates in m
% z_traj - Altitude trajectory coordinates in m (positive up)
% Ex Inputs:
% n_points = 100000;
% p = [500,200,150];
% sa = 400;
% si = 250;
% traj_speed = 20;
% yaw_rot = 0*pi/180;
% pitch_rot = 0*pi/180;
% roll_rot = 0*pi/180;
%
% Make theta vector
theta_deg = linspace(0,360,n_points);
theta_rad = theta_deg*pi/180;
% Calculate Radius
r = sa*si./(sqrt((si*cos(theta_rad)).^2+(sa*sin(theta_rad)).^2));
% Calculate Roation Matricies
R_yaw = [cos(yaw_rot) sin(yaw_rot) 0; -sin(yaw_rot) cos(yaw_rot) 0; 0 0 1];
R_pitch = [cos(pitch_rot) 0 -sin(pitch_rot); 0 1 0; sin(pitch_rot) 0 cos(pitch_rot)]; 
R_roll = [1 0 0; 0 cos(roll_rot) sin(roll_rot); 0 -sin(roll_rot) cos(roll_rot)];
R_321 = R_roll*R_pitch*R_yaw;% roll pitch yaw
% Convert Ellipse polar to cartesian
[x_el,y_el] = pol2cart(theta_rad,r);
% Make the altitude component
z_el = zeros(size(y_el));
% ROTATE!
traj_pos = R_321*[x_el;y_el;z_el];
% Pull them back out
x_el = traj_pos(1,:);
y_el = traj_pos(2,:);
z_el = traj_pos(3,:);
% Add Center Offsets 
x_traj = x_el+p(1);
y_traj = y_el+p(2);
z_traj = z_el+p(3);
% Calculate Estimate for Path Length
path_length = 0;
for i = 1:n_points-1
   d_not = sqrt((x_el(i+1)-x_el(i))^2+(y_el(i+1)-y_el(i))^2+(z_el(i+1)-z_el(i))^2); 
   path_length = path_length+d_not;
end
% Get Ellipse Period from Path length and speed
ellipse_per = path_length/traj_speed;
% Generate Outpu Vector
t_traj = linspace(0,ellipse_per,n_points);
% End