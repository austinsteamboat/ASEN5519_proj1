%=========================================================================
% Author: Drew Ellison
% Date  : 10/2/2014
% Description: Generates command vector [airspeed;turn rate;climb rate]
% All 3 commands are generated based on the 2012 paper "Tracking Expanding
% Star Curves Using Guidance Vector Fields" by Frew and Lawrence. The
% function in this paper generates a field of desired velocities in order
% to converge on some closed pattern. Airspeed and climb rate commands
% result directly from this calculation. Turn rate is calculated by using a
% proportional controller for chi_dot based on the error in the current
% course angle. 
%=========================================================================

%% Main function
function [u,path] = f_frew(x)
% Set the airspeed
global v;

%% Initialize persistent variables 
persistent r_theta
persistent z_theta
persistent mu_x
persistent mu_y
persistent mu_z
persistent path_pts

% Assuming we have access to wind speed
global wind

% Add some Gaussian noise to measurements
wind_measurement = wind + randn*0;
x = x + [randn*0;randn*0;randn*0;randn*0];

%% Create ellipse and appropriate Fourier fit for r(theta) and z(theta)
% This section will be done offline for the project, just the Fourier
% fits will be hardcoded. 
if isempty(r_theta)
    path_pts = csvread('path_points.csv');
    %Calculate mean value of path in order to center on origin
    mu_x = mean(path_pts(:,1));     
    mu_y = mean(path_pts(:,2));
    %Leave z coordinates as are in order to avoid singularity in 
    %calculation of xi
    mu_z = 0;                       
    %Center ellipse on the origin in order to make cylindrical coordinates
    %conveniant
    path_pts(:,1) = path_pts(:,1) - mu_x;
    path_pts(:,2) = path_pts(:,2) - mu_y;
    path_pts(:,3) = path_pts(:,3) - mu_z;
    %Convert to polar coordinates
    path_pts_polar = [(path_pts(:,1).^2 + path_pts(:,2).^2).^.5,...
        atan2(path_pts(:,2),path_pts(:,1)),path_pts(:,3)];
    %Fit r and z as functions of theta with fourier series
    r_theta = fit(path_pts_polar(:,2),path_pts_polar(:,1),'fourier8');
    z_theta = fit(path_pts_polar(:,2),path_pts_polar(:,3),'fourier8');
end

% Translate aircraft coordinates into origin-centered-ellipse coordinate
% system
x_trans_1 = x(1:3) - [mu_x;mu_y;mu_z];

%% Beginning of future real time code
% Convert vehicle position to polar coordinates
r = sqrt(x_trans_1(1)^2 + x_trans_1(2)^2);
theta = atan2(x_trans_1(2),x_trans_1(1));
z = x_trans_1(3);
% Find the desired velocity command at the current position
u_vel_1 = calc_field_val(x_trans_1,r_theta,z_theta,v);
% Convert to inertial velocities
u_x_vel_1 = u_vel_1(1) * cos(theta) - u_vel_1(2) * sin(theta);
u_y_vel_1 = u_vel_1(1) * sin(theta) + u_vel_1(2) * cos(theta);
% Find the difference in course angle
delta_chi = atan2(u_y_vel_1,u_x_vel_1) - x(4);
% Account for errors near 0 and 360 degrees
delta_chi_list = [delta_chi,delta_chi + 2*pi,delta_chi - 2*pi];
[~,chi_ind] = min(abs(delta_chi_list));
delta_chi = delta_chi_list(chi_ind);
% Assign chi dot gain
k_chi = .2;
% Calculate commanded turn rate
chi_dot = k_chi*(delta_chi);
% Implement saturation point for turn limit
chi_dot_limit = 30*pi/180;
chi_dot = turn_rate_limit(chi_dot,chi_dot_limit);
% Implement saturation point for climb rate
z_dot_limit = 5;
z_dot = climb_rate_limit(u_vel_1(3),z_dot_limit);
% Output command vector
u = [norm([u_x_vel_1;u_y_vel_1;z_dot]);chi_dot;z_dot];
% Output the ellipse path for plotting
path = path_pts;
path(:,1) = path(:,1) + mu_x;
path(:,2) = path(:,2) + mu_y;
path(:,3) = path(:,3) + mu_z;
end

function u_vel = calc_field_val(x_trans,r_theta,z_theta,v)
k_r = 100;
k_z = 500;

% Calculate aircraft coordinates in cylindrical coordinates
r = sqrt(x_trans(1)^2 + x_trans(2)^2);
theta = atan2(x_trans(2),x_trans(1));
z = x_trans(3);

% Find r and z on path at theta of aircraft
r_d = r_theta(theta);
z_d = z_theta(theta);
% Calculate r' and z' at theta of aircraft
r_d_prime = differentiate(r_theta,theta);
z_d_prime = differentiate(z_theta,theta);
% Calculate the coordinates in the new coordinate system
rho = r/r_d;
phi = theta;
xi = z/z_d;
% Set up appropriate matrices
M_r = diag([1,1/r,1]);
M_q = diag([1,1/rho,1]);
% Define the Jacobian 
J = [r_d, r/r_d * r_d_prime,0;0,1,0;0 z/z_d*z_d_prime z_d];

h_q = [-k_r * (rho - 1); 2*rho;-k_z*(xi - 1)];

dx_dt = 0; % This term is zero since the curve isn't time varying

v1 = inv(M_r) * J * M_q * h_q;
v2 = [0;0;0]; % dx/dt is 0 since curve isn't time varying

alpha =(-v1'*v2 + sqrt((v1'*v2)^2 - (v1'*v1)*(v2'*v2 - v^2)))/(v1'*v1);
%Calculate the the commanded velocity
u_vel = alpha * v1 + v2;
end

function chi_dot = turn_rate_limit(chi_dot_commanded,limit)
if abs(chi_dot_commanded) > limit
    chi_dot = chi_dot_commanded/abs(chi_dot_commanded) * limit;
else
    chi_dot = chi_dot_commanded;
end
end

function z_dot = climb_rate_limit(z_dot_commanded,limit)
if abs(z_dot_commanded) > limit
    z_dot = z_dot_commanded/abs(z_dot_commanded) * limit;
else
    z_dot = z_dot_commanded;
end
end