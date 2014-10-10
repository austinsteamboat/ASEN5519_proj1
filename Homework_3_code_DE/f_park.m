%=========================================================================
% Author: Drew Ellison
% Date  : 10/2/2014
% Description: Generates command vector [airspeed;turn rate;climb rate]
% Turn rates (chi_dots) are based on the 2004 paper "A
% New Nonlinear Guidance Logic for Trajectory Tracking" by Park, Deyst, and
% How. This algorithm has been adapted for 3D situations by using a
% proportional controller to independently control climb rate. The turn
% rate commands are generated assuming that the target and the aircraft are
% at the same altitude.
%=========================================================================

function [u,path] = f_park(x,n)
%% Extract different paramters from n
p   = n{1};
sa  = n{2};
si  = n{3};
R   = n{4};
v   = n{5};
l1  = 25;
%% Initialize parameters
% Initialize the ellipse curve 
persistent path_pts
if isempty(path_pts)
    path_pts = curve_gen(p,sa,si,R);
end
% Initialize/Update the current target waypoint
% Remember last waypoint
persistent current_target_ind
% If not initialized, find nearest point on ellipse and set as waypoint
if isempty(current_target_ind)
    dist = inf;
    for i = 1:size(path_pts,1)
        dist_temp = norm(x(1:2) - path_pts(i,1:2)');
        if dist_temp < dist
            dist = dist_temp;
            current_target_ind = i;
        end
    end
% Otherwise use function to find target index on path
else
    current_target_ind = find_target(path_pts,x,current_target_ind,l1);
end

%% Calculate the commanded acceleration
% Find target coordinates in inertial frame
target = path_pts(current_target_ind,:)';
% Convert target coordinates to body frame to ease calculation of eta
l1 = l1_body(x,target);
% Calculate eta
eta = eta_calc(l1);
% Find appropraite acceleration
as = -2*v^2/sqrt(l1(1)^2 + l1(2)^2) * sin(eta);
% Find resulting command for theta_dot
theta_dot = as/v;
% Use proportional controller to find climb rate
z_dot = z_prop_controller(x,path_pts);
% Assign command vector
u = [v,theta_dot,z_dot]';
% Output path for plotting
path = path_pts;
end

%% Climb rate calculation
function z_dot = z_prop_controller(x,path)
% This function generates climb rate commands based on the error in
% altitude between the current position and the altitude of the nearest
% point on the trajectory. It could definitely be optimized for performance
% in real time, but for current simulation purposes, it suffices. 

% Find the distance to the nearest point
dist = inf;
for i = 1:size(path,1)
    dist_temp = norm(x(1:3) - path(i,:)');
    if dist_temp < dist
        dist = dist_temp;
        closest_pt_ind = i;
    end
end

% Extract the altitude of the nearest point
closest_pt_z = path(closest_pt_ind,3);
% Assign to target z variable
target_z = closest_pt_z;

% Assign proportional gain, this parameter can be tuned for performance
prop_gain = .5;
% Calculate the climb rate
z_dot = prop_gain*(target_z - x(3));
end

%% Target ID calculation
function target_ind = find_target(path_pts,x,current_target_ind,l1)
% This function returns the index on the waypoint list of the new current
% target

% Find the current target
target_ind = current_target_ind;
while norm(path_pts(target_ind,1:2)' - x(1:2)) < l1
    %While the distance of the current target point is less than L1,
    %continue looping through points
    %Loop through ellipse points appropriately
    if target_ind ~= size(path_pts,1)
        target_ind = target_ind + 1;
    else
        target_ind = 1;
    end
end
end

function l1b_vec = l1_body(x,target)
%Find L1 in inertial coordinates
l1_inertial = target - x(1:3);
% Negate y and z and then rotate to get in body coordinates
l1_2 = [l1_inertial(1);-l1_inertial(2);-l1_inertial(3)];
% Rotate L1 appropriately
l1b_vec = [cos(x(4)) sin(x(4)) 0;-sin(x(4)) cos(x(4)) 0;0 0 1]'*l1_2;
end

function n = eta_calc(l1b)
%Calculate eta based on L1 in body coordinates, where eta is the angle
%between the current velocity vector and the vector from the aircraft to
%the current target position
n = atan2(l1b(2),l1b(1));
end
