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
%
% This version will be converted into C code.
%=========================================================================
function u = f_frew_C_template(x,y,z,chi)

% Read in Fourier coefficients
coeffs = ones(18,4);

a_r_0 = coeffs(1,1);
a_r_1 = coeffs(2,1);
a_r_2 = coeffs(4,1);
a_r_3 = coeffs(6,1);
a_r_4 = coeffs(8,1);
a_r_5 = coeffs(10,1);
a_r_6 = coeffs(12,1);
a_r_7 = coeffs(14,1);
a_r_8 = coeffs(16,1);
b_r_1 = coeffs(3,1);
b_r_2 = coeffs(5,1);
b_r_3 = coeffs(7,1);
b_r_4 = coeffs(9,1);
b_r_5 = coeffs(11,1);
b_r_6 = coeffs(13,1);
b_r_7 = coeffs(15,1);
b_r_8 = coeffs(17,1);
w_r = coeffs(18,1);

a_z_0 = coeffs(1,2);
a_z_1 = coeffs(2,2);
a_z_2 = coeffs(4,2);
a_z_3 = coeffs(6,2);
a_z_4 = coeffs(8,2);
a_z_5 = coeffs(10,2);
a_z_6 = coeffs(12,2);
a_z_7 = coeffs(14,2);
a_z_8 = coeffs(16,2);
b_z_1 = coeffs(3,2);
b_z_2 = coeffs(5,2);
b_z_3 = coeffs(7,2);
b_z_4 = coeffs(9,2);
b_z_5 = coeffs(11,2);
b_z_6 = coeffs(13,2);
b_z_7 = coeffs(15,2);
b_z_8 = coeffs(17,2);
w_z = coeffs(18,2);

a_r_prime_0 = coeffs(1,3);
a_r_prime_1 = coeffs(2,3);
a_r_prime_2 = coeffs(4,3);
a_r_prime_3 = coeffs(6,3);
a_r_prime_4 = coeffs(8,3);
a_r_prime_5 = coeffs(10,3);
a_r_prime_6 = coeffs(12,3);
a_r_prime_7 = coeffs(14,3);
a_r_prime_8 = coeffs(16,3);
b_r_prime_1 = coeffs(3,3);
b_r_prime_2 = coeffs(5,3);
b_r_prime_3 = coeffs(7,3);
b_r_prime_4 = coeffs(9,3);
b_r_prime_5 = coeffs(11,3);
b_r_prime_6 = coeffs(13,3);
b_r_prime_7 = coeffs(15,3);
b_r_prime_8 = coeffs(17,3);
w_r_prime = coeffs(18,3);

a_z_prime_0 = coeffs(1,4);
a_z_prime_1 = coeffs(2,4);
a_z_prime_2 = coeffs(4,4);
a_z_prime_3 = coeffs(6,4);
a_z_prime_4 = coeffs(8,4);
a_z_prime_5 = coeffs(10,4);
a_z_prime_6 = coeffs(12,4);
a_z_prime_7 = coeffs(14,4);
a_z_prime_8 = coeffs(16,4);
b_z_prime_1 = coeffs(3,4);
b_z_prime_2 = coeffs(5,4);
b_z_prime_3 = coeffs(7,4);
b_z_prime_4 = coeffs(9,4);
b_z_prime_5 = coeffs(11,4);
b_z_prime_6 = coeffs(13,4);
b_z_prime_7 = coeffs(15,4);
b_z_prime_8 = coeffs(17,4);
w_z_prime = coeffs(18,4);

% Transform vehicle position to polar coordinates
r = (x.^2 + y.^2).^.5;
theta = atan2(y,x);
z = z;

% Set vector field gains
k_r = 100;
k_z = 500;

r_d = a_r_0 + a_r_1*cos(theta*w_r) + b_r_1*sin(theta*w_r) + ...
               a_r_2*cos(2*theta*w_r) + b_r_2*sin(2*theta*w_r) + a_r_3*cos(3*theta*w_r) + b_r_3*sin(3*theta*w_r) + ...
               a_r_4*cos(4*theta*w_r) + b_r_4*sin(4*theta*w_r) + a_r_5*cos(5*theta*w_r) + b_r_5*sin(5*theta*w_r) + ...
               a_r_6*cos(6*theta*w_r) + b_r_6*sin(6*theta*w_r) + a_r_7*cos(7*theta*w_r) + b_r_7*sin(7*theta*w_r) + ...
               a_r_8*cos(8*theta*w_r) + b_r_8*sin(8*theta*w_r);
z_d = a_z_0 + a_z_1*cos(theta*w_z) + b_z_1*sin(theta*w_z) + ...
               a_z_2*cos(2*theta*w_z) + b_z_2*sin(2*theta*w_z) + a_z_3*cos(3*theta*w_z) + b_z_3*sin(3*theta*w_z) + ...
               a_z_4*cos(4*theta*w_z) + b_z_4*sin(4*theta*w_z) + a_z_5*cos(5*theta*w_z) + b_z_5*sin(5*theta*w_z) + ...
               a_z_6*cos(6*theta*w_z) + b_z_6*sin(6*theta*w_z) + a_z_7*cos(7*theta*w_z) + b_z_7*sin(7*theta*w_z) + ...
               a_z_8*cos(8*theta*w_z) + b_z_8*sin(8*theta*w_z);
r_d_prime = a_r_prime_0 + a_r_prime_1*cos(theta*w_r_prime) + b_r_prime_1*sin(theta*w_r_prime) + ...
               a_r_prime_2*cos(2*theta*w_r_prime) + b_r_prime_2*sin(2*theta*w_r_prime) + a_r_prime_3*cos(3*theta*w_r_prime) + b_r_prime_3*sin(3*theta*w_r_prime) + ...
               a_r_prime_4*cos(4*theta*w_r_prime) + b_r_prime_4*sin(4*theta*w_r_prime) + a_r_prime_5*cos(5*theta*w_r_prime) + b_r_prime_5*sin(5*theta*w_r_prime) + ...
               a_r_prime_6*cos(6*theta*w_r_prime) + b_r_prime_6*sin(6*theta*w_r_prime) + a_r_prime_7*cos(7*theta*w_r_prime) + b_r_prime_7*sin(7*theta*w_r_prime) + ...
               a_r_prime_8*cos(8*theta*w_r_prime) + b_r_prime_8*sin(8*theta*w_r_prime);
z_d_prime = a_z_prime_0 + a_z_prime_1*cos(theta*w_z_prime) + b_z_prime_1*sin(theta*w_z_prime) + ...
               a_z_prime_2*cos(2*theta*w_z_prime) + b_z_prime_2*sin(2*theta*w_z_prime) + a_z_prime_3*cos(3*theta*w_z_prime) + b_z_prime_3*sin(3*theta*w_z_prime) + ...
               a_z_prime_4*cos(4*theta*w_z_prime) + b_z_prime_4*sin(4*theta*w_z_prime) + a_z_prime_5*cos(5*theta*w_z_prime) + b_z_prime_5*sin(5*theta*w_z_prime) + ...
               a_z_prime_6*cos(6*theta*w_z_prime) + b_z_prime_6*sin(6*theta*w_z_prime) + a_z_prime_7*cos(7*theta*w_z_prime) + b_z_prime_7*sin(7*theta*w_z_prime) + ...
               a_z_prime_8*cos(8*theta*w_z_prime) + b_z_prime_8*sin(8*theta*w_z_prime);

rho = r/r_d;
phi = theta;
zeta = z/z_d;

v1_1 = -k_r*(rho - 1)*r_d + 2*r/r_d *r_d_prime;
v1_2 = 2*r;
v1_3 = 2*z/z_d * z_d_prime - k_z*(zeta - 1)*z_d;

v_desired = 20;
alpha = v_desired/(v1_1^2 + v1_2^2 + v1_3^2)^.5;

v_r = alpha*v1_1;
v_theta = alpha*v1_2;
v_z = alpha*v1_3;

vx = v_r * cos(theta) - v_theta * sin(theta);
vy = v_r * sin(theta) + v_theta * cos(theta);

delta_chi = atan2(vy,vx) - chi;

delta_chi_list = [delta_chi,delta_chi + 2*pi,delta_chi - 2*pi];
[~,chi_ind] = min(abs(delta_chi_list));
delta_chi = delta_chi_list(chi_ind);
% Assign chi dot gain
k_chi = .2;
% Calculate commanded turn rate
chi_dot = k_chi*(delta_chi);

u = [v_desired;chi_dot;v_z];



