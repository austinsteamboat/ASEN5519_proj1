clear all
clear f_frew
clear f_park
clear simulate_flight
clear curve_gen
clc

% Define ellipse parameters
p = [500,200,150]'; %Ellipse center
sa = 400;           %Ellipse semi-major axis
si = 250;           %Ellipse semi-minor axis
phi = 0;            %Ellipse roll
theta = 40*pi/180;  %Ellipse pitch
psi = 30*pi/180;    %Ellipse yaw
% Ellipse Rotation Matrix
R = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)] * ...
    [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]*...
    [cos(psi) -sin(psi) 0; sin(psi) cos(phi) 0; 0 0 1];

% Vehicle Airspeed
v = 20;

% Parameter list for part a
na = {p,sa,si,eye(3),v};
% Parameter list for part b
nb = {p,sa,si,R,v};

%Initialize Vehicle State
x_0 = [450;200;140;0];

% Set parameters
tmax = 200;
function_choose = 2; % 1:Park, 2:Frew
delay = 0;
pos_list = [85, 210,100,0;525,525,200,0;925,225,175,-pi/2;510,-25,200,pi];
delay_list = [.5,1,1.5,2];
%Run simulation
for i = 1:size(pos_list,1)
    simulate_flight(pos_list(1,:)',nb,tmax,delay_list(i),function_choose,i);
end
