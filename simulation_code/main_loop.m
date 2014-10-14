clear all
clear f_frew
clc

global v
global wind
v = 20;
% Adds wind into simulation, currently not accounted for in controller at
% all. Do we have access to a wind estimate, or just the estimate of
% airspeed?
wind = [0;0;0];

%Initialize Vehicle State
x_0 = [450;200;140;0];

% Set parameters
tmax = 200;
function_choose = 2;
delay = 0;

%Run simulation
simulate_flight(x_0,tmax,0);

