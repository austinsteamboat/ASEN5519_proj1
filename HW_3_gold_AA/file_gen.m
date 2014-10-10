% file_gen.m
% this is the file that generates the function structures in matlab
% symbolic toolbox, then generates the associated m-files.
% I wouldn't recommend re-running this, but you can see how the
% v1_run.m and alpha_run.m were made
%
% Run with F5 or ctrl+enter
clear,clc
syms zd rd r z sa si theta theta k1 k2 udes
% Calculate the desired radius based on the function for an ellipse
% in polar coordinates
rd = (sa*si)/(sqrt(((si*cos(theta)).^2)+((sa*sin(theta)).^2)));
% Calculate the rho and zeta values:
rho = r/rd;
zeta = z/zd;
% Calculate Mr_inv matrix:
Mr_inv = [1 0 0;0 r 0;0 0 1];
% Calculate Mq Matrix:
Mq = [1 0 0; 0 1/rho 0; 0 0 1];
% Update Control Law:
hq = [-k1*(rho-1); 2*rho; -k2*(zeta-1)];
% Calculate partial derivatives for the Jacobian
rd_prime = diff(rd,theta);
zd_prime = diff(zd,theta);
J = [rd rd_prime*r/rd 0; 0 1 0; 0 zd_prime*z/zd zd];
% Put all the pieces together to get v1:
v1 = Mr_inv*J*Mq*hq;
% Calculate the Normalization term:
alpha = sqrt(v1.'*v1*udes^2)/(v1.'*v1);
% Calculate the polar vector field:
% I didn't actually use these, but you could build
% functions directly out of these.
u = alpha*v1;
u = [1 0 0; 0 1/r 0; 0 0 1]*u;
rdot = u(1);
thetadot = u(2);
zdot = u(3);
%% Run this cell to actually re-generate the functions
% Probably backup the existing .m files before running this
alpha_run = matlabFunction(alpha,'file','alpha_run.m');
v1_run = matlabFunction(v1,'file','v1_run.m');