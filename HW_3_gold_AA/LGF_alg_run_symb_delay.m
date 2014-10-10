% LGVF Article Alg Implementation Delay
% Required Helper Files:
% elip_gen.m - generates ellipse trajectory for error calcs
% boldify.m - makes the graphs look nice
% rot_mat321.m - yaw-pitch-roll rotation matrix
% rot_mat123.m - inverse yaw-pitch-roll rotation matrix
% v1_run.m - generates the v1 command for the vector field
% alpha_run.m - generates the alpha scaling term 
%
% Run with F5 or ctrl+enter
%
clear,clc
% syms rD thetaD zD t u_des r z theta
Ts = 1/250; % 1 kHz Simulation Rate
% Simulation Duration
T_sim = 80; % in seconds
% Duratio of Sim in Cycles
Sim_dur = T_sim/Ts;
% Trajectory Inputs
n_points = 10000;
p = [500,200,150]; % ellipse center
% p = [0,0,150];
sa = 400; % Semi Major
si = 250; % Semi Minor
traj_speed = 20; % Desired Trajectory Airspeed
yaw_rot = 0*pi/180; % yaw angle in rads
pitch_rot = 0*pi/180; % pitch angle in rads
roll_rot = 0*pi/180; % roll angle in rads
yaw = yaw_rot+eps; % add eps in case of divides by zero
pitch = pitch_rot+eps;
roll = roll_rot+eps;
% Generate Ellipse Vector
[t_out,x_out,y_out,z_out] = elip_gen(p,sa,si,n_points,traj_speed,yaw_rot,pitch_rot,roll_rot);
[theta_traj,r_traj,z_traj] = cart2pol(x_out-p(1),y_out-p(2),z_out);
% 
% Build Trajector Matrix for indexing
X_traj = [theta_traj', r_traj', z_traj'];
% Set desired speed
u_des = traj_speed;
% Set Initial States
time_vec = 0;
X_dot_not = [0, 0, 0, 0]; % Initial Aircraft Rates
angle_err = 0;
% X_not = [[x_out(1), y_out(1), z_out(1)]+[x_out(1)+20, 10, 0]), 90*pi/180];% Initial Aircraft States
% X_not = [[x_out(1), y_out(1), z_out(1)] + [10,10,10], 0*pi/180];
X_not = [0,0,0,0*pi/180]; % initial position and pointing
% Initialize States
X_pos = X_not;
X_dot = X_dot_not;
X_dot_pol = X_dot_not;
interp_val_d = X_traj(1,:);
% Loop Parameters
traj_t_step = mean(diff(t_out));
m_cyc = 10; % Counter value for autopilot delay
            % Ie autopilot loop updates every m_cyc simulation cycles
loop_counter = m_cyc; % Initialize loop counter for that
% Control Parameters
k1 = 10; % attraction for r/rd
k2 = 10; % attraction for z/zd
zd = p(3);
udes = u_des;
kp = 16; % angle proportional control gains
ki = 8; % angle forward integral control gain
% Start The Loop
%
for i = 1:Sim_dur
   % find the nearest index for trajectory error calculation  
   delta_dist2 = sqrt((x_out-X_pos(1)).^2+(y_out-X_pos(2)).^2+(z_out-X_pos(3)).^2);
   [val_dum1,ind_min2] = min(abs(delta_dist2));
   X_pos(1:3) = X_pos(1:3)-p(1:3);% translate to origin
   if (loop_counter==m_cyc) % If it's the m'th cycle, update control
       [X_pos1(1),X_pos1(2),X_pos1(3)] = rot_mat123(X_pos(1),X_pos(2),X_pos(3),roll_rot,pitch_rot,yaw_rot); % Rotate the aircrafts position to the ellipse's plane
       [X_theta,X_r,X_z] = cart2pol(X_pos1(1),X_pos1(2),X_pos1(3)); % go to cylindrical
       X_pos_pol = [X_theta,X_r,X_z];
       % Update time step
       time_vec = time_vec+Ts;
       r = X_r+eps; % assign r,theta,z, again add eps incase of divide by zero
       theta = X_theta+eps;
       z = X_z+eps;
       % Calculate Vector Field
       alpha1 = alpha_run(k1,k2,r,sa,si,theta,udes,z,eps);
       v1 = v1_run(k1,k2,r,sa,si,theta,z,eps);
       u_not = alpha1*v1;
       rdot = u_not(1);
       thetadot = u_not(2)/r;
       zdot = u_not(3);
       % Check if vector field in NaN
       if isnan(rdot)
           rdot = 0;
       end
       if isnan(zdot)
           zdot = 0;
       end
       if isnan(thetadot)
           thetadot = 0;
       end
       % Translate to Cartesian
       x_dot_calc = real(rdot*cos(theta)-r*thetadot*sin(theta));
       y_dot_calc = real(rdot*sin(theta)+r*thetadot*cos(theta));
       % Rotate back to aircraft body frame
       [x_dot_calc,y_dot_calc,zdot] = rot_mat321(x_dot_calc,y_dot_calc,zdot,roll_rot,pitch_rot,yaw_rot);
       u1 = u_des; % airspeed -> just set it to our desired speed
       % The alpha normalization takes care of forcing the commands
       % to the desired airspeed
       u2 = zdot; % climb rate, zdot = u2 in the unicycle model
       % This will give us a velocity vector we need to align with
       % We'll need to find the angle of that vector and
       % With some control logic, point ourselves to it
       %
       % Calculate Commanded Velocity Angle
       vcom_angle = atan2(y_dot_calc,x_dot_calc);
       % Calculate Current Veloctiy Angle, assuming no wind this should be
       % course angle
       vel_angle = X_pos(4);
       % Update Old Error
       angle_err_old = angle_err;
       % Calculate New Error
       angle_err = (vcom_angle-vel_angle);
       % This logic is a check for the pi->-pi jump
       % It checks if there's a big jump and doesn't update if there is
       if abs(angle_err)>(2*pi*.9)
           angle_err = angle_err_old;
       end
       % Calculate Error Slope for Forward Projection
       angle_err_slope = (angle_err-angle_err_old)/Ts;
       u3 = kp*angle_err+ki*((angle_err_slope*1*Ts)+angle_err);
       % Saturation Logic
       if u3>pi
           u3 = pi*.99; % Added the .99 to keep from wrapping at pi=-pi
       elseif u3<-pi
           u3 = -pi*.99;
       end
       loop_counter = 0;
   end
   % Start Kinematic Model
   X_pos_old = X_pos;
   % Assign Current Rates
   X_dot(1) = u1*cos(X_pos(4));
   X_dot(2) = u1*sin(X_pos(4));
%    X_dot(1) = x_dot_calc;
%    X_dot(2) = y_dot_calc;
   X_dot(3) = u2;
   X_dot(4) = u3;
   % Update New State by integrating new rates
   X_pos(1) = X_pos_old(1)+ (X_dot(1))*Ts+p(1);
   X_pos(2) = X_pos_old(2)+ (X_dot(2))*Ts+p(2);
   X_pos(3) = X_pos_old(3)+ (X_dot(3))*Ts+p(3);
   X_pos(4) = X_pos_old(4)+ (X_dot(4))*Ts;
   % Update Loop Counter
   loop_counter = loop_counter+1; 
   % Set Output Vectors
   % Mostly Debug Signals
   t_vec(i) = time_vec;
   X_vec(i,:) = X_pos;
   X_dot_vec(i,:) = X_dot;
   u1_vec(i) = u1;
   u2_vec(i) = u2;
   u3_vec(i) = u3;
   xdot_vec(i) = x_dot_calc;
   ydot_vec(i) = y_dot_calc;
   rdot_vec(i) = rdot;
   zdot_vec(i) = zdot;
   thetadot_vec(i) = thetadot;
   v_com_vec(i) = vcom_angle;
   err_vec(i) = angle_err;
   theta_vec(i) = theta;
   alpha1_vec(i) = alpha1;
   ind_min2_vec(i) = ind_min2;
end    
%
% Visualize Results
figure(1)
clf(1)
subplot(2,2,1)
hold all
plot(X_not(1),X_not(2),'O')
plot(x_out,y_out) 
plot(X_vec(:,1),X_vec(:,2))
plot(X_pos(1),X_pos(2),'O')
legend('Start','Desired','Actual','Stop')
xlim([-1e3,1e3])
ylim([-1e3,1e3])
ylabel('East')
xlabel('North')
title('Birds Eye View')
grid('on')
hold off
subplot(2,2,2)
hold all
plot3(x_out,y_out,z_out)
plot3(X_vec(:,1),X_vec(:,2),X_vec(:,3))
legend('Desired','Actual')
hold off
xlim([-1e3,1e3])
ylim([-1e3,1e3])
zlim([-1e3,1e3])
xlabel('North')
ylabel('East')
zlabel('Alt')
title('3D Plot')
subplot(2,2,3:4)
hold all
plot(t_vec,abs(x_out(ind_min2_vec).'-X_vec(:,1)))
plot(t_vec,abs(y_out(ind_min2_vec).'-X_vec(:,2)))
plot(t_vec,abs(z_out(ind_min2_vec).'-X_vec(:,3)))
xlabel('Time in Seconds')
ylabel('Absolute Value of Error')
title('Error Plot')
legend('North Error','East Error','Alt Error')
grid('on')
% ylim([-1,10])
hold off
boldify
hold off
xlabel('time in simulation steps')
ylabel('Error in m')
legend('North Error','East Error','Alt Error')
boldify