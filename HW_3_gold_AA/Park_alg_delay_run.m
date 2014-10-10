%% Park Article Alg Implementation
% Required Helper Files:
% elip_gen.m - generates ellipse trajectory
% boldify.m - makes the graphs look nice
% Run with F5 or ctrl+enter
clear,clc%,close('all')
tic
% Simulation Period
Ts = 1/250; % 1 kHz Simulation Rate
% Simulation Duration
T_sim = 50; % in seconds
% Duratio of Sim in Cycles
Sim_dur = T_sim/Ts;
% Trajectory Inputs
n_points = 10000;
p = [500,200,150];
sa = 400;
si = 250;
traj_speed = 20;
yaw_rot = 0*pi/180;
pitch_rot = 0*pi/180;
roll_rot = 0*pi/180;
u1 = traj_speed; % m/s - Airspeed command
kz = 1; % proportional control for altitude commands
% Calculate the Trajectory
[t_out,x_out,y_out,z_out] = elip_gen(p,sa,si,n_points,traj_speed,yaw_rot,pitch_rot,roll_rot);
X_traj = [x_out', y_out', z_out'];
% System Parameters
L1 = 20; % Size of the forward looking line
L1_detection_threshold = 1; % how close to L1 we need to be to consider we've closed on the trajectory
index_size = 1000; % how far ahead in the index to look
% Initial Conditions
time_vec = 0;
FSM_case = 0; %FSM case, 0 is closing
X_dot_not = [0, 0, 0, 0]; % Initial Aircraft State Rates
X_not = [(X_traj(1,:)+[20, 10, 0]), 90*pi/180]; % Initial Aircraft State
% Current Aircraft Postion
X_pos = X_not; % initialize x_state
% Current Aircraft Rates
X_dot = X_dot_not; % initialzie x_dot_state
% Old Aircraft Position and Rates:
X_pos_old = [0,0,0,0];
index_val = 1;
% Loop Counter Parameters
index_L1 = 1;
m_cyc = 20; % Counter value for autopilot delay
            % Ie autopilot loop updates every m_cyc simulation cycles
loop_counter = m_cyc; % Initialize loop counter for that
for i = 1:Sim_dur
    % Start the Loop
    %
    [X_theta,X_r,X_z] = cart2pol(X_pos(2)-p(2),X_pos(1)-p(1),X_pos(3)-p(3));
    theta = X_theta;
    rd1 = (sa*si)/(sqrt(((si*cos(theta)).^2)+((sa*sin(theta)).^2)));
    zd1 = p(3);
    if(loop_counter==m_cyc) % If we've made enough cycles, update the control
        % Calculate the Distance to the
        delta_x = x_out(index_val:(index_val+index_size-1))-X_pos(1);
        delta_y = y_out(index_val:(index_val+index_size-1))-X_pos(2);
        delta_z = z_out(index_val:(index_val+index_size-1))-X_pos(3);
        delta_dist = sqrt(delta_x.^2+delta_y.^2+delta_z.^2);
        delta_dist2 = sqrt((x_out-X_pos(1)).^2+(y_out-X_pos(2)).^2+(z_out-X_pos(3)).^2);
        [val_dum,ind_min] = min(abs(delta_dist-L1));
        [val_dum1,ind_min2] = min(abs(delta_dist2));
        % Save the Deltas of x and y at the min
        delta_x_min = delta_x(ind_min);
        delta_y_min = delta_y(ind_min);
        % Check if we made the detection threshold
        if val_dum<L1_detection_threshold
            index_L1 = ind_min+index_val-1; % if yes, update our index_L1
            FSM_case = 1; % set FSM state to we found it
        else
            FSM_case = 0; % if no, don't update index_L1, set state to looking
        end
        %
        index_val = index_L1; % Either way update index_val
        L1_point = X_traj(index_val,:); % Update our L1 point as well
        % Calculate angles and commands if we have one
        if FSM_case==1
            L1_angle = atan2(delta_x_min,delta_y_min);
            V_angle = atan2(X_dot(1),X_dot(2));
            %     L1_angle*180/pi
            %     V_angle*180/pi
            n_angle = V_angle-L1_angle;
            %         n_angle = L1_angle-V_angle;
            %     n_angle*180/pi
            ascmd = 2*u1^2/L1*sin(n_angle);
            u3 = 1*ascmd/u1;
            u2 = kz*(L1_point(3)-X_pos(3));
        end
        % for i = 1:Sim_dur
        loop_counter = 0;
    end
    % Update Aircraft States
    % Update Previous Step Values
    time_old = time_vec;
    X_pos_old = X_pos;
    % Assign Current Rates
    X_dot(1) = u1*cos(X_pos(4));
    X_dot(2) = u1*sin(X_pos(4));
    X_dot(3) = u2;
    X_dot(4) = u3;
    % Update New State by integrating new rates
    X_pos(1) = X_pos_old(1)+ (X_dot(1))*Ts;
    X_pos(2) = X_pos_old(2)+ (X_dot(2))*Ts;
    X_pos(3) = X_pos_old(3)+ (X_dot(3))*Ts;
    X_pos(4) = X_pos_old(4)+ (X_dot(4))*Ts;
    time_vec = time_old+Ts;
    loop_counter = loop_counter+1;
    % Set Outpu Vectors
    t_vec(i) = time_vec;
    X_vec(i,:) = X_pos;
    X_dot_vec(i,:) = X_dot;
    u3_vec(i) = u3;
    ind_min2_vec(i) = ind_min2;
end
toc
%
% Visualization
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