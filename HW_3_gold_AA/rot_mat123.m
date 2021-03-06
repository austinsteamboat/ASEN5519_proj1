function [x_out,y_out,z_out] = rot_mat123(x_in,y_in,z_in,roll,pitch,yaw)
R_yaw = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
R_pitch = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)]; 
R_roll = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
R_321 = R_roll*R_pitch*R_yaw;% roll pitch yaw
dum_val = R_321^-1*[x_in;y_in;z_in];
x_out = dum_val(1);
y_out = dum_val(2);
z_out = dum_val(3);
