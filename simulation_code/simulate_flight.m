function [x_list,error_list] = simulate_flight(x,tmax,delay)

global v;
global wind;

x0 = x;
% Initialize time variables
dt = 0.1;
t = 0:dt:tmax;

% Set up control delay logic
delay_ind = round(delay/dt);
u_list = zeros(delay_ind,3);
u_list(:,1) = ones(delay_ind,1) .* v;

% Initialize plotting lists
x_list = [];
error_list = [];

for i = 1:size(t,2)
    % Write now, the function just takes in truth position in SI units and
    % adds some noise to it. Do we want to actually implement a simulation
    % with GPS conversions in it? I don't think it's necessary, just make
    % sure that the path that Frew gives us and the output of the GPS code
    % are in the same coordinate system and we should be good. 
    
    % Choose which function to use
    [u,path] = f_frew(x);
    % Update command queue
    if delay_ind ~= 0
        for j = 1:delay_ind - 1
            u_list(j,:) = u_list(j+1,:);
        end
        u_list(delay_ind,:) = u';
        
        % Choose appropriate command
        u_command = u_list(1,:);
    else
        u_command = u;
    end
    
    % Update truth
    vxy = u_command(1);
    xdot(1) = vxy * cos(x(4)) + wind(1);
    xdot(2) = vxy * sin(x(4)) + wind(2);
    xdot(3) = u_command(3) + wind(3);
    xdot(4) = u_command(2);
    
    % Keep heading bounded
    while x(4) > 2*pi
        x(4) = x(4) - 2*pi;
    end
    
    x = x + xdot' * dt;
    % Append to list for plotting
    x_list = [x_list;x'];
    
%     % Calculate error from path
%     dist = inf;
%     for j = 1: size(path,1)
%         dist_temp = sum((path(j,:)'-x(1:3)).^2);
%         if dist_temp < dist
%             dist = dist_temp;
%         end
%     end
%     
%     error_list = [error_list;dist];

figure(1)
clf
scatter3(x(1),x(2),x(3),50,'filled')
hold on
plot3(x_list(:,1),x_list(:,2),x_list(:,3))
hold on
plot3(path(:,1),path(:,2),path(:,3),'Color','g','LineWidth',2)
grid on
axis([-200 1000 -200 600 -100 500])
end
% % Error Plots
% figure(2)
% subplot(1,2,2);
% title(strcat('Time Delay = ',num2str(delay)))
% semilogy(t,error_list');
% xlabel('Time')
% ylabel('Error^2')
% axis([0 200 0 5e4])
end