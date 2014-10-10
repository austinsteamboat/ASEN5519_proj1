function [x_list,error_list] = simulate_flight(x,n,tmax,delay,function_choose,plot_ind)
v = n{5};
x0 = x;
% Initialize time variables
dt = 0.1;
t = 0:dt:tmax;

% Set up control delay logic
delay_ind = round(delay/dt);
u_list = zeros(delay_ind,3);
u_list(:,1) = ones(delay_ind,1) * v;

% Initialize plotting lists
x_list = [];
error_list = [];

for i = 1:size(t,2)
    % Choose which function to use
    if function_choose == 1
        [u,path] = f_park(x,n);
    else
        [u,path] = f_frew(x,n);
    end
    
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
    vxy = sqrt(v^2); %- u_command(3)^2);
    xdot(1) = vxy * cos(x(4)); %+ vxy * sin(x(4));
    xdot(2) = vxy * sin(x(4)); %+ vxy * cos(x(4));
    xdot(3) = u_command(3);
    xdot(4) = u_command(2);
    
    % Keep heading bounded
    while x(4) > 2*pi
        x(4) = x(4) - 2*pi;
    end
    
    x = x + xdot' * dt;
    % Append to list for plotting
    x_list = [x_list;x'];
    
    % Calculate error from path
    dist = inf;
    for j = 1: size(path,1)
        dist_temp = sum((path(j,:)'-x(1:3)).^2);
        if dist_temp < dist
            dist = dist_temp;
        end
    end
    
    error_list = [error_list;dist];
        
end
% Plots
figure(plot_ind)
clf
subplot(1,2,1)
title(strcat('Time Delay = ',num2str(delay)))
scatter3(x0(1),x0(2),x0(3),10,'filled')
plot3(x_list(:,1),x_list(:,2),x_list(:,3))
hold on
plot3(path(:,1),path(:,2),path(:,3),'Color','g','LineWidth',2)
grid on
axis([-200 1000 -200 600 -100 500])
subplot(1,2,2);
title(strcat('Time Delay = ',num2str(delay)))
semilogy(t,error_list');
xlabel('Time')
ylabel('Error^2')
axis([0 200 0 5e4])
end