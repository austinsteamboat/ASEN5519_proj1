function generate_coefficients

format long g
path_pts = csvread('path_to_follow_final.csv');
%Calculate mean value of path in order to center on origin
mu_x = mean(path_pts(:,1));
mu_y = mean(path_pts(:,2));
%Leave z coordinates as are in order to avoid singularity in
%calculation of xi
mu_z = 0;
%Center ellipse on the origin in order to make cylindrical coordinates
%conveniant
path_pts(:,1) = path_pts(:,1) - mu_x;
path_pts(:,2) = path_pts(:,2) - mu_y;
path_pts(:,3) = path_pts(:,3) - mu_z;
%Convert to polar coordinates
path_pts_polar = [(path_pts(:,1).^2 + path_pts(:,2).^2).^.5,...
    atan2(path_pts(:,2),path_pts(:,1)),path_pts(:,3)];
%Fit r and z as functions of theta with fourier series
r_theta = fit(path_pts_polar(:,2),path_pts_polar(:,1),'fourier8');
z_theta = fit(path_pts_polar(:,2),path_pts_polar(:,3),'fourier8');

r_theta_coeffs = coeffvalues(r_theta)'
z_theta_coeffs = coeffvalues(z_theta)'

r_d_prime_pts = differentiate(r_theta,path_pts(:,2));
z_d_prime_pts = differentiate(z_theta,path_pts(:,2));

r_d_prime = fit(path_pts_polar(:,2),r_d_prime_pts,'fourier8');
z_d_prime = fit(path_pts_polar(:,2),z_d_prime_pts,'fourier8');

r_d_prime_coeffs = coeffvalues(r_d_prime)';
z_d_prime_coeffs = coeffvalues(r_d_prime)';

coeff_matrix = [r_theta_coeffs,z_theta_coeffs,r_d_prime_coeffs,z_d_prime_coeffs];

csvwrite('fourier_coefficients.csv',coeff_matrix);

string = '';
for i = 1:size(coeff_matrix,2)
    if i == 1
        id_str = '_r';
    elseif i ==2 
        id_str = '_z';
    elseif i ==3
        id_str = '_r_prime';
    else
        id_str = '_z_prime';
    end
    for j = 1:size(coeff_matrix,1)
        if j == 1
            %string = strcat(string,'float a',id_str,'_0;\r\n');
            string = strcat(string,'float a',id_str,'_0 = ',num2str(coeff_matrix(j,i) ,'%5.10f'),';\r\n');
        elseif j == 18
           % string = strcat(string,'float w',id_str,';\r\n');
            string = strcat(string,'float w',id_str,' = ',num2str(coeff_matrix(j,i),'%5.10f'),';\r\n');
        elseif mod(j,2) == 0
           % string = strcat(string,'float a',id_str,'_',num2str(j/2),';\r\n');
            string = strcat(string,'float a',id_str,'_',num2str(j/2),' = ',num2str(coeff_matrix(j,i),'%5.10f'),';\r\n');
        else
           % string = strcat(string,'float b',id_str,'_',num2str((j-1)/2),';\r\n');
            string = strcat(string,'float b',id_str,'_',num2str((j-1)/2),' = ',num2str(coeff_matrix(j,i),'%5.10f'),';\r\n');
        end
    end
end

% string
fid = fopen('coefficient_header.txt','w');
fprintf(fid,string);
fclose(fid);

end
