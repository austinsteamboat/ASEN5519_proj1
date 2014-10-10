%=========================================================================
% Author: Drew Ellison
% Date  : 10/2/2014
% Description: 
% This function generates a list of points on the ellipse described by the
% input parameters.
%=========================================================================

function path_pts = curve_gen(p,sa,si,R)
x0 = p(1);                  %Extract center location
y0 = p(2);
z0 = p(3);

ellipse_pts = [];           %Initialize ellipse points list
xmin = -sa;                 %Set minimum to negative major axis
%Choose the number of points to use in the ellipse
n=5000;                     
dx = sa/(n/2);              %Calculate the distance between x values
x = xmin;                   %Initialize x

for i = 1:n
    %Append new ellipse points in the x-y plane to begin (All +y)
    ellipse_pts = [ellipse_pts;x,y(sa,si,x),0]; 
    %Update x
    x = x + dx;                                 
end
%Extract different components of the ellipse
ellipse_x = ellipse_pts(:,1);                   
ellipse_y = ellipse_pts(:,2);
ellipse_z = ellipse_pts(:,3);
%negate y column
ellipse_pts_reverse = [ellipse_x(end:-1:1),-ellipse_y(end:-1:1),ellipse_z]; 
%Append reveresed points in order to complete the ellipse
ellipse_pts = [ellipse_pts;ellipse_pts_reverse];

for i = 1:size(ellipse_pts,1)
    ellipse_pts(i,:) = (R*ellipse_pts(i,:)')';   %Rotate the ellipse in 3D
end
%Translate the ellipse to the appropriate center point
path_pts = [ellipse_pts(:,1) + x0, ellipse_pts(:,2) + y0,ellipse_pts(:,3) + z0]; 
end

function y_pt = y(sa,si,x)
y_pt = si*sqrt(1-(x./sa).^2); %Calculate y based on ellipse parameters
end