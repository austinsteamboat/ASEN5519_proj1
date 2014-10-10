function [x,y,z] = gps_to_local_xyz(lat_in,lon_in,alt_in,lat_orig,lon_orig,alt_orig)
% Inputs
% lat_in - current GPS lat in decimal degrees
% lon_in - current GPS lon in decimal degrees
% alt_in - current GPS alt in m
% lat_orig - global GPS origin lat in decimal degrees
% lon_orig - global GPS origin lon in decimal degrees
% alt_orig - global GPS origin alt in m
% Outputs
% x - north displacement in m relative to the GPS origin in m
% y - East displacement in m relative to the GPS origin in m
% z - Altitude displacement above the GPS origin in m
% Algorith is based on great circle distances
%
% earth radius in m
r = 6378100;
% degrees to rad
lat_orig = lat_orig*pi/180;
lat_in = lat_in*pi/180;
lon_orig = lon_orig*pi/180;
lon_in = lon_in*pi/180;
d_lon = lon_in-lon_orig;
d_lat = lat_in-lat_orig;
% GC X,Y
y = r*sin(d_lon)*cos(lat_orig);
x = r*(cos(lat_orig)*sin(lat_in)-sin(lat_orig)*cos(lat_in)*cos(d_lon));
% Z's just subtract
z = alt_in-alt_orig;
