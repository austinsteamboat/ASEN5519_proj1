% GPS to NED Script
% We'll want to do this between two GPS locations as raw conversion of GPS
% data will give us coordinates relative to 
clear,clc,close('all')
lat_orig = 40.138036; % lat in degrees
lon_orig = -105.245588; % long in degrees
alt_orig = 5551*unitsratio('m','ft');
%
% lat_in = 40.147060;
% lon_in = -105.245548; % 1 km north of origin
lat_in = 40.138014;
lon_in = -105.233842; % 1 km east of origin
alt_in = 5780*unitsratio('m','ft');
[x,y,z] = gps_to_local_xyz(lat_in,lon_in,alt_in,lat_orig,lon_orig,alt_orig);
% bear_1 = atan2(y,x)*180/pi
% d_2_t = sqrt(x.^2+y.^2)