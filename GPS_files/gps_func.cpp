// function example
#include <stdio.h>
#include <math.h>

#define PI 3.141592653589793 // pi
#define r_earth 6371000 // earth radius
#define deg_val 0.005555555556 // 1/180

void gps_xy (double lat_in, double lon_in, double lat_orig, double lon_orig, double NED_pos[2])
{
  double lat_orig1 = lat_orig*PI*deg_val;// precompute 1/180 so no divide
  double lon_orig1 = lon_orig*PI*deg_val;
  double lat_in1 = lat_in*PI*deg_val;
  double lon_in1 = lon_in*PI*deg_val;
  double d_lon = lon_in1-lon_orig1;
  double d_lat = lat_in1-lat_orig1;
  double lat_m = (lat_in1+lat_orig1)/2;
  //float x = r_earth*(cos(lat_orig1)*sin(lat_in1)-sin(lat_orig1)*cos(lat_in1)*cos(d_lon));  
  double x = r_earth*d_lat;
  double y = r_earth*(d_lon*cos(lat_m));
  NED_pos[0] = x;
  NED_pos[1] = y;
}

int main ()
{
// Initialize array
  double arr[2];
// Current Value
//  double lat_in = 40.147060;
//  double lon_in = -105.245548; // 1km north of origin
//
  double lat_in = 40.138014;
  double lon_in = -105.233842; // 1km east of origin

// Origin
  double lat_orig = 40.138036;
  double lon_orig = -105.245588;
  int arr2[] = {10,11};
  gps_xy(lat_in,lon_in,lat_orig,lon_orig,arr);
  printf("The North result is : %f \n",arr[0]);
  printf("The East result is  : %f \n",arr[1]);
}
