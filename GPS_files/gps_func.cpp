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

float smoother_func(float cur_val,float val_array[32])
{
    float dum_val = 0;
    int int_val = 0;
    for(int_val = (32-2); int_val>=0; int_val--)
    {
      val_array[int_val+1] = val_array[int_val];
      dum_val = dum_val+val_array[int_val];
    }
    val_array[0] = cur_val;
    dum_val = dum_val+cur_val;
    return dum_val/32;// /8
}


int main ()
{
  float s_arr[32] = {0};
  float c_val = 1;
  float sum_out;
  for(int a1 = 0; a1<32; a1++)
  {
   s_arr[a1] = a1+2;
  }
  printf("Arr 0 before  : %f \n",s_arr[0]);
  printf("Arr 1 before  : %f \n",s_arr[1]);
  printf("Arr 2 before  : %f \n",s_arr[2]);
  printf("Arr 29 before  : %f \n",s_arr[29]);
  printf("Arr 30 before  : %f \n",s_arr[30]);
  printf("Arr 31 before  : %f \n",s_arr[31]);
  sum_out = smoother_func(c_val,s_arr);
  printf("Sum out : %f \n",sum_out);
  printf("Arr 0 before  : %f \n",s_arr[0]);
  printf("Arr 1 before  : %f \n",s_arr[1]);
  printf("Arr 2 before  : %f \n",s_arr[2]);
  printf("Arr 29 before  : %f \n",s_arr[29]);
  printf("Arr 30 before  : %f \n",s_arr[30]);
  printf("Arr 31 before  : %f \n",s_arr[31]);
}
