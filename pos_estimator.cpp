

void pos_estimate(double Cur_NED_gps[3],double Old_NED_gps[3], int time_ind, double position_est[3])
{
float N_slope;
float E_slope;
float D_slope;

N_slope = Cur_NED_gps[0] - Old_NED_gps[0];
E_slope = Cur_NED_gps[1] - Old_NED_gps[1];
D_slope = Cur_NED_gps[2] - Old_NED_gps[2];

position_est[0] = Cur_NED_gps[0] + N_slope * time_ind * .02;
position_est[1] = Cur_NED_gps[1] + E_slope * time_ind * .02;
position_est[2] = Cur_NED_gps[2] + D_slope * time_ind * .02;
}