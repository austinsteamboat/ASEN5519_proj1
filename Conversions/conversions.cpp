#include <iostream>
#include <math.h>
using namespace std;

#define GRAV_CU 9.81
#define PI_CU 3.14159

//Convert turn rate in rad/s to roll angle in centidegrees
float roll_ang_conv(float chidot, float airspeed) {
	float roll_ang = atan((airspeed*chidot)/GRAV_CU);
	return (roll_ang*180/PI_CU)*100;
}

//Convert climb rate in m/s to pitch angle in centidegrees
float pitch_ang_conv(float climbrate, float airspeed) {
	float pitch_ang = asin(climbrate/airspeed);
	return (pitch_ang*180.0/PI_CU)*100.0;
}

int main() {
	
	//test roll angle conversion
	float speed = 20;
	float turn_rate = -0.1;
	float roll;
	roll = roll_ang_conv(turn_rate, speed);
	cout << roll << " centidegrees" << endl;

	//test pitch angle conversion
	float climb_rate = -5.0;
	float pitch;
	pitch = pitch_ang_conv(climb_rate,speed);
	cout << pitch << " centidegrees" << endl;
	
	return 0;
}
