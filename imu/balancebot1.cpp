#include <iostream>
#include "mpu9250.h"
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdio.h>
#include <math.h>
//#include <Eigen/Dense>  // copy the Eigen folder into /usr/local/include/


#define BILLION 1000000000
#define ACCFACTOR 2.0/32768.0
#define UPERIOD 10000	// in microsecond
#define ENC_REV_CNT 1632.0
#define DELTAT 0.01
#define Kp 4
#define Kd 4
using namespace std;
//using namespace Eigen;


int main(){

	MPU9250 imu(2,0x68);
	imu.getReady();
	imu.deltat=DELTAT;
	imu.alpha=0.90;

	imu.pitch_offset=0;//angle_offset;
	usleep(2000000);

	while(true){
		imu.getData();
//		imu.deltat=DELTAT;
		imu.getAngle();
		//imu.pitch

		printf("IMU.Ax, IMU.Ay, IMU.Az: %06.6f, %06.6f, %06.6f, %06.6f, %06.6f, %06.6f \n", imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz);

		usleep(50000);


	}


/*
   	for(int i=0;i<100;i++){
   		gpio.clearDataOut(GPIO_30);
		pwm.cmpA(0.01*i);
   		pwm.cmpB(0.01*i);
   		eqep1.readPosition();
		eqep2.readPosition();
   		imu.getData();
   		imu.getAngle();
   		cout << imu.pitch << "," << imu.ax << "g, " << imu.ay << "g, " << imu.az << "g, " << imu.gx << "dps, " << imu.gy << "dps, " << imu.gz << "dps, " << eqep1.eqepPosition << ", " << eqep2.eqepPosition << "\n";
   		usleep(100000);	
   	}



	MPU9250 imu(2,0x68);
	imu.getReady();

	long int interval1=0,interval2=0;
	long int adjust=0;
	struct timespec prev_time,cur_time, cur_lapse_time, prev_lapse_time;
	long int accum_err=0, err=0,err1=0,err1_prev=0,err1_d=0;
	clock_gettime(CLOCK_MONOTONIC,&prev_time);
	float unitLength=0;
*/

	return 0;
}
