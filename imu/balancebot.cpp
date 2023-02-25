#include "ros/ros.h"
#include <iostream>
#include "butter.h"
#include "pwm.h"
#include "gpio.h"
#include "eqep.h"
#include "kalmanfilter.h"
#include "mpu9250.h"
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdio.h>
#include <math.h>
#include "balancebot/myMessage.h"
#include <Eigen/Dense>  // copy the Eigen folder into /usr/local/include/

#define BILLION 1000000000
#define ACCFACTOR 2.0/32768.0
#define UPERIOD 5000  // in microsecond
#define ENC_REV_CNT 1632.0
#define DELTAT 0.005
#define Kp 1
#define Kd 2
using namespace std;
using namespace Eigen;


int main(int argc, char **argv){

  ros::init(argc,argv,"balancebot");
  ros::NodeHandle nh;
  
  ros::Publisher pub=nh.advertise<balancebot::myMessage>("balancebot_msg",100);
  ros::Rate loop_rate(200);
  
  BUTTER butter;

  // IMU setting
  PWM pwm(1);

  GPIO gpio(0);
  gpio.open();

  MPU9250 imu(2,0x68);
  imu.getReady();
  imu.deltat=DELTAT;
  imu.alpha=0.98;

  eQEP eqep1(1);
  eQEP eqep2(2);
  eqep1.open();
  eqep2.open();

  // PWMPERIOD=(TBPRD+1)/TBCLK
  // TBCLK=SYSCLOKOUT/CLKDIV/HSPCLKDIV,    SYSCLOKOUT=100Mhz
  // CLKDIV_1 CLKDIV_2 CLKDIV_4 CLKDIV_8 CLKDIV_16 CLKDIV_32 CLKDIV_64 CLKDIV_128
  // HSPCLKDIV_1 HSPCLKDIV_2 HSPCLKDIV_4 HSPCLKDIV_6 HSPCLKDIV_8 HSPCLKDIV_10 HSPCLKDIV_12 HSPCLKDIV_14
  // Note that Max for period is 65535 (16bit)
  // combination example:
  // 100kHz: 999, HSPCLKDIV_1, CLKDIV_1
  // 1kHz: 4999, HSPCLKDIV_10, CLKDIV_2
  //pwm.getReady(999,HSPCLKDIV_1,CLKDIV_1); //100kHz
  //pwm.getReady(999,HSPCLKDIV_2,CLKDIV_1); //50kHz
  //pwm.getReady(749,HSPCLKDIV_2,CLKDIV_2); //33.3kHz 
  pwm.getReady(999,HSPCLKDIV_2,CLKDIV_2); //25kHz // the best
  //pwm.getReady(999,HSPCLKDIV_10,CLKDIV_1);  //10kHz
  //pwm.getReady(999,HSPCLKDIV_10,CLKDIV_2);  //5kHz
  //pwm.getReady(4999,HSPCLKDIV_10,CLKDIV_2); //1kHz

  int count;
  
  Vector4d x0, xhat;
  Vector2d y;
  VectorXd control_u(1);
  Matrix4d AA, I=MatrixXd::Identity(4,4);
  MatrixXd BB=MatrixXd::Zero(4, 1);
  MatrixXd CC=MatrixXd::Zero(2, 4);
  Matrix4d Qk;
  Matrix2d Rk;

  x0 << 0,0,0,0;
  DKF dkf(x0,4,1,2);



  long int interval1,interval2;
  long int adjust;
  struct timespec prev_time,cur_time, cur_lapse_time, prev_lapse_time;
  long int accum_err, err,err1,err1_prev,err1_d;
  float unitLength;
  float angle;
  float angle_offset;





  float theta, phi, phi1, phi2, theta_dot, phi_dot, phi_dot1, phi_dot2, delta_theta, delta_phi1, delta_phi2;
  float theta_butter, theta_dot_butter, phi_butter, phi_dot_butter;
  int encoder1,encoder2;
  
  
  float k1,k2,k3,k4;
  float f1, f2, f3, f4, control, dutycycle, dutycycle1, dutycycle2, dutycycle11, dutycycle22, f3_prev;
  float u, u1, u2;
  float theta_next, theta_dot_next, phi_next, phi_dot_next;
  float m, r, Id, M, l, Ip, g; 
  float lMr, IdmMr2, glM, l2M2r2, Ipl2M, lMIpl2Mr, Ipl2MIdmMr2;
  float Rm, Kb, Km;

  MatrixXd A(4,4), B(4,1), K(1,4);  // double

  VectorXd x_next(4), x_cur(4) ,tau(1), f(4), x_hat(4), x_hat_next(4);
  VectorXd theta_in=VectorXd::Zero(5), theta_out=VectorXd::Zero(5);
  VectorXd phi_in=VectorXd::Zero(5), phi_out=VectorXd::Zero(5);
  VectorXd theta_dot_in=VectorXd::Zero(5), theta_dot_out=VectorXd::Zero(5);
  VectorXd phi_dot_in=VectorXd::Zero(5), phi_dot_out=VectorXd::Zero(5);

  float data[12][20000];
  float phierr, phierr_dot, phierr_prev;
  float phierr1, phierr_dot1, phierr_prev1;
  float phierr2, phierr_dot2, phierr_prev2;
  float theta_accul;
  
  float m11, m22, m12, m21, c21, g1;

  float angle_adjust=0.0;

  float PID1, PID2;









  while(1)
  {
  
  
  
    usleep(1000000);

    // initialization
    x0 << 0,0,0,0;
    xhat=x0;
    
    Qk<<0.01,0,0,0,
        0,0.01,0,0,
        0,0,0.01,0,
        0,0,0,0.01;
    
    Rk<<0.01,0,
        0,0.01;
   
    interval1=0;
    interval2=0;
    adjust=0;
    accum_err=0;
    err=0;
    err1=0;
    err1_prev=0;
    err1_d=0;
    unitLength=0;
    angle=0.0;
    count=0;
      

    clock_gettime(CLOCK_MONOTONIC,&prev_time);  

      // Calibration
    gpio.dataOut((gpio.dataIn() | GPIO_05) & ~ GPIO_04);  // turn idle LED (red) on 
    cout << "Calibration starts in 2 seconds after button 1." << endl;
    cout << "Hold the robot upright for about 2 seconds." << endl;
    cout << "During calibration, both LEDs will be on." << endl;  
  

    cout << "Press button 1 to start calibration." << endl;
    while((gpio.dataIn() & GPIO_15)==GPIO_15)
    {
      //cout << (gpio.dataIn() & GPIO_03) <<endl;
      usleep(10000);
    }  

    usleep(2000000);  // wait two seconds
    
    cout << "Calibration started." << endl;
    gpio.dataOut(gpio.dataIn() | GPIO_05 | GPIO_04);  // turn both LEDs on for calibration
    angle_offset=0.0;
    for (int i=0;i<400;i++)
    {
      clock_gettime(CLOCK_MONOTONIC,&prev_time);
      imu.getData();  

      clock_gettime(CLOCK_MONOTONIC,&cur_time);
      interval1=(cur_time.tv_sec-prev_time.tv_sec)*BILLION + (cur_time.tv_nsec-prev_time.tv_nsec);
      err=UPERIOD-interval1/1000.0;
      err1=UPERIOD-interval2/1000.0;
      accum_err+=err1;
      err1_d=err1-err1_prev;
      err1_prev=err1;
      adjust=err+0.1*err1+0.1*accum_err+0.05*err1_d;
      if(adjust<0)
        adjust=0;
      
      usleep(adjust);
      clock_gettime(CLOCK_MONOTONIC,&cur_lapse_time);
      interval2=(cur_lapse_time.tv_sec-prev_lapse_time.tv_sec)*BILLION + (cur_lapse_time.tv_nsec-prev_lapse_time.tv_nsec);    
      imu.getInitAngle();
      angle_offset=(float)i/((float)i+1.0)*angle_offset+1.0/((float)i+1.0)*imu.pitch;
      cout << i << "," << angle_offset << "," << imu.pitch << "," << imu.ax << "g, " << imu.ay << "g, " << imu.az << "g, " << imu.gx << "dps, " << imu.gy << "dps, " << imu.gz << "dps, " << interval2 << "," << err << "," << adjust <<"\n";
      //adjust=0;
      prev_lapse_time=cur_lapse_time;
      //angle=imu.pitch;
    }
    imu.pitch_offset=0;//angle_offset;
    gpio.dataOut(gpio.dataIn() & ~(GPIO_05 | GPIO_04));  // turn both LEDs off
    cout << "Calibration is completed." << endl;
    cout << "Offset angle is " << angle_offset << " radian." << endl;
    cout << "Now, the robot will start making balance in 2 seconds." << endl;
    usleep(2000000);  // wait two seconds  

    gpio.dataOut((gpio.dataIn() | GPIO_04) & ~ GPIO_05);  // turn run LED (green) on   
  
  

    theta=0.0;
    phi=0.0;
    phi1=0.0;
    phi2=0.0;
    theta_dot=0.0;
    phi_dot=0.0;
    phi_dot1=0.0;
    phi_dot2=0.0;
    delta_theta=0.0;
    delta_phi1=0.0;
    delta_phi2=0.0;
    theta_butter=0.0;
    theta_dot_butter=0.0;
    phi_butter=0.0;
    phi_dot_butter=0.0;
    
    eqep1.readPosition();
    eqep2.readPosition();
    encoder1=-eqep1.eqepPosition;
    encoder2=eqep2.eqepPosition;
    
    k1=-9.64;
    k2=-3.46;
    k3=-0.032;
    k4=-0.065;
    //k1=-19.925,k2=-10.3,k3=-0.032,k4=-0.0795;
    //k1=-10.45,k2=-3.40,k3=-0.032,k4=-0.061;
    //k1=-134.2305,k2=-15.2099,k3=-2.2361,k4=-2.7443;
    //k1=-5.0139,k2=-0.6213,k3=-0.0707,k4=-0.0889;
    f1=0.0;
    f2=0.0;
    f3=0.0;
    f4=0.0;
    control=0.0;
    dutycycle=0.0;
    dutycycle1=0.0;
    dutycycle2=0.0;
    dutycycle11=0.0;
    dutycycle22=0.0;
    f3_prev=0.0;
    u=0.0;
    u1=0.0;
    u2=0.0;
    theta_next=0.0;
    theta_dot_next=0.0;
    phi_next=0.0;
    phi_dot_next=0.0;
    m=0.0227;
    r=0.03;
    Id=0.000010206;
    M=0.7781164;
    l=0.08;
    Ip=0.0029;
    g=9.81;
    Rm=5.714;
    Kb=0.4715;
    Km=0.212;  

    lMr=l*M*r;
    IdmMr2=Id+(m+M)*r*r;
    glM=g*l*M;
    l2M2r2=l*l*M*M*r*r;
    Ipl2M=Ip+l*l*M;
    lMIpl2Mr=l*M*(Ip+l*l*M)*r;
    Ipl2MIdmMr2=(Ip+l*l*M)*(Id+(m+M)*r*r);
   
    
    /*
    A << 0,1,0,0,
    198.4884,0,0,0,
    0,0,0,1,
    -507.4002,0,0,0;  

    B << 0,
    -1141.265825,
    0,
    4268.9362;
  */
    A << 0,1,0,0,
    198.4884,0,0,0,
    0,0,0,1,
    -705.889,0,0,0;
    
    B << 0,
    -1462.178,
    0,
    6551.468;
    
    AA=I+A*DELTAT;
    BB=B*DELTAT;
    //CC=I;
    CC<<1,0,0,0,
    0,0,1,0;
    
    //K << -5.0139,-0.6213,-0.0707,-0.0889;
    //K << -5.0139,-0.06213,-0.0707,-0.00889;
    
    
    
      
  

    //x << 1, 1, 1, 1;
    //u << 1;
    
    x_hat<<theta,theta_dot,(phi1+phi2)/2,(phi_dot1+phi_dot2)/2;
    
    
    phierr=0.0;
    phierr_dot=0.0;
    phierr_prev=0.0;
    phierr1=0.0;
    phierr_dot1=0.0;
    phierr_prev1=0.0;
    phierr2=0.0;
    phierr_dot2=0.0;
    phierr_prev2=0.0;
    theta_accul=0.0;
    
    m11=Ipl2M;
    m22=IdmMr2;
    m12=0.0;
    m21=m12;
    c21=0.0;
    g1=0.0;  

    interval1=0;
    
  	
    clock_gettime(CLOCK_MONOTONIC,&prev_time);  

    pwm.runPWM(pwm.pwm1a_enable);
    pwm.runPWM(pwm.pwm1b_enable);


  	while(ros::ok() && angle<PI/4 && angle>-PI/4 && count < 20000) // 100 sec
  	{
      
      clock_gettime(CLOCK_MONOTONIC,&cur_time);  

      interval1+=(cur_time.tv_sec-prev_time.tv_sec)*BILLION + (cur_time.tv_nsec-prev_time.tv_nsec);
    
      if(count%200 == 0)
      {
          //cout << interval1 << endl;      
          printf("%d percent\r",(int)(count/200));
          fflush(stdout);  // This line flushes stdout inmediatly
      }
    
      prev_time=cur_time;  

      /////////////////////////////////////////////
      // get sensor data and estimate the states.//
      /////////////////////////////////////////////
      eqep1.readPosition();
      eqep2.readPosition();
      delta_phi1=(float)(-eqep1.eqepPosition-encoder1)/ENC_REV_CNT*2*PI;
      delta_phi2=(float)(eqep2.eqepPosition-encoder2)/ENC_REV_CNT*2*PI;   
      encoder1=-eqep1.eqepPosition;
      encoder2=eqep2.eqepPosition;  

      phi+=(delta_phi1+delta_phi2)/2;
      phi_dot=(delta_phi1+delta_phi2)/2/DELTAT;  

      phi1+=delta_phi1;
      phi2+=delta_phi2;
      phi_dot1=delta_phi1/DELTAT;
      phi_dot2=delta_phi2/DELTAT;  
  

      //cout << encoder1 << "," << encoder2 << endl;  

      //cout << phi << ", " << delta_phi1 << ", " << delta_phi2 << endl;  

      imu.getData();    // this process takes about 1.85 ms
      imu.deltat=DELTAT;
      imu.getAngle();
      //theta_dot=(imu.pitch-angle_offset-theta)/DELTAT;
      theta_dot=imu.gx/180.0*PI;;
      theta=imu.pitch-angle_offset-angle_adjust;
      //theta=imu.pitch;
      angle=theta;        

      // Low Pass Filter
  /*  
      theta_in=butter.rshift(theta_in);
      theta_out=butter.rshift(theta_out);
      theta_in(0)=theta;
      theta_butter=butter.filter(theta_in,theta_out);
      theta_out(0)=theta_butter;  

      theta_dot_in=butter.rshift(theta_dot_in);
      theta_dot_out=butter.rshift(theta_dot_out);
      theta_dot_in(0)=theta_dot;
      theta_dot_butter=butter.filter(theta_dot_in,theta_dot_out);
      theta_dot_out(0)=theta_dot_butter;  
  

      phi_in=butter.rshift(phi_in);
      phi_out=butter.rshift(phi_out);
      phi_in(0)=phi;
      phi_butter=butter.filter(phi_in,phi_out);
      phi_out(0)=phi_butter;  

      phi_dot_in=butter.rshift(phi_dot_in);
      phi_dot_out=butter.rshift(phi_dot_out);
      phi_dot_in(0)=phi_dot;
      phi_dot_butter=butter.filter(phi_dot_in,phi_dot_out);
      phi_dot_out(0)=phi_dot_butter;
  */  
  

  /*   
      m12=lMr*cos(xhat(0));
      c21=-lMr*sin(xhat(0))*xhat(1);
      g1=-glM*sin(xhat(0));
      
      u=(m11*m22-m12*m21)*(-Kd*xhat(1)-Kp*xhat(0))+m22*g1-m12*c21*xhat(1);
      u/=-(m22+m12);
      u=-u;     

      x_cur << theta, theta_dot, phi, phi_dot;
    */
      x_hat<<theta,theta_dot,(phi1+phi2)/2,(phi_dot1+phi_dot2)/2;  
  
  
  

      // Control, LQR
      u1=-(k1*theta+k2*theta_dot+k3*phi1+k4*phi_dot1);
      u2=-(k1*theta+k2*theta_dot+k3*phi2+k4*phi_dot2);  

      tau<<(u1+u2)/2;
      x_hat_next=AA*x_hat+BB*tau;
      //tau=K*x_cur;  // take - sign since motor polarity is flipped.
    //  tau=K*xhat;  // take - sign since motor polarity is flipped.
      
    //  control_u<<-u;
      //control_u=-tau;
    //  y<<theta,phi;  

    //  dkf.Update(AA, BB, CC, Qk, Rk, control_u, y);   // this process takes about 1.75 ms
    //  xhat=dkf.xhatk1;
      
      
    //  f=A*xhat+B*(-tau);
      //u=tau(0);
      //m=0.0227, r=0.03, Id=0.000010206, M=0.7781164, l=0.08, Ip=0.0029, g=9.81;
      
  //    m12=lMr*cos(theta);
  //    c21=-lMr*sin(theta)*theta_dot;
  //    g1=-glM*sin(theta);
      
  //    u=(m11*m22-m12*m21)*(-Kd*theta_dot-Kp*theta)+m22*g1-m12*c21*theta_dot;
  //    u/=-(m22+m12);
  //    u=-u;  

      
  //    f1=theta_dot;
  //    f2=-lMr*u*cos(theta)+IdmMr2*(-u+glM*sin(theta))-l2M2r2*cos(theta)*sin(theta)*theta_dot*theta_dot;
      //f2=-lMr*u*cos(theta)+IdmMr2*(-u+glM*sin(theta));
  //    f2/=Ipl2MIdmMr2-l2M2r2*cos(theta)*cos(theta);
  //    f3=phi_dot;
  //    f4=Ipl2M*u-lMr*cos(theta)*(-u+glM*sin(theta))+lMIpl2Mr*sin(theta)*theta_dot*theta_dot;
      //f4=Ipl2M*u-lMr*cos(theta)*(-u+glM*sin(theta));
  //    f4/=Ipl2MIdmMr2-l2M2r2*cos(theta)*cos(theta);  

      // Estimate next state
  //    x_next=f*DELTAT+x_cur;
     
     
  //    theta_next=f1*DELTAT+theta;
  //    theta_dot_next=f2*DELTAT+theta_dot;
  //    phi_next=f3*DELTAT+phi;
  //    phi_dot_next=f4*DELTAT+phi_dot;
  /*
      f1=f(0);
      f2=f(1);
      f3=f(2);
      f4=f(3);
      
      theta_next=x_next(0);
      theta_dot_next=x_next(1);
      phi_next=x_next(2);
      phi_dot_next=x_next(3);
  */
      // PD control tracking
      //control=Kp*f3*DELTAT+Kd*f4*DELTAT;
  //    control=Kp*f3*DELTAT+Kd*(f3-f3_prev);
  //    f3_prev=f3;  

      //unitLength=sqrt(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az);
      //cout << imu.pitch << "," << imu.ax << "g, " << imu.ay << "g, " << imu.az << "g, " << imu.gx << "dps, " << imu.gy << "dps, " << imu.gz << "dps, " << interval2 << "," << err << "," << adjust <<"\n";
      //adjust=0;
      //cout << control << endl;  
  

      //cout << imu.pitch << "," << imu.ax << "g, " << imu.ay << "g, " << imu.az << "g, " << imu.gx << "dps, " << imu.gy << "dps, " << imu.gz << "dps, " << interval2 << "," << err << "," << adjust <<"\n";  

      //dutycycle=-control*0.1;
       //dutycycle=-theta*5-theta_dot*0.1;
      //dutycycle=-u*0.1;
      
      //dutycycle=-(u+0.0245209*(phi_dot-theta_dot))/0.4449;  

  //    dutycycle=-(u+0.0245209*(xhat(3)-xhat(1)))/0.4449;
      
      //dutycycle=0.2;
  //    dutycycle=u*40;
      
      
      //dutycycle=tau(0)*5;
      //dutycycle=control*0.1;  
  

  /*
      if(count<1000 || count >9000){
        phierr=phi-0;
        phierr1=phi1-0;
        phierr2=phi2-0;
      }
      else{
        phierr=phi-sin(count*0.1)*0.4;
        phierr1=phi1-sin(count*0.1)*0.4;
        phierr2=phi2-sin(count*0.1)*0.4;
      }
      
      //phierr=xhat(2)-phi_next;
      phierr_dot=phierr-phierr_prev;
      dutycycle=phierr*2+phierr_dot*0.1;
      phierr_prev=phierr;  

      phierr_dot1=phierr1-phierr_prev1;
      dutycycle1=phierr1*2+phierr_dot1*0.1;
      phierr_prev1=phierr1;  

      phierr_dot2=phierr2-phierr_prev2;
      dutycycle2=phierr2*2+phierr_dot2*0.1;
      phierr_prev2=phierr2;
  */  
  

      
      //dutycycle=-u;
      //dutycycle1=dutycycle;
      //dutycycle=-(theta-phi)*0.1;
     
      //if(theta>-2 && theta<2)
      //  dutycycle=0;
      //dutycycle=0.5;
      //dutycycle=0.5;  
  
  

      //dutycycle1=Rm*u1/Km+Kb*phi_dot1;
      //dutycycle2=Rm*u2/Km+Kb*phi_dot2;  
  
  
  

      //dutycycle1=Rm*u1/Km;
      //dutycycle2=Rm*u2/Km;
      dutycycle1=Kp*(x_hat_next(2)-x_hat(2))+Kd*(x_hat_next(3)-x_hat(3));
      dutycycle2=Kp*(x_hat_next(2)-x_hat(2))+Kd*(x_hat_next(3)-x_hat(3));
      
      
      theta_accul+=theta;
      // temporary PID
      //float PID=15*theta+0.0*theta_dot+0.25*theta_accul;
      //float PID=15*theta+0.005*theta_dot+0.25*theta_accul; // this works
      //float PID=10*theta+0.005*theta_dot+0.25*theta_accul;  // this also works
      //float PID=20*theta+0.005*theta_dot+0.4*theta_accul;  // this also works
      
      // Adjust angle misalignment
      angle_adjust=0.001*theta_accul;  

      // after putting the battery on top
      //float PID=10*theta+0.005*theta_dot+0.25*theta_accul+0.5*phi;  // this also works => too weak
      //float PID=20*theta+0.005*theta_dot+0.25*theta_accul+0.5*phi;  // this also works
      //float PID=20*theta+0.005*theta_dot+0.25*theta_accul+0.5*phi;  // this also works
      //float PID=20*theta+0.005*theta_dot+1*theta_accul+0.5*phi;  // this also works
      
      PID1=20*theta+0.005*theta_dot+0.25*theta_accul+0.5*phi;  // this also works
      PID2=20*theta+0.005*theta_dot+0.25*theta_accul+0.5*phi;  // this also works  

      dutycycle1=PID1;
      dutycycle2=PID2;  

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
      //dutycycle1=dutycycle1*atan(abs(theta)*10)*2/PI;
      //dutycycle2=dutycycle2*atan(abs(theta)*10)*2/PI;  

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////    
      dutycycle11=dutycycle1;
      dutycycle22=dutycycle2;  

      dutycycle1=-dutycycle1;
      dutycycle2=-dutycycle2;
      
      // motor 1
      if(dutycycle1<0.0)
      {
        dutycycle1=-dutycycle1;
        if(dutycycle1>1)
        {
          dutycycle1=1;
        }
        //dutycycle1=(dutycycle1+1.0/9.0)*0.9;
          pwm.cmpB(dutycycle1);          // motor 1
          gpio.clearDataOut(GPIO_31);
      } else
      {
        if(dutycycle1>1)
        {
          dutycycle1=1;
        }      
        //dutycycle1=(dutycycle1+1/9)*0.9;
          pwm.cmpB(1-dutycycle1);  // motor 1
          gpio.setDataOut(GPIO_31);
      }  

      // motor 2
      if(dutycycle2<0.0)
      {
        dutycycle2=-dutycycle2;
        if(dutycycle2>1)
        {
          dutycycle2=1;
        }
        //dutycycle2=(dutycycle2+1.0/9.0)*0.9;
          pwm.cmpA(dutycycle2);          // motor 2
          gpio.clearDataOut(GPIO_30);
      } else
      {
        if(dutycycle2>1)
        {
          dutycycle2=1;
        }
        //dutycycle2=(dutycycle2+1/9)*0.9;
          pwm.cmpA(1-dutycycle2);  // motor 2
          gpio.setDataOut(GPIO_30);
      }  
  

  //    clock_gettime(CLOCK_MONOTONIC,&cur_lapse_time);
  //    interval2=(cur_lapse_time.tv_sec-prev_lapse_time.tv_sec)*BILLION + (cur_lapse_time.tv_nsec-prev_lapse_time.tv_nsec);    
      //cout << angle << ", " << control << ", " << dutycycle << "," << interval2 << "," << err << "," << adjust <<"\n";
      //printf("%f, %f, %f,%f,%f,%f,%lu,%lu,%f,%f,%f,%f,%f,%f,%f, %ld, %ld, %ld\n", , phi, phi_dot, encoder1, encoder2, delta_phi1, delta_phi2,  f1, f2, f3, f4, control, u, interval2, err, adjust);
     
     /*
      printf("Angle, Theta, Theta_dot: %06.6f, %06.6f, %06.6f \n", angle, theta, theta_dot);
      printf("phi, phi_dot: %06.6f, %06.6f\n",phi, phi_dot);
      printf("encoder1, encoder2, delta_phi1, delta_phi2: %09d,%09d,%06.6f,%06.6f\n",encoder1, encoder2, delta_phi1, delta_phi2);
      printf("f1, f2, f3, f4: %06.6f, %06.6f, %06.6f, %06.6f\n",f1, f2, f3, f4);
      printf("control, u, dutycycle: %06.6f, %06.6f, %06.6f\n", control, u, dutycycle1);
      printf("interval2, err, adjust: %10ld, %10ld, %10ld\n",interval2, err, adjust);
      printf("---------------------------------------------------------------\n");
      */
     /**/
     data[0][count]=theta;
     data[1][count]=theta_dot;
     data[2][count]=phi1;
     data[3][count]=phi_dot1;
     data[4][count]=phi2;
     data[5][count]=phi_dot2;
  //   data[4][count]=theta_butter;
  //   data[5][count]=theta_dot_butter;
     data[6][count]=u1;
     data[7][count]=u2;
     data[8][count]=dutycycle11;
     data[9][count]=dutycycle22;
     data[10][count]=x_hat_next(2);
     data[11][count]=theta_accul;
          
  //     cout << imu.pitch << "," << imu.ax << "g, " << imu.ay << "g, " << imu.az << "g, " << imu.gx << "dps, " << imu.gy << "dps, " << imu.gz << "dps, " << interval2 << "," << err << "," << adjust <<"\n";  

       
  //     balancebot::myMessage msg;
  //     msg.data=count;
  //     msg.pitch=imu.pitch;
  //     pub.publish(msg);  // this process takes about 1.65 ms
  //     ROS_INFO("send msg = %d, %f",count, imu.pitch);
       count++;
       loop_rate.sleep();
    }  

  /*  
    clock_gettime(CLOCK_MONOTONIC,&cur_time);
    interval1+=(cur_time.tv_sec-prev_time.tv_sec)*BILLION + (cur_time.tv_nsec-prev_time.tv_nsec);
    cout << interval1 << endl;
    
      if(count%200 == 0)
      {
          cout << interval1 << endl;      
          interval1=0;
      }
  */
  //cout << encoder1 << "," << encoder2 << endl; 


    gpio.dataOut(gpio.dataIn() & ~(GPIO_05 | GPIO_04));  // turn both LEDs off
    
    // stop motors
    pwm.cmpA(0);          // motor 2
    pwm.cmpB(0);          // motor 1
    gpio.clearDataOut(GPIO_31 | GPIO_30); 
    pwm.stopPWM(pwm.pwm1a_enable);
    pwm.stopPWM(pwm.pwm1b_enable);
    
    FILE * pFile;
    pFile = fopen ("/home/ubuntu/catkin_ws/src/balancebot/data.txt","w");
    for (int i=0 ; i<count+1 ; i++)
    {
      fprintf (pFile, "%f\t",data[0][i]);
      fprintf (pFile, "%f\t",data[1][i]);
      fprintf (pFile, "%f\t",data[2][i]);
      fprintf (pFile, "%f\t",data[3][i]);
      fprintf (pFile, "%f\t",data[4][i]);
      fprintf (pFile, "%f\t",data[5][i]);
      fprintf (pFile, "%f\t",data[6][i]);
      fprintf (pFile, "%f\t",data[7][i]);
      fprintf (pFile, "%f\t",data[8][i]);
      fprintf (pFile, "%f\t",data[9][i]);
      fprintf (pFile, "%f\t",data[10][i]);
      fprintf (pFile, "%f\n",data[11][i]);
    }
    fclose (pFile);
  	  
  

    cout << "Press button 1 to restart. Press button 2 to stop the program." << endl;
    while(1)
    {
      if((gpio.dataIn() & GPIO_15)!=GPIO_15)
        break;
      if((gpio.dataIn() & GPIO_03)!=GPIO_03)
        return 0;
      
      usleep(10000);
    }  


  }

  gpio.clearDataOut(0xFFFFFFFF);
  pwm.close();  

	return 0;
}