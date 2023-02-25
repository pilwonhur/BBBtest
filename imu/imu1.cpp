#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include "I2Cdev.h"

#define I2C_BUS "/dev/i2c-2" // Use I2C bus 2
#define MPU9250_ADDR 0x68    // MPU-9250 I2C address
//#define AK8963_ADDRESS   0x0C

using namespace std;
int file;
//int filem;

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
int16_t magX, magY, magZ;

void writeRegister(uint8_t reg, uint8_t value) {
  if (write(file, &reg, 1) != 1) {
    std::cerr << "Failed to write register address" << std::endl;
    exit(1);
  }
  if (write(file, &value, 1) != 1) {
    std::cerr << "Failed to write register value" << std::endl;
    exit(1);
  }
}

void readRegisters(uint8_t reg, uint8_t count, uint8_t* data) {
  if (write(file, &reg, 1) != 1) {
    std::cerr << "Failed to write register address" << std::endl;
    exit(1);
  }
  if (read(file, data, count) != count) {
    std::cerr << "Failed to read data" << std::endl;
    exit(1);
  }
}
/*
void writeRegisterM(uint8_t reg, uint8_t value) {
  if (write(filem, &reg, 1) != 1) {
    std::cerr << "Failed to write register address" << std::endl;
    exit(1);
  }
  if (write(filem, &value, 1) != 1) {
    std::cerr << "Failed to write register value" << std::endl;
    exit(1);
  }
}

void readRegistersM(uint8_t reg, uint8_t count, uint8_t* data) {
  if (write(filem, &reg, 1) != 1) {
    std::cerr << "Failed to write register address" << std::endl;
    exit(1);
  }
  if (read(filem, data, count) != count) {
    std::cerr << "Failed to read data" << std::endl;
    exit(1);
  }
}
*/
int main() {
  file = open(I2C_BUS, O_RDWR);
  if (file < 0) {
    std::cerr << "Failed to open the bus" << std::endl;
    return 1;
  }
  if (ioctl(file, I2C_SLAVE, MPU9250_ADDR) < 0) {
    std::cerr << "Failed to connect to the sensor" << std::endl;
    return 1;
  }

//  filem=open(I2C_BUS, O_RDWR);
//  ioctl(filem, I2C_SLAVE, AK8963_ADDRESS);

  // Initialize the MPU-9250
  writeRegister(0x6B, 0x00); // Wake up the sensor
  writeRegister(0x1A, 0x06); // Set accelerometer and gyroscope to 1kHz sample rate
  writeRegister(0x1B, 0x18); // Set gyroscope range to +/-2000 degrees/second
  writeRegister(0x1C, 0x08); // Set accelerometer range to +/-4g
  write_byte(0x0C,0x37, 0x02);
//  i2c_smbus_write_byte_data(file, 0x37, 0x02); // Enable bypass mode for magnetometer
  usleep(100000);
  // Initialize the AK8963 magnetometer
  writeRegister(0x0A, 0x0F);
//  i2c_smbus_write_byte_data(file, 0x0A, 0x0F);
  usleep(100000);
  writeRegister(0x0A, 0x16);
//  i2c_smbus_write_byte_data(file, 0x0A, 0x16); // Set magnetometer to 16-bit resolution
  usleep(100000);
  // Read and print sensor data
  while (true) {
    // Read accelerometer data
    uint8_t accelData[6];
    readRegisters(0x3B, 6, accelData);
    accelX = (accelData[0] << 8) | accelData[1];
    accelY = (accelData[2] << 8) | accelData[3];
    accelZ = (accelData[4] << 8) | accelData[5];
    // Read gyroscope data
    uint8_t gyroData[6];
    readRegisters(0x43, 6, gyroData);
    gyroX = (gyroData[0] << 8) | gyroData[1];
    gyroY = (gyroData[2] << 8) | gyroData[3];
    gyroZ = (gyroData[4] << 8) | gyroData[5];
    // Read magnetometer data
    uint8_t magData[8];
    readRegisters(0x02,8,magData);
//    i2c_smbus_read_bytes_data(file, 0x02, 8, magData);
    //if (magData[0] & 0x01){
    //  if((magData[0] & 0x02)){
        if ((magData[7] & 0x08)) { // Check if magnetic sensor overflow bit is set
          magX = (magData[2] << 8) | magData[1];
          magY = (magData[4] << 8) | magData[3];
          magZ = (magData[6] << 8) | magData[5];
        }
    //  }
    //}
    // Convert raw data to physical units
    float accelScale = 4.0/32768.0; // Accelerometer sensitivity is 4g for range +/-4g
    float gyroScale = 2000.0/32768.0; // Gyroscope sensitivity is 2000 degrees/second for range +/-2000 degrees/second
    float magScale = 4912.0/32760.0; // Magnetometer sensitivity is 4912 microTesla for range +/-4800 microTesla
    float ax = accelX * accelScale;
    float ay = accelY * accelScale;
    float az = accelZ * accelScale;
    float gx = gyroX * gyroScale;
    float gy = gyroY * gyroScale;
    float gz = gyroZ * gyroScale;
    float mx = magX * magScale;
    float my = magY * magScale;
    float mz = magZ * magScale;
    // Apply sensor fusion to obtain orientation
    float roll, pitch, yaw;
    float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0; // Initialize quaternion
    float dt = 0.01; // Time step is 10ms
    float beta = 0.1; // Fusion gain
    float recipNorm;
    // Normalization of accelerometer and magnetometer measurements
    recipNorm = 1/sqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    recipNorm = 1/sqrt(mx*mx + my*my + mz*mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    // Auxiliary variables to avoid repeated arithmetic
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
    // Gradient descent algorithm to minimize error between estimated direction of gravity and measured direction of gravity from accelerometer
    float hx, hy, hz, bx, bz;
    hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
    hy = 2*mx*(q1q2 + q0q3) + my*(0.5 - q1q1 - q3q3) + mz*(q2q3 - q0q1);
    hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);
    bx = sqrt(hx*hx + hy*hy);
    bz = hz;
    roll = atan2(ay, az);
    pitch = atan2(-ax, bx);
    float s1 = 2*(q0*q2 - q1*q3);
    float s2 = 2*(q2*q3 + q0*q1);
    float s3 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    yaw = atan2(s1, s2);
    // Complementary filter to combine accelerometer and gyroscope data
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    ex = (ay*q3 - az*q2) + beta*gx;
    ey = (az*q1 - ax*q3) + beta*gy;
    ez = (ax*q2 - ay*q1) + beta*gz;
    gx = gx + ex;
    gy = gy + ey;
    gz = gz + ez;
    vx = q1 + 2*(-q2*q3 + q0*q1);
    vy = q2 + 2*(q1*q3 + q0*q2);
    vz = q3 + 2*(-q1*q2 + q0*q3);
    norm = sqrt(vx*vx + vy*vy + vz*vz);
    if (norm == 0) {
      vx = 0;
      vy = 0;
      vz = 0;
    }
    else {
      vx = vx/norm;
      vy = vy/norm;
      vz = vz/norm;
    }
    float exInt, eyInt, ezInt;
    exInt = 0;
    eyInt = 0;
    ezInt = 0;
    exInt = exInt + ex*dt;
    eyInt = eyInt + ey*dt;
    ezInt = ezInt + ez*dt;
    q0 = q0 + (-q1*vx - q2*vy - q3*vz)*0.5*dt;
    q1 = q1 + (q0*vx + q2*vz - q3*vy)*0.5*dt;
    q2 = q2 + (q0*vy - q1*vz + q3*vx)*0.5*dt;
    q3 = q3 + (q0*vz + q1*vy - q2*vx)*0.5*dt;
    q0 = q0 + exInt;
    q1 = q1 + eyInt;
    q2 = q2 + ezInt;
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0/norm;
    q1 = q1/norm;
    q2 = q2/norm;
    q3 = q3/norm;
    // Print sensor data
//    std::cout << "Roll: " << roll*180/M_PI << " degrees, Pitch: " << pitch*180/M_PI*180/M_PI << " degrees, Yaw: " << yaw*180/M_PI << " degrees" << std::endl;
//    cout << (int)magData[0] << endl;
    cout << mx << "," << my << "," << mz << endl;
    usleep(500000); // Wait for 10ms before updating the data again
  }
  return 0;
}


