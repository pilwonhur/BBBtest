#include "mpu9250.h"
#include <iostream>
#include <sstream>
#include <fcntl.h>
#include <stdio.h>
#include <iomanip>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <math.h>

#define HEX(x) setw(2) << setfill('0') << hex << (int)(x)

using namespace std;

MPU9250::MPU9250(unsigned int bus, unsigned int device) {

	this->Gscale=GFS_1000DPS;
    this->Ascale=AFS_2G;
    this->Mscale=MFS_16BITS;
    this->Mmode=0x02;
    this->pitch_offset=0.0;


  this->file=-1;
  this->bus = bus;
  this->device = device;
  this->open();
}

int MPU9250::open(){
   string name;
   if(this->bus==0) name = BBB_I2C_0;
   else if(this->bus==1) name = BBB_I2C_1;
   else name = BBB_I2C_2;

   if((this->file=::open(name.c_str(), O_RDWR)) < 0){ // to get the compiler to search the namespace just outside of that
      perror("I2C: failed to open the bus\n");
    return 1;
   }
   if(ioctl(this->file, I2C_SLAVE, this->device) < 0){
      perror("I2C: Failed to connect to the device\n");
    return 1;
   }
   return 0;
}

int MPU9250::writeRegister(unsigned int registerAddress, unsigned char value){
   unsigned char buffer[2];
   buffer[0] = registerAddress;
   buffer[1] = value;
   if(::write(this->file, buffer, 2)!=2){ // 
      perror("I2C: Failed write to the device\n");
      return 1;
   }
   return 0;
}

int MPU9250::write(unsigned char value){
   unsigned char buffer[1];
   buffer[0]=value;
   if (::write(this->file, buffer, 1)!=1){
      perror("I2C: Failed to write to the device\n");
      return 1;
   }
   return 0;
}

unsigned char MPU9250::readRegister(unsigned int registerAddress){
   this->write(registerAddress);
   unsigned char buffer[1];
   if(::read(this->file, buffer, 1)!=1){
      perror("I2C: Failed to read in the value.\n");
      return 1;
   }
   return buffer[0];
}
/*
unsigned char* MPU9250::readRegisters(unsigned int number, unsigned int fromAddress){
  	this->write(fromAddress);
  	unsigned char* data = new unsigned char[number];
    if(::read(this->file, data, number)!=(int)number){
       perror("IC2: Failed to read in the full buffer.\n");
     return NULL;
    }
  return data;
}
*/
/*
void MPU9250::readRegisters(unsigned int number, unsigned int fromAddress, unsigned char * data)
{
	this->write(fromAddress);
 	//unsigned char* data = new unsigned char[number];
 	data=new unsigned char[number];
    if(::read(this->file, data, number)!=(int)number){
       perror("IC2: Failed to read in the full buffer.\n");
     //return NULL;
    }
  //return data;   
}
*/
unsigned char *MPU9250::readRegisters(unsigned int number, unsigned int fromAddress)
{
	this->write(fromAddress);
 	unsigned char* data = new unsigned char[number];
    if(::read(this->file, data, number)!=(int)number){
       perror("IC2: Failed to read in the full buffer.\n");
     return NULL;
    }
  return data;   
}


void MPU9250::debugDumpRegisters(unsigned int number){
  cout << "Dumping Registers for Debug Purposes:" << endl;
  //unsigned char *registers = this->readRegisters(number);
  unsigned char *registers;
  registers=this->readRegisters(number,0);
  for(int i=0; i<(int)number; i++){
    cout << HEX(*(registers+i)) << " ";
    if (i%16==15) cout << endl;
  }
  cout << dec;
}

void MPU9250::close(){
  ::close(this->file);
  this->file = -1;
}

MPU9250::~MPU9250() {
  if(file!=-1) this->close();
}


short MPU9250::combineRegisters(unsigned char msb, unsigned char lsb){
   return ((short)msb<<8)|(short)lsb;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::MPU9250SelfTest(float *SelfTestRes) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	//float *SelfTestRes;
  	unsigned char *rawData=new unsigned char[6];
  	unsigned char *selfTest=new unsigned char[6];
  	int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
  	float factoryTrim[6];
  	uint8_t FS = 0;
   

  	this->writeRegister(SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  	this->writeRegister(CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  	this->writeRegister(GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  	this->writeRegister(ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  	this->writeRegister(ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
    rawData=this->readRegisters(6, ACCEL_XOUT_H);        // Read the six raw data registers into data array
    //this->readRegisters(6, ACCEL_XOUT_H,rawData);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    rawData=this->readRegisters(6, GYRO_XOUT_H);       // Read the six raw data registers sequentially into data array
    //this->readRegisters(6, GYRO_XOUT_H,rawData);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
  	this->writeRegister(ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  	this->writeRegister(GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  	usleep(25000);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
    rawData=this->readRegisters(6, ACCEL_XOUT_H);  // Read the six raw data registers into data array
    //this->readRegisters(6, ACCEL_XOUT_H,rawData);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    rawData=this->readRegisters(6, GYRO_XOUT_H);  // Read the six raw data registers sequentially into data array
	//this->readRegisters(6, GYRO_XOUT_H,rawData);  // Read the six raw data registers sequentially into data array    
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }   
  
  // Configure the gyro and accelerometer for normal operation
  	this->writeRegister(ACCEL_CONFIG, 0x00);  
  	this->writeRegister(GYRO_CONFIG,  0x00);  
  	usleep(25000);  // Delay a while to let the device stabilize
  
  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = this->readRegister(SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = this->readRegister(SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = this->readRegister(SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = this->readRegister(SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = this->readRegister(SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = this->readRegister(SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results



  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    SelfTestRes[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
    SelfTestRes[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
  }

  //cout << "selftest: " << SelfTestRes[0] << "," << SelfTestRes[1] << "," << SelfTestRes[2] << "," << SelfTestRes[3] << "," << SelfTestRes[4] << "," << SelfTestRes[5] << endl;
  delete[] rawData;
  delete[] selfTest;
}


// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
//void MPU9250::calibrateMPU9250(float *gyroBias, float *accelBias)
void MPU9250::calibrateMPU9250(float * Bias)//(float *gyroBias)//,float *accelBias)
{  
  	unsigned char *data=new unsigned char[6]; // data array to hold accelerometer and gyro x, y, z, data
  	uint16_t ii, packet_count, fifo_count;
  	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
 
  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  	this->writeRegister(PWR_MGMT_1, 0x80);
  	usleep(100000);
 
 // get stable time source; Auto select clock source to be PLL gyroscope
 // reference if ready else use the internal oscillator, bits 2:0 = 001
  	this->writeRegister(PWR_MGMT_1, 0x01);  
  	this->writeRegister(PWR_MGMT_2, 0x00);
  	usleep(200000);                                    

  // Configure device for bias calculation
  	this->writeRegister(INT_ENABLE, 0x00);   // Disable all interrupts
  	this->writeRegister(FIFO_EN, 0x00);      // Disable FIFO
  	this->writeRegister(PWR_MGMT_1, 0x00);   // Turn on internal clock source
  	this->writeRegister(I2C_MST_CTRL, 0x00); // Disable I2C master
  	this->writeRegister(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  	this->writeRegister(USER_CTRL, 0x0C);    // Reset FIFO and DMP
  	usleep(15000);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  	this->writeRegister(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  	this->writeRegister(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  	this->writeRegister(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  	this->writeRegister(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  	this->writeRegister(USER_CTRL, 0x40);   // Enable FIFO  
  	this->writeRegister(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  	usleep(40000); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  	this->writeRegister(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  	data=this->readRegisters(2, FIFO_COUNTH);	// read FIFO sample count
  	//this->readRegisters(2, FIFO_COUNTH,data);	// read FIFO sample count
  	fifo_count = ((uint16_t)data[0] << 8) | data[1];
  	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    data=this->readRegisters(12, FIFO_R_W);	// read data for averaging
    //this->readRegisters(12, FIFO_R_W,data);	// read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.8 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  this->writeRegister(XG_OFFSET_H, data[0]);
  this->writeRegister(XG_OFFSET_L, data[1]);
  this->writeRegister(YG_OFFSET_H, data[2]);
  this->writeRegister(YG_OFFSET_L, data[3]);
  this->writeRegister(ZG_OFFSET_H, data[4]);
  this->writeRegister(ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  Bias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  Bias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  Bias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  data=this->readRegisters(2, XA_OFFSET_H);	// Read factory accelerometer trim values
  //this->readRegisters(2, XA_OFFSET_H,data);	// Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  data=this->readRegisters(2, YA_OFFSET_H);
  //this->readRegisters(2, YA_OFFSET_H,data);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  data=this->readRegisters(2, ZA_OFFSET_H);
  //this->readRegisters(2, ZA_OFFSET_H,data);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
  
/*
  this->writeRegister(XA_OFFSET_H, data[0]);
  this->writeRegister(XA_OFFSET_L, data[1]);
  this->writeRegister(YA_OFFSET_H, data[2]);
  this->writeRegister(YA_OFFSET_L, data[3]);
  this->writeRegister(ZA_OFFSET_H, data[4]);
  this->writeRegister(ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for display in the main program
   Bias[3] = (float)accel_bias[0]/(float)accelsensitivity; 
   Bias[4] = (float)accel_bias[1]/(float)accelsensitivity;
   Bias[5] = (float)accel_bias[2]/(float)accelsensitivity;

   delete[] data;
  /*     */   
}

void MPU9250::initMPU9250()
{  
 // wake up device
  this->writeRegister(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  usleep(100000); // Wait for all registers to reset 

 // get stable time source
  this->writeRegister(PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  usleep(200000); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  this->writeRegister(CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  this->writeRegister(SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = this->readRegister(GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x02; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  this->writeRegister(GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  c = this->readRegister(ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  this->writeRegister(ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = this->readRegister(ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  this->writeRegister(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   this->writeRegister(INT_PIN_CFG, 0x22);    
   this->writeRegister(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   usleep(100000);
}

void MPU9250::getReady(){
	float *SelfTestRes, *Bias; 
	
	SelfTestRes=new float[6];
	this->MPU9250SelfTest(SelfTestRes);//SelfTestRes);

	Bias=new float[6];
	this->calibrateMPU9250(Bias);//(SelfTestRes);

	cout << "selftest: " << SelfTestRes[0] << "," << SelfTestRes[1] << "," << SelfTestRes[2] << "," << SelfTestRes[3] << "," << SelfTestRes[4] << "," << SelfTestRes[5] << endl;
	cout << "bias: " << Bias[0] << "," << Bias[1] << "," << Bias[2] << "," << Bias[3] << "," << Bias[4] << "," << Bias[5] << "\n"; 

	this->initMPU9250();

	this->getGres();
	this->getAres();
	this->getMres();

	delete[] SelfTestRes;
	delete[] Bias;
}

void MPU9250::getMres() {
  switch (Mscale)
  {
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void MPU9250::getGres() {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void MPU9250::getAres() {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

void MPU9250::getData(){
	unsigned char* data;
	//clock_gettime(CLOCK_MONOTONIC,&start_t);
	data=this->readRegisters(14,ACCEL_XOUT_H);	// This is way faster. almost the same speed as one read.
	this->ax=this->combineRegisters(data[0],data[1])*this->aRes;
	this->ay=this->combineRegisters(data[2],data[3])*this->aRes;
	this->az=this->combineRegisters(data[4],data[5])*this->aRes;
	this->gx=this->combineRegisters(data[8],data[9])*this->gRes;
	this->gy=this->combineRegisters(data[10],data[11])*this->gRes;
	this->gz=this->combineRegisters(data[12],data[13])*this->gRes;
	//cout << data << endl;
	//clock_gettime(CLOCK_MONOTONIC,&end_t);
	//interval=((end_t.tv_sec-start_t.tv_sec)*BILLION + (end_t.tv_nsec-start_t.tv_nsec))/1000;
	//cout << (float)accx*ACCFACTOR << "g, " << (float)accy*ACCFACTOR << "g, " << (float)accz*ACCFACTOR <<"g, " << interval << "\n";
}


void MPU9250::getAngle(){
	//Complementary filter
	float angle_acc, angle_gyro;
	angle_acc=atan2(this->ay,this->az);
	angle_gyro=this->pitch+gx*this->deltat/180.0*PI;
	this->pitch=this->alpha*angle_gyro+(1-this->alpha)*angle_acc-this->pitch_offset;
  
  //this->pitch=angle_acc-this->pitch_offset;
  //this->pitch=angle_gyro-this->pitch_offset;
}


void MPU9250::getInitAngle(){
  this->pitch=atan2(this->ay,this->az);
}














//sssasdfasdfasdf

















/*

//1111111111
void MPU9250::calculatePitchAndRoll(){
	
  float gravity_range;
  switch(ADXL345::range){
    case ADXL345::PLUSMINUS_16_G: gravity_range=32.0f; break;
    case ADXL345::PLUSMINUS_8_G: gravity_range=16.0f; break;
    case ADXL345::PLUSMINUS_4_G: gravity_range=8.0f; break;
    default: gravity_range=4.0f; break;
  }
    float resolution = 1024.0f;
    if (this->resolution==ADXL345::HIGH) resolution = 8192.0f; //13-bit resolution
    float factor = gravity_range/resolution;

    float accXg = this->accelerationX * factor;
    float accYg = this->accelerationY * factor;
    float accZg = this->accelerationZ * factor;
  float accXSquared = accXg * accXg ;
  float accYSquared = accYg * accYg ;
  float accZSquared = accZg * accZg ;
  this->pitch = 180 * atan(accXg/sqrt(accYSquared + accZSquared))/M_PI;
  this->roll = 180 * atan(accYg/sqrt(accXSquared + accZSquared))/M_PI;
}

int MPU9250::updateRegisters(){
   //update the DATA_FORMAT register
   char data_format = 0x00;  //+/- 2g with normal resolution
   //Full_resolution is the 3rd LSB
   data_format = data_format|((this->resolution)<<3);
   data_format = data_format|this->range; // 1st and 2nd LSB therefore no shift
   return this->writeRegister(DATA_FORMAT, data_format);
}

ADXL345::ADXL345(unsigned int I2CBus, unsigned int I2CAddress):
  I2CDevice(I2CBus, I2CAddress){   // this member initialisation list calls the parent constructor
  this->I2CAddress = I2CAddress;
  this->I2CBus = I2CBus;
  this->accelerationX = 0;
  this->accelerationY = 0;
  this->accelerationZ = 0;
  this->pitch = 0.0f;
  this->roll = 0.0f;
  this->registers = NULL;
  this->range = ADXL345::PLUSMINUS_16_G;
  this->resolution = ADXL345::HIGH;
  this->writeRegister(POWER_CTL, 0x08);
  this->updateRegisters();
}

int ADXL345::readSensorState(){
  this->registers = this->readRegisters(BUFFER_SIZE, 0x00);
  if(*this->registers!=0xe5){
    perror("ADXL345: Failure Condition - Sensor ID not Verified");
    return -1;
  }
  this->accelerationX = this->combineRegisters(*(registers+DATAX1), *(registers+DATAX0));
  this->accelerationY = this->combineRegisters(*(registers+DATAY1), *(registers+DATAY0));
  this->accelerationZ = this->combineRegisters(*(registers+DATAZ1), *(registers+DATAZ0));
  this->resolution = (ADXL345::RESOLUTION) (((*(registers+DATA_FORMAT))&0x08)>>3);
  this->range = (ADXL345::RANGE) ((*(registers+DATA_FORMAT))&0x03);
  this->calculatePitchAndRoll();
  return 0;
}


void ADXL345::setRange(ADXL345::RANGE range) {
  this->range = range;
  updateRegisters();
}

void ADXL345::setResolution(ADXL345::RESOLUTION resolution) {
  this->resolution = resolution;
  updateRegisters();
}

void ADXL345::displayPitchAndRoll(int iterations){
  int count = 0;
  while(count < iterations){
        cout << "Pitch:"<< this->getPitch() << " Roll:" << this->getRoll() << "     \r"<<flush;
        usleep(100000);
        this->readSensorState();
        count++;
  }
}
*/








































/*


void MPU9250::getMres() {
  switch (Mscale)
  {
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void MPU9250::getGres() {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void MPU9250::getAres() {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

















void MPU9250::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void MPU9250::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void MPU9250::readMagData(int16_t * destination)
{
  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of
  // data acquisition
  uint8_t rawData[7];
  // Wait for magnetometer data ready bit to be set
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
  {
    // Read the six raw data and ST2 registers sequentially into data array
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    // Check if magnetic sensor overflow set, if not then report data
    if(!(c & 0x08))
    {
      // Turn the MSB and LSB into a signed 16-bit value
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
      // Data stored as little Endian 
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    }
  }
}

int16_t MPU9250::readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
}

// Calculate the time the last update took for use in the quaternion filters
void MPU9250::updateTime()
{
  Now = micros();
  
  // Set integration time by time elapsed since last filter update
  deltat = ((Now - lastUpdate) / 1000000.0f);
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
}

void MPU9250::initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

void MPU9250::initMPU9250()
{  
 // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x02; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}




   


        
// Wire.h read and write protocols
void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count,
                        uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
    dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
*/
