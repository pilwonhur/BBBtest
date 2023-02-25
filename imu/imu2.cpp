#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "I2Cdev.h"
#include "MPU9250.h"

#define I2C_BUS 2 // I2C bus number
#define MPU9250_ADDRESS 0x68 // MPU9250 I2C address
#define AK8963_ADDRESS 0x0C // AK8963 I2C address

int main() {
    // Initialize the I2C bus and set up the MPU9250 device
    int file;
    file = open("/dev/i2c-2", O_RDWR);
    if (file < 0) {
        std::cout << "Error opening I2C bus" << std::endl;
        return 1;
    }
    if (ioctl(file, I2C_SLAVE, MPU9250_ADDRESS) < 0) {
        std::cout << "Error setting MPU9250 address" << std::endl;
        return 1;
    }
    MPU9250 imu;
    imu.initialize();

    // Set up the AK8963 device through the MPU9250 passthrough mode
    write_byte(file, MPU9250_ADDRESS, 0x37, 0x02);
    usleep(10000);
    write_byte(file, AK8963_ADDRESS, 0x0A, 0x16);

    // Read and print the sensor data
    while (true) {
        imu.getMotion6();
        std::cout << "Acceleration (g): [" << imu.ax << ", " << imu.ay << ", " << imu.az << "]" << std::endl;
        std::cout << "Rotation (dps): [" << imu.gx << ", " << imu.gy << ", " << imu.gz << "]" << std::endl;
        uint8_t buffer[6];
        read_bytes(file, AK8963_ADDRESS, 0x03, 6, buffer);
        int16_t mx = ((int16_t)buffer[1] << 8) | buffer[0];
        int16_t my = ((int16_t)buffer[3] << 8) | buffer[2];
        int16_t mz = ((int16_t)buffer[5] << 8) | buffer[4];
        std::cout << "Magnetometer (uT): [" << mx << ", " << my << ", " << mz << "]" << std::endl;
        usleep(500000);
    }

    close(file);
    return 0;
}

