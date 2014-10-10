#ifndef IMU_h
#define IMU_h

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Filter.h"

#define rac22 0.7071
#define ROLL_OFFSET 0
#define PITCH_OFFSET 0
#define YAW_OFFSET 0

class IMU {
  public:
    IMU();

    void init();
    bool processAngles(float angles[],float rates[] );

  private:
    void calibrate_gyro();

    int16_t acc_x_in, acc_y_in, acc_z_in;
    int16_t gyro_x_in, gyro_y_in, gyro_z_in;

    float accXangle, accYangle, accZangle; // Angle calculate using the accelerometer
    float gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro
    float compAngleX, compAngleY;

    MPU6050 mpu9050;
    uint32_t timer;
    float gyro_x_offset, gyro_y_offset, gyro_z_offset;

    float gyroXrate ;
    float gyroYrate ;
    float gyroZrate;

    float accXf;
    float accYf;
    float accZf;

    Filter filterX;
    Filter filterY;
    Filter filterZ;

    float alpha_gyro;
    float c;
    float dt;
    char StrAnglesvib[7];
};

#endif
