#ifndef IMU_h
#define IMU_h

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Filter.h"

#define rac22 0.7071
#define COMPLEMENTARY_RATIO 0.995
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
    void setup_initial_angles();
    void update_gyro();
    void update_accel();
    void combine();

    MPU6050 mpu9050;
    Filter filterX;
    Filter filterY;
    Filter filterZ;

    float gyro_part;
    float acc_part;

    int16_t acc_x_in, acc_y_in, acc_z_in;
    int16_t gyro_x_in, gyro_y_in, gyro_z_in;

    float acc_x_angle, acc_y_angle;
    float gyro_x_angle, gyro_y_angle, gyro_z_angle;
    float compAngleX, compAngleY;

    uint32_t timer;
    float gyro_x_offset, gyro_y_offset, gyro_z_offset;

    float gyro_x_rate;
    float gyro_y_rate;
    float gyro_z_rate;

    float accXf;
    float accYf;
    float accZf;

    float dt;
};

#endif
