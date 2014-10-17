// Much of this was adapted from:
// https://github.com/RomainGoussault/quadcopter

#ifndef IMU_h
#define IMU_h

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Filter.h"

#define GYRO_PART 0.995
#define ACC_PART (1.0 - GYRO_PART)

#define ACCEL_X_OFFSET  146
#define ACCEL_Y_OFFSET  -1335
#define ACCEL_Z_OFFSET  1456
#define GYRO_X_OFFSET 8
#define GYRO_Y_OFFSET 67
#define GYRO_Z_OFFSET -47

class IMU {
  public:
    IMU();

    void init();
    bool update_sensor_values();

    float x_angle, y_angle, z_angle;
    float x_rate, y_rate, z_rate;

  private:
    void set_offsets();
    void setup_initial_angles();
    void update_gyro();
    void update_accel();
    void combine();

    MPU6050 mpu9050;
    Filter filter_x;
    Filter filter_y;
    Filter filter_z;

    int16_t acc_x_in, acc_y_in, acc_z_in;
    int16_t gyro_x_in, gyro_y_in, gyro_z_in;

    float acc_x_angle, acc_y_angle;
    float gyro_x_angle, gyro_y_angle, gyro_z_angle;
    float comp_angle_x, comp_angle_y;

    uint32_t gyro_update_timer, accel_update_timer, combination_update_timer;
    float gyro_x_offset, gyro_y_offset, gyro_z_offset;

    float gyro_x_rate;
    float gyro_y_rate;
    float gyro_z_rate;

    float dt;
};

#endif
