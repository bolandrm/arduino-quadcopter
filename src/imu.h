// Much of this was adapted from:
// https://github.com/RomainGoussault/quadcopter

#ifndef IMU_h
#define IMU_h

#include <Arduino.h>
#include "MPULib.h"
#include "calibrator.h"

#define GYRO_PART 0.99
#define ACC_PART (1.0 - GYRO_PART)
#define ALPHA 0.9
#define GYRO_X_OFFSET 0.0;
#define GYRO_Y_OFFSET 0.0;
#define GYRO_Z_OFFSET 0.0;
#define ACCEL_X_OFFSET 2;
#define ACCEL_Y_OFFSET 8;
#define ACCEL_Z_OFFSET 49;

class IMU {
  public:
    void init();
    bool update_sensor_values();

    float x_angle, y_angle, z_angle;
    float x_rate, y_rate, z_rate;

    int16_t acc_x_in, acc_y_in, acc_z_in;

  private:
    void setup_initial_angles();
    void update_gyro();
    void update_accel();
    void combine();

    MPULib mpu;
    Calibrator calibrator;

    float gyro_x_in, gyro_y_in, gyro_z_in;

    float acc_x_angle, acc_y_angle;
    float gyro_x_angle, gyro_y_angle, gyro_z_angle;
    float comp_angle_x, comp_angle_y;

    float gyro_x_rate;
    float gyro_y_rate;
    float gyro_z_rate;

    uint32_t gyro_update_timer, accel_update_timer, combination_update_timer;
    float gyro_x_offset, gyro_y_offset, gyro_z_offset;


    float dt;
};

#endif
