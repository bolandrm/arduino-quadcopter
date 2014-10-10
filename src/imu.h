#ifndef IMU_h
#define IMU_h

#define COMPLEMENTARY_RATIO 0.995

#include "I2Cdev.h"
#include "MPU6050.h"

class IMU {
  public:
    IMU();

    void init();
    void update_sensor_values();

    float gyro_x_rate, gyro_y_rate, gyro_z_rate;
    float gyro_angle_x, gyro_angle_y, gyro_angle_z;
    int16_t accel_x, accel_y, accel_z;
    float angle_x, angle_y;

  private:
    MPU6050 mpu6050;

    void calibrate_gyro();
    void update_gyro();
    void update_accel();
    void combine();

    uint32_t gyro_update_timer, accel_update_timer, combination_update_timer;

    int16_t gyro_x, gyro_y, gyro_z;
    int16_t gyro_x_offset, gyro_y_offset, gyro_z_offset;
};

#endif
