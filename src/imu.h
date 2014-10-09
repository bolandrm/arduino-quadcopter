#ifndef IMU_h
#define IMU_h

#include "I2Cdev.h"
#include "MPU6050.h"

class IMU {
  public:
    IMU();
    void init();
    void update_sensor_values();

  private:
    MPU6050 mpu6050;

    void calibrate_gyro();
    void update_gyro();
    void update_accel();

    uint32_t gyro_update_timer;
    uint32_t accel_update_timer;

    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    int16_t gyro_x_offset, gyro_y_offset, gyro_z_offset;
};

#endif
