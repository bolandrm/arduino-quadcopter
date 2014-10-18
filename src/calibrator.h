// Thanks to Luis Ródenas <luisrodenaslorda@gmail.com>
// http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/

#ifndef calibrator_h
#define calibrator_h

#include "MPU6050.h"

class Calibrator {
  public:
    Calibrator();
    void calibrate(MPU6050 *);

  private:
    void mean_sensors();

    MPU6050 *mpu9050;

    int buffersize;
    int acel_deadzone;
    int giro_deadzone;

    int16_t ax, ay, az, gx, gy, gz;
    int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
    int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
};

#endif
