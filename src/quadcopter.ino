#define SERIAL_PORT_SPEED 115200

#include "imu.h"

IMU imu;

int counter = 0;

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  Fastwire::setup(800, true);
  imu.init();
}

float angles[3];
float rates[3];

void loop() {
  imu.processAngles(angles, rates);

  if (counter == 500) {
    Serial.print(" x angle: "); Serial.print(angles[0]);
    Serial.print(" y angle: "); Serial.print(angles[1]);

    // Serial.print(" x rate: "); Serial.print(imu.gyro_x_rate);
    // Serial.print(" y rate: "); Serial.print(imu.gyro_y_rate);
    // Serial.print(" z rate: "); Serial.print(imu.gyro_z_rate);
    // Serial.print(" x gyro: "); Serial.print(imu.gyro_angle_x);
    // Serial.print(" y gyro: "); Serial.print(imu.gyro_angle_y);

    // Serial.print(" x accel: "); Serial.print(imu.accel_x);
    // Serial.print(" y accel: "); Serial.print(imu.accel_y);

    // Serial.print(" x angle: "); Serial.print(imu.angle_x);
    // Serial.print(" y angle: "); Serial.print(imu.angle_y);

    // // Serial.print(" x rate: "); Serial.print(imu.gyro_x_rate);
    // // Serial.print(" y rate: "); Serial.print(imu.gyro_y_rate);
    // // Serial.print(" z rate: "); Serial.print(imu.gyro_z_rate);
    // Serial.print(" x gyro: "); Serial.print(imu.gyro_angle_x);
    // Serial.print(" y gyro: "); Serial.print(imu.gyro_angle_y);

    // Serial.print(" x accel: "); Serial.print(imu.accel_x);
    // Serial.print(" y accel: "); Serial.print(imu.accel_y);

    Serial.println();

    counter = 0;
  } else {
    counter += 1;
  }
}
