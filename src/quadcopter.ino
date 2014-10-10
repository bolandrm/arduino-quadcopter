#define SERIAL_PORT_SPEED 115200

#include "imu.h"

IMU imu;
int counter = 0;
uint32_t loop_time;

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  imu.init();
}

void loop() {
  uint32_t loop_start_time = micros();

  while(!imu.update_sensor_values());

  if (counter == 250) {
    Serial.print(" x angle: "); Serial.print(imu.x_angle);
    Serial.print(" y angle: "); Serial.print(imu.y_angle);
    Serial.print(" loop_time (hz): "); Serial.print(1000000/loop_time);

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

  loop_time = micros() - loop_start_time;
}
