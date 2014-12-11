// Much of this was adapted from:
// https://github.com/RomainGoussault/quadcopter

#include "IMU.h"
#include "Wire.h"

void IMU::init() {
  Wire.begin();
  TWBR = 10;  // setup i2c to run at 444 kHz

  Serial.println("Initializing MPU...");
  mpu.init();

  delay(100); // Wait for sensor to stabilize

  //calibrator.read_calibration(&mpu9050);
  //calibrator.calibrate(&mpu);
  setup_initial_angles();

  delay(100); // Wait for sensor to stabilize
}

bool IMU::update_sensor_values() {
  bool updated = false;

  if ((millis() - accel_update_timer) > 20) {    // 50 hz
    update_accel();
    accel_update_timer = millis();
    updated = true;
  }

  if ((micros() - gyro_update_timer) > 1300) {   // ~800 Hz
    update_gyro();
    gyro_update_timer = micros();
    updated = true;
  }

  if (updated) {
    combine();

    x_angle = comp_angle_x - 180;
    y_angle = comp_angle_y - 180;
    z_angle = gyro_z_angle;
  }

  return updated;
}

void IMU::update_gyro() {
  mpu.getGyroData(&gyro_x_in, &gyro_y_in, &gyro_z_in);

  gyro_x_rate = gyro_x_in + GYRO_X_OFFSET;
  gyro_y_rate = gyro_y_in + GYRO_Y_OFFSET;
  gyro_z_rate = gyro_z_in + GYRO_Z_OFFSET;

  x_rate = GYRO_ALPHA * gyro_x_rate + (1-GYRO_ALPHA) * x_rate;
  y_rate = GYRO_ALPHA * gyro_y_rate + (1-GYRO_ALPHA) * y_rate;
  z_rate = gyro_z_rate;

  //Integration of gyro rates to get the angles
  gyro_x_angle += x_rate * (float)(micros() - gyro_update_timer) / 1000000;
  gyro_y_angle += y_rate * (float)(micros() - gyro_update_timer) / 1000000;
  gyro_z_angle += z_rate * (float)(micros() - gyro_update_timer) / 1000000;
}

void IMU::update_accel() {
  mpu.getAxlData(&acc_x_in, &acc_y_in, &acc_z_in);

  acc_x_in = acc_x_in + ACCEL_X_OFFSET;
  acc_y_in = acc_y_in + ACCEL_Y_OFFSET;
  acc_z_in = acc_z_in + ACCEL_Z_OFFSET;

  float acc_x_filtered = ACC_ALPHA * acc_x_filter.in(acc_x_in) + (1-ACC_ALPHA) * acc_x_filtered;
  float acc_y_filtered = ACC_ALPHA * acc_y_filter.in(acc_y_in) + (1-ACC_ALPHA) * acc_y_filtered;
  float acc_z_filtered = ACC_ALPHA * acc_z_filter.in(acc_z_in) + (1-ACC_ALPHA) * acc_z_filtered;

  acc_x_angle = (atan2(acc_x_filtered, acc_z_filtered) + PI) * RAD_TO_DEG;
  acc_y_angle = (atan2(acc_y_filtered, acc_z_filtered) + PI) * RAD_TO_DEG;
}

void IMU::combine() {
  //Angle calculation through Complementary filter
  dt = (float)(micros()-combination_update_timer)/1000000.0;
  comp_angle_x = GYRO_PART * (comp_angle_x + (x_rate * dt)) + ACC_PART * acc_x_angle;
  comp_angle_y = GYRO_PART * (comp_angle_y + (y_rate * dt)) + ACC_PART * acc_y_angle;

  combination_update_timer = micros();
}

void IMU::setup_initial_angles() {
  mpu.getGyroData(&gyro_x_in, &gyro_y_in, &gyro_z_in);
  mpu.getAxlData(&acc_x_in, &acc_y_in, &acc_z_in);

  acc_x_angle = (atan2(acc_x_in, acc_z_in) + PI) * RAD_TO_DEG;
  acc_y_angle = (atan2(acc_y_in, acc_z_in) + PI) * RAD_TO_DEG;

  gyro_x_angle = acc_x_angle;
  gyro_y_angle = acc_y_angle;
  gyro_z_angle = 0;

  gyro_x_rate = gyro_x_in;
  gyro_y_rate = gyro_y_in;

  x_rate = gyro_x_rate;
  y_rate = gyro_y_rate;

  comp_angle_x = acc_x_angle;
  comp_angle_y = acc_y_angle;
}

IMU::IMU() :
  acc_x_filter(7, 0.0),
  acc_y_filter(7, 0.0),
  acc_z_filter(7, 0.0)
{}
