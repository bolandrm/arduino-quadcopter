// Much of this was adapted from:
// https://github.com/RomainGoussault/quadcopter

#include "IMU.h"

void IMU::init() {
  Fastwire::setup(800, true);

  // Initialize device
  Serial.println("Initializing I2C devices...");
  mpu9050.initialize();

  // Check connection
  Serial.println("Testing device connections...");
  Serial.println(mpu9050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  delay(100); // Wait for sensor to stabilize
  mpu9050.setDLPFMode(3);  //Set Low Pass filter 

  //calibrator.read_calibration(&mpu9050);
  calibrator.calibrate(&mpu9050);
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

  if ((micros() - gyro_update_timer) > 2500) {   // 400 Hz
    update_gyro();
    gyro_update_timer = micros();
    updated = true;
  }

  if (updated) {
    combine();

    x_angle = -1 * (comp_angle_x - 180);
    y_angle = comp_angle_y - 180;
    z_angle = gyro_z_angle;

    x_rate = ALPHA * gyro_x_rate + (1-ALPHA) * x_rate;
    y_rate = ALPHA * gyro_y_rate + (1-ALPHA) * y_rate;
    z_rate = gyro_z_rate;
  }

  return updated;
}

void IMU::update_gyro() {
  mpu9050.getRotation(&gyro_x_in, &gyro_y_in, &gyro_z_in);

  // Angular rates provided by gyro (131 is number from datasheet, dunno)
  gyro_x_rate = (float) gyro_x_in / 131.0;
  gyro_y_rate = (float) gyro_y_in / 131.0;
  gyro_z_rate = (float) gyro_z_in / 131.0;

  //Integration of gyro rates to get the angles
  gyro_x_angle += gyro_x_rate * (float)(micros() - gyro_update_timer) / 1000000;
  gyro_y_angle += gyro_y_rate * (float)(micros() - gyro_update_timer) / 1000000;
  gyro_z_angle += gyro_z_rate * (float)(micros() - gyro_update_timer) / 1000000;
}

void IMU::update_accel() {
  mpu9050.getAcceleration(&acc_x_in, &acc_y_in, &acc_z_in);

  float acc_x_filtered = filter_x.update(acc_x_in);
  float acc_y_filtered = filter_y.update(acc_y_in);
  float acc_z_filtered = filter_z.update(acc_z_in);

  acc_y_angle = (atan2(acc_x_filtered, acc_z_filtered) + PI) * RAD_TO_DEG;
  acc_x_angle = (atan2(acc_y_filtered, acc_z_filtered) + PI) * RAD_TO_DEG;
}

void IMU::combine() {
  //Angle calculation through Complementary filter
  dt = (float)(micros()-combination_update_timer)/1000000.0;
  comp_angle_x = GYRO_PART * (comp_angle_x + (gyro_x_rate * dt)) + ACC_PART * acc_x_angle;
  comp_angle_y = GYRO_PART * (comp_angle_y + (gyro_y_rate * dt)) + ACC_PART * acc_y_angle;

  combination_update_timer = micros();
}

void IMU::setup_initial_angles() {
  mpu9050.getRotation(&gyro_x_in, &gyro_y_in, &gyro_z_in);
  mpu9050.getAcceleration(&acc_x_in, &acc_y_in, &acc_z_in);

  acc_x_angle = (atan2(acc_x_in, acc_z_in) + PI) * RAD_TO_DEG;
  acc_y_angle = (atan2(acc_y_in, acc_z_in) + PI) * RAD_TO_DEG;

  gyro_x_angle = acc_x_angle;
  gyro_y_angle = acc_y_angle;
  gyro_z_angle = 0;

  // Angular rates provided by gyro (131 is number from datasheet, dunno)
  gyro_x_rate = (float) gyro_x_in / 131.0;
  gyro_y_rate = (float) gyro_y_in / 131.0;
  x_rate = gyro_x_rate;
  y_rate = gyro_y_rate;

  comp_angle_x = acc_x_angle;
  comp_angle_y = acc_y_angle;
}

void IMU::reset() {
  // This does nothing ..
  Serial.println("RESETING");
  mpu9050.resetSensors();
}
