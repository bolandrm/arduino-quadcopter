// Much of this was adapted from:
// https://github.com/RomainGoussault/quadcopter

#include "IMU.h"

IMU::IMU() {}

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

  set_offsets();

  setup_initial_angles();
}

bool IMU::update_sensor_values() {
  bool updated = false;

  if ((millis() - accel_update_timer) > 20) {
    update_accel();
    accel_update_timer = millis();
    updated = true;
  }

  if ((micros() - gyro_update_timer) > 2000) {
    update_gyro();
    gyro_update_timer = micros();
    updated = true;
  }

  if (updated) {
    combine();

    x_angle = comp_angle_x - 180;
    y_angle = -1 * (comp_angle_y - 180);
    z_angle = gyro_z_angle;

    x_rate = gyro_x_rate;
    y_rate = gyro_y_rate;
    z_rate = gyro_z_rate;
  }

  return updated;
}

void IMU::update_gyro() {
  mpu9050.getRotation(&gyro_x_in, &gyro_y_in, &gyro_z_in);

  // Angular rates provided by gyro (131 is number from datasheet, dunno)
  gyro_x_rate = (float) gyro_x_in / 131.0;
  gyro_y_rate = (float) -1 * gyro_y_in / 131.0;
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

  comp_angle_x = acc_x_angle;
  comp_angle_y = acc_y_angle;
}

void IMU::set_offsets() {
  int16_t ax, ay, az,gx, gy, gz;
  int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
  int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  // mpu9050.setXAccelOffset(ACCEL_X_OFFSET);
  // mpu9050.setYAccelOffset(ACCEL_Y_OFFSET);
  // mpu9050.setZAccelOffset(ACCEL_Z_OFFSET);
  // mpu9050.setXGyroOffset(GYRO_X_OFFSET);
  // mpu9050.setYGyroOffset(GYRO_Y_OFFSET);
  // mpu9050.setZGyroOffset(GYRO_Z_OFFSET);
}
