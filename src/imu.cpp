#include "imu.h"

IMU::IMU() {}

void IMU::init() {
  Fastwire::setup(800, true);

	Serial.println("Initializing mpu6050...");
  mpu6050.initialize();

	// Check connection
	Serial.println("Testing device connections...");
	Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  delay(100); // Wait for sensor to stabilize
  mpu6050.setDLPFMode(3); //Set Low Pass filter

  calibrate_gyro();

  combination_update_timer = millis();
}

void IMU::update_sensor_values() {
  if ((micros() - gyro_update_timer) > 2000) {
    update_gyro();
    gyro_update_timer = micros();
  }

  if ((millis() - accel_update_timer) > 20) {
    update_accel();
    accel_update_timer = millis();
  }

  combine();
}

void IMU::combine() {
  uint32_t timer = micros();
  float dt = (float) (timer - combination_update_timer) / 1000000.0;

  float comp_gyro = COMPLEMENTARY_RATIO;
  float comp_accel = 1.0 - comp_gyro;

  gyro_angle_x += gyro_x_rate * dt;
  gyro_angle_y += gyro_y_rate * dt;
  gyro_angle_z += gyro_z_rate * dt;

  angle_x = comp_gyro * (angle_x + gyro_x_rate * dt) + comp_accel * accel_x;
  angle_y = comp_gyro * (angle_y + gyro_y_rate * dt) + comp_accel * accel_y;

  combination_update_timer = timer;
}

void IMU::update_gyro() {
  mpu6050.getRotation(&gyro_x, &gyro_y, &gyro_z);

  gyro_x_rate = (float) (gyro_x - gyro_x_offset) / 131.0;
	gyro_y_rate = (float) -1 * (gyro_y - gyro_y_offset) / 131.0;
	gyro_z_rate = (float) (gyro_z - gyro_z_offset) / 131.0;
}

void IMU::update_accel() {
  int16_t accel_x_in, accel_y_in, accel_z_in;
  mpu6050.getAcceleration(&accel_x_in, &accel_y_in, &accel_z_in);

  accel_x = (atan2(accel_x_in, accel_z_in) + PI) * RAD_TO_DEG;
	accel_y = (atan2(accel_y_in, accel_z_in) + PI) * RAD_TO_DEG;
}

void IMU::calibrate_gyro() {
  uint16_t n = 200;

  float sX = 0.0;
  float sY = 0.0;
  float sZ = 0.0;

  for (uint16_t i = 0; i < n; i++)
  {
    mpu6050.getRotation(&gyro_x, &gyro_y, &gyro_z);
    sX += mpu6050.getRotationX();
    sY += mpu6050.getRotationY();
    sZ += mpu6050.getRotationZ();
  }

  gyro_x_offset = sX / n;
  gyro_y_offset = sY / n;
  gyro_z_offset = sZ / n;

  Serial.print("Setting offsets: x: "); Serial.print(gyro_x_offset);
  Serial.print(" y: "); Serial.print(gyro_y_offset);
  Serial.print(" z: "); Serial.print(gyro_z_offset);
  Serial.println();
}
