#include "imu.h"

IMU::IMU() {}

void IMU::init() {
  mpu6050.initialize();

  delay(100); // Wait for sensor to stabilize
  mpu6050.setDLPFMode(3); //Set Low Pass filter

  calibrate_gyro();
}

void IMU::update_sensor_values() {

  if ((micros() - gyro_update_timer) > 1300) {
    update_gyro();
    gyro_update_timer = micros();
  }

  if ((millis() - accel_update_timer) > 20) {
    update_accel();
    accel_update_timer = millis();
  }

  //gyroXrate = (float) (gyroX-gyroXoffset)/131.0; //140 µsec on Arduino MEGA
	//gyroYrate = -((float) (gyroY-gyroYoffset)/131.0);
	//gyroZrate = ((float) (gyroZ-gyroZoffset)/131.0);
}

void IMU::update_gyro() {
}

void IMU::update_accel() {
}

void IMU::calibrate_gyro() {
  uint16_t n = 200;

  float sX = 0.0;
  float sY = 0.0;
  float sZ = 0.0;

  for (uint16_t i = 0; i < n; i++)
  {
    //mpu6050.getRotation(&gyroX, &gyroY, &gyroZ);
    sX += mpu6050.getRotationX();
    sY += mpu6050.getRotationY();
    sZ += mpu6050.getRotationZ();
  }

  gyro_x_offset = sX/n;
  gyro_y_offset = sY/n;
  gyro_z_offset = sZ/n;

  Serial.print("Setting offsets: x: "); Serial.print(gyro_x_offset);
  Serial.print(" y: "); Serial.print(gyro_y_offset);
  Serial.print(" z: "); Serial.print(gyro_z_offset);
  Serial.println();
}
