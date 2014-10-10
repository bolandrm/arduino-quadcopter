#include "IMU.h"

IMU::IMU() {}

void IMU::init()
{
  Fastwire::setup(800, true);

	// Initialize device
	Serial.println("Initializing I2C devices...");
	mpu9050.initialize();

	// Check connection
	Serial.println("Testing device connections...");
	Serial.println(mpu9050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	delay(100); // Wait for sensor to stabilize

	mpu9050.setDLPFMode(3);  //Set Low Pass filter 

	gyro_part = 0.995;
	acc_part = 1 - gyro_part;

  setup_initial_angles();
  calibrate_gyro();
}

bool IMU::processAngles(float angles[],float rates[])
{
  mpu9050.getRotation(&gyro_x_in, &gyro_y_in, &gyro_z_in);
  mpu9050.getAcceleration(&acc_x_in, &acc_y_in, &acc_z_in);

	//Filter
	accXf = filterX.update(acc_x_in);
	accYf = filterY.update(acc_y_in);
	accZf = filterZ.update(acc_z_in);

	// Angular rates provided by gyro
	gyro_x_rate = (float) (gyro_x_in - gyro_x_offset) / 131.0;
	gyro_y_rate = (float) -1 * (gyro_y_in - gyro_y_offset) / 131.0;
	gyro_z_rate = (float) (gyro_z_in - gyro_z_offset) / 131.0;

	//Angle provided by accelerometer
	acc_y_angle = (atan2(accXf,accZf)+PI)*RAD_TO_DEG; // 
	acc_x_angle = (atan2(accYf,accZf)+PI)*RAD_TO_DEG;

	//Integration of gyro rates to get the angles
	gyro_x_angle += gyro_x_rate*(float)(micros()-timer)/1000000;
	gyro_y_angle += gyro_y_rate*(float)(micros()-timer)/1000000;
	gyro_z_angle += gyro_z_rate*(float)(micros()-timer)/1000000;

	//Angle calculation through Complementary filter
	dt = (float)(micros()-timer)/1000000.0;
	compAngleX = gyro_part*(compAngleX+(gyro_x_rate*dt))   +   acc_part*acc_x_angle;
	compAngleY = gyro_part*(compAngleY+(gyro_y_rate*dt))  +   acc_part*acc_y_angle;

	timer = micros();

	//45 deg rotation for roll and pitch (depending how your IMU is fixed to your quad)
	angles[0]=  -rac22* compAngleX + rac22*compAngleY + ROLL_OFFSET;
	angles[1]=  -rac22* compAngleX - rac22*compAngleY +2*rac22*PI*RAD_TO_DEG + PITCH_OFFSET;
	angles[2]=  gyro_z_angle;

	rates[0]=   -  rac22* gyro_x_rate + rac22*gyro_y_rate;
	rates[1]= - rac22* gyro_x_rate - rac22*gyro_y_rate;
	rates[2]=  gyro_z_rate;
}

void IMU::calibrate_gyro() {
	//Gyro Calibration: Rough guess of the gyro drift at startup
	float n = 200;

	float sX = 0.0;
	float sY = 0.0;
	float sZ = 0.0;

	for (int i = 0; i < n; i++)
	{
		sX += mpu9050.getRotationX();
		sY += mpu9050.getRotationY();
		sZ += mpu9050.getRotationZ();
	}

	gyro_x_offset = sX/n;
	gyro_y_offset = sY/n;
	gyro_z_offset = sZ/n;
}

void IMU::setup_initial_angles() {
  mpu9050.getRotation(&gyro_x_in, &gyro_y_in, &gyro_z_in);
  mpu9050.getAcceleration(&acc_x_in, &acc_y_in, &acc_z_in);

	acc_x_angle = (atan2(acc_x_in,acc_z_in)+PI)*RAD_TO_DEG;
	acc_y_angle = (atan2(acc_y_in,acc_z_in)+PI)*RAD_TO_DEG;

	gyro_x_angle = acc_x_angle;
	gyro_y_angle = acc_y_angle;
	gyro_z_angle = 0;

	compAngleX = acc_x_angle;
	compAngleY = acc_y_angle;
}
