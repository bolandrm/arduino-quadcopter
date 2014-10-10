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

	alpha_gyro = 0.995;
	c = (1-alpha_gyro)*1;

  mpu9050.getRotation(&gyro_x_in, &gyro_y_in, &gyro_z_in);
  mpu9050.getAcceleration(&acc_x_in, &acc_y_in, &acc_z_in);

	accXangle = (atan2(acc_x_in,acc_z_in)+PI)*RAD_TO_DEG;
	accYangle = (atan2(acc_y_in,acc_z_in)+PI)*RAD_TO_DEG;

	gyroXangle = accXangle;
	gyroYangle = accYangle;
	gyroZangle = 0;

	compAngleX = accXangle;
	compAngleY = accYangle;

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
	gyroXrate = (float) (gyro_x_in-gyro_x_offset)/131.0; //140 µsec on Arduino MEGA
	gyroYrate = -((float) (gyro_y_in-gyro_y_offset)/131.0);
	gyroZrate = ((float) (gyro_z_in-gyro_z_offset)/131.0);

	//Angle provided by accelerometer
	//Time taken: 400 µsec on Arduino MEGA
	accYangle = (atan2(accXf,accZf)+PI)*RAD_TO_DEG; // 
	accXangle = (atan2(accYf,accZf)+PI)*RAD_TO_DEG;

	//Integration of gyro rates to get the angles
	gyroXangle += gyroXrate*(float)(micros()-timer)/1000000;
	gyroYangle += gyroYrate*(float)(micros()-timer)/1000000;
	gyroZangle += gyroZrate*(float)(micros()-timer)/1000000;

	//Angle calculation through Complementary filter
	//Time taken: 200 µsec on Arduino MEGA
	dt = (float)(micros()-timer)/1000000.0;
	compAngleX = alpha_gyro*(compAngleX+(gyroXrate*dt))   +   c*accXangle;
	compAngleY = alpha_gyro*(compAngleY+(gyroYrate*dt))  +   c*accYangle;

	timer = micros();

	//45 deg rotation for roll and pitch (depending how your IMU is fixed to your quad)
	angles[0]=  -rac22* compAngleX + rac22*compAngleY + ROLL_OFFSET;
	angles[1]=  -rac22* compAngleX - rac22*compAngleY +2*rac22*PI*RAD_TO_DEG + PITCH_OFFSET;
	angles[2]=  gyroZangle;

	rates[0]=   -  rac22* gyroXrate + rac22*gyroYrate;
	rates[1]= - rac22* gyroXrate - rac22*gyroYrate;
	rates[2]=  gyroZrate;
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
