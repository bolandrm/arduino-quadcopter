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
  mpu9050.setXAccelOffset(0);
  mpu9050.setYAccelOffset(0);
  mpu9050.setZAccelOffset(0);
  mpu9050.setXGyroOffset(0);
  mpu9050.setYGyroOffset(0);
  mpu9050.setZGyroOffset(0);

  calibration();
}

void IMU::calibration(){
  int buffersize=300;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
  int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
  int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
  int16_t ax, ay, az,gx, gy, gz;
  int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
  int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

  //    meansensors();
    long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  
    while (i<(buffersize+101)){
      // read raw accel/gyro measurements from device
      mpu9050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
        buff_ax=buff_ax+ax;
        buff_ay=buff_ay+ay;
        buff_az=buff_az+az;
        buff_gx=buff_gx+gx;
        buff_gy=buff_gy+gy;
        buff_gz=buff_gz+gz;
      }
      if (i==(buffersize+100)){
        mean_ax=buff_ax/buffersize;
        mean_ay=buff_ay/buffersize;
        mean_az=buff_az/buffersize;
        mean_gx=buff_gx/buffersize;
        mean_gy=buff_gy/buffersize;
        mean_gz=buff_gz/buffersize;
      }
      i++;
      delay(2); //Needed so we don't get repeated measures
    }
  //    meansensors();


  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  
  while (1){
    int ready=0;
    mpu9050.setXAccelOffset(ax_offset);
    mpu9050.setYAccelOffset(ay_offset);
    mpu9050.setZAccelOffset(az_offset);

    mpu9050.setXGyroOffset(gx_offset);
    mpu9050.setYGyroOffset(gy_offset);
    mpu9050.setZGyroOffset(gz_offset);

  //    meansensors();
    long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  
    while (i<(buffersize+101)){
      // read raw accel/gyro measurements from device
      mpu9050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
        buff_ax=buff_ax+ax;
        buff_ay=buff_ay+ay;
        buff_az=buff_az+az;
        buff_gx=buff_gx+gx;
        buff_gy=buff_gy+gy;
        buff_gz=buff_gz+gz;
      }
      if (i==(buffersize+100)){
        mean_ax=buff_ax/buffersize;
        mean_ay=buff_ay/buffersize;
        mean_az=buff_az/buffersize;
        mean_gx=buff_gx/buffersize;
        mean_gy=buff_gy/buffersize;
        mean_gz=buff_gz/buffersize;
      }
      i++;
      delay(2); //Needed so we don't get repeated measures
    }
  //    meansensors();
  
  
    Serial.println("...");

    Serial.print("readings:\t");
    Serial.print(mean_ax); Serial.print("\t");
    Serial.print(mean_ay); Serial.print("\t");
    Serial.print(mean_az); Serial.print("\t");
    Serial.print(mean_gx); Serial.print("\t");
    Serial.print(mean_gy); Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("offsets:\t");
    Serial.print(ax_offset); Serial.print("\t");
    Serial.print(ay_offset); Serial.print("\t");
    Serial.print(az_offset); Serial.print("\t");
    Serial.print(gx_offset); Serial.print("\t");
    Serial.print(gy_offset); Serial.print("\t");
    Serial.println(gz_offset);

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}
