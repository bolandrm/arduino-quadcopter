// Thanks to Luis Ródenas <luisrodenaslorda@gmail.com>
// http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/

#include "calibrator.h"

Calibrator::Calibrator() {
  // Amount of readings used to average, make it higher to get more precision
  // but sketch will be slower  (default:1000)
  buffersize=300;

  // Acelerometer error allowed, make it lower to get more precision,
  // but sketch may not converge  (default:8)
  acel_deadzone=8;

  // Gyro error allowed, make it lower to get more precision,
  // but sketch may not converge  (default:1)
  giro_deadzone=1;
}

// void Calibrator::read_calibration(MPU6050 *mpu) {
//   ax_offset = eeprom_read_word(0);
//   ay_offset = eeprom_read_word((uint16_t *) (16 * 1 - 1));
//   az_offset = eeprom_read_word((uint16_t *) (16 * 2 - 1));
//   gx_offset = eeprom_read_word((uint16_t *) (16 * 3 - 1));
//   gy_offset = eeprom_read_word((uint16_t *) (16 * 4 - 1));
//   gz_offset = eeprom_read_word((uint16_t *) (16 * 5 - 1));
// 
//   Serial.print("READING CALIBRATION: ");
//   Serial.print(ax_offset); Serial.print("\t");
//   Serial.print(ay_offset); Serial.print("\t");
//   Serial.print(az_offset); Serial.print("\t");
//   Serial.print(gx_offset); Serial.print("\t");
//   Serial.print(gy_offset); Serial.print("\t");
//   Serial.println(gz_offset);
// 
//   mpu9050->setXAccelOffset(ax_offset);
//   mpu9050->setYAccelOffset(ay_offset);
//   mpu9050->setZAccelOffset(az_offset);
//   mpu9050->setXGyroOffset(gx_offset);
//   mpu9050->setYGyroOffset(gy_offset);
//   mpu9050->setZGyroOffset(gz_offset);
// }

void Calibrator::calibrate(MPULib *mpu_in) {
  mpu = mpu_in;

  //mpu9050->setXAccelOffset(0);
  //mpu9050->setYAccelOffset(0);
  //mpu9050->setZAccelOffset(0);
  // gyro->setXGyroOffset(0);
  // gyro->setYGyroOffset(0);
  // gyro->setZGyroOffset(0);
  gx_offset = 0;
  gy_offset = 0;
  gz_offset = 0;

  mean_sensors();

  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=-mean_az/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;

  while (1){
    int ready=0;

    // mpu9050->setXAccelOffset(ax_offset);
    // mpu9050->setYAccelOffset(ay_offset);
    // mpu9050->setZAccelOffset(az_offset);
    // gyro->setXGyroOffset(gx_offset);
    // gyro->setYGyroOffset(gy_offset);
    // gyro->setZGyroOffset(gz_offset);

    mean_sensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset-mean_az/acel_deadzone;

    // if (abs(mean_gx)<=giro_deadzone) ready++;
    // else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    // if (abs(mean_gy)<=giro_deadzone) ready++;
    // else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    // if (abs(mean_gz)<=giro_deadzone) ready++;
    // else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    // if (ready==6) break;
    if (ready==3) break;
  }

  // Hardcode the accel offsets for now...
  // mpu9050->setXAccelOffset(143);
  // mpu9050->setYAccelOffset(-1317);
  // mpu9050->setZAccelOffset(1457);
  // 143        -1317   1457    8       66      -47

  Serial.print("WRITING CALIBRATION: ");
  Serial.print(ax_offset); Serial.print("\t");
  Serial.print(ay_offset); Serial.print("\t");
  Serial.print(az_offset); Serial.print("\t");
  Serial.print(gx_offset); Serial.print("\t");
  Serial.print(gy_offset); Serial.print("\t");
  Serial.println(gz_offset);

  eeprom_write_word((uint16_t *) 0, ax_offset);
  eeprom_write_word((uint16_t *) (16 * 1 - 1), ay_offset);
  eeprom_write_word((uint16_t *) (16 * 2 - 1), az_offset);
  eeprom_write_word((uint16_t *) (16 * 3 - 1), gx_offset);
  eeprom_write_word((uint16_t *) (16 * 4 - 1), gy_offset);
  eeprom_write_word((uint16_t *) (16 * 5 - 1), gz_offset);
}

void Calibrator::mean_sensors() {
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    //mpu9050->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mpu->getGyroData(&gx, &gy, &gz);

    gx = gx + gx_offset;
    gy = gy + gy_offset;
    gz = gz + gz_offset;

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
}
