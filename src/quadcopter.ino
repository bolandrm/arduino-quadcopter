#define SERIAL_PORT_SPEED 115200

#include "imu.h"
#include "remote_control.h"
#include "rc_interrupts.h"
#include "flight_controller.h"

IMU imu;
RemoteControl rc;
FlightController flight_controller;

int counter = 0;
uint32_t loop_time;
uint32_t loop_start_time;
void debug_output();

uint32_t gyro_update_timer = micros();

float buffer[3]; //Gyro buffer

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  imu.init();
  //bind_rc_interrupts();
  //flight_controller.init(&rc, &imu);
}

void loop() {
  while(!imu.update_sensor_values());
  rc.read_values();
  //flight_controller.process(counter == 240);

  debug_output();
}

void debug_output() {
  loop_time = micros() - loop_start_time;
  loop_start_time = micros();

  if (counter == 250) {
    Serial.print("loop_time (hz): "); Serial.print(1000000/loop_time);
    Serial.print("x_gyro: "); Serial.print(imu.x_rate);
    Serial.print(" \t y_gyro: "); Serial.print(imu.y_rate);
    Serial.print(" \t x_ang: "); Serial.print(imu.x_angle);
    Serial.print(" \t y_ang "); Serial.print(imu.y_angle);
    Serial.println();

    Serial.print("x_ang_raw: "); Serial.print(imu.acc_x_in);
    Serial.print(" \t y_ang_raw: "); Serial.print(imu.acc_y_in);
    Serial.print(" \t z_ang_raw "); Serial.print(imu.acc_z_in);

    Serial.println();
    Serial.println();

    counter = 0;
  } else {
    counter += 1;
  }
}
