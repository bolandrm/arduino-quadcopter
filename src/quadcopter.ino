#define SERIAL_PORT_SPEED 115200

#include "config.h"
#include "imu.h"
#include "remote_control.h"
#include "rc_interrupts.h"
#include "flight_controller.h"

IMU imu;
RemoteControl rc;
FlightController flight_controller;

bool debug = false;
uint32_t last_debug_time;
uint32_t loop_time;
uint32_t loop_start_time;
void debug_output();
void set_debug();

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  imu.init();
  bind_rc_interrupts();
  flight_controller.init(&rc, &imu);
  last_debug_time = micros();
}

void loop() {
  while(!imu.update_sensor_values());
  rc.read_values();
  set_debug();
  flight_controller.process(debug);

  if (debug && !CHART_DEBUG) { debug_output(); }
  debug = false;
}

void set_debug() {
  if (millis() - last_debug_time > 100) {
    debug = true;
    last_debug_time = millis();
  }
}

void debug_output() {
  loop_time = micros() - loop_start_time;
  loop_start_time = micros();

  Serial.print("loop_time (hz): "); Serial.print(1000000/loop_time);
  // Serial.print("x_gyro: "); Serial.print(imu.x_rate);
  // Serial.print(" \t y_gyro: "); Serial.print(imu.y_rate);
  // Serial.print(" \t x_ang: "); Serial.print(imu.x_angle);
  // Serial.print(" \t y_ang "); Serial.print(imu.y_angle);
  // Serial.println();

  // Serial.print("x_ang_raw: "); Serial.print(imu.acc_x_in);
  // Serial.print(" \t y_ang_raw: "); Serial.print(imu.acc_y_in);
  // Serial.print(" \t z_ang_raw "); Serial.print(imu.acc_z_in);
  // Serial.println();

  // Serial.print("thrttl: "); Serial.print(rc.get(RC_THROTTLE));
  // Serial.print("\t x_tar: "); Serial.print(rc.get(RC_ROLL));
  // Serial.print("\t y_tar: "); Serial.print(rc.get(RC_PITCH));
  // Serial.print("\t z_tar: "); Serial.print(rc.get(RC_YAW));
  // Serial.print("\t pot_a: "); Serial.print(rc.get(RC_POT_A));
  // Serial.print("\t pot_b: "); Serial.print(rc.get(RC_POT_B));

  Serial.println();
  Serial.println();
}
