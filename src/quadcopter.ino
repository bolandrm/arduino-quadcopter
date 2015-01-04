#define SERIAL_PORT_SPEED 115200

#include "imu.h"
#include "remote_control.h"
#include "rc_interrupts.h"
#include "flight_controller.h"
#include "debugger.h"

IMU imu;
RemoteControl rc;
FlightController flight_controller;
Debugger debugger;

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  imu.init();
  bind_rc_interrupts();
  flight_controller.init(&rc, &imu);

  debugger.init(&rc, &imu, &flight_controller);
}

void loop() {
  while(!imu.update_sensor_values());

  rc.read_values();
  flight_controller.process();

  debugger.print();
}
