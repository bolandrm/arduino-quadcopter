#define SERIAL_PORT_SPEED 115200

#include "imu.h"
#include "remote_control.h"
#include "rc_interrupts.h"

IMU imu;
RemoteControl rc;

int counter = 0;
uint32_t loop_time;

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  imu.init();
  bind_rc_interrupts();
}

void loop() {
  uint32_t loop_start_time = micros();

  while(!imu.update_sensor_values());
  rc.read_values();

  if (counter == 250) {
    Serial.print(" x angle: "); Serial.print(imu.x_angle);
    Serial.print(" y angle: "); Serial.print(imu.y_angle);
    Serial.print(" throttle: "); Serial.print(rc.get(RC_THROTTLE));
    Serial.print(" roll: "); Serial.print(rc.get(RC_ROLL));
    Serial.print(" pitch: "); Serial.print(rc.get(RC_PITCH));
    Serial.print(" loop_time (hz): "); Serial.print(1000000/loop_time);
    Serial.println();

    counter = 0;
  } else {
    counter += 1;
  }

  loop_time = micros() - loop_start_time;
}
