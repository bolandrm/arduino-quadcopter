#include "debugger.h"

void Debugger::init(RemoteControl *_rc, IMU *_imu, FlightController *_fc) {
  rc = _rc;
  imu = _imu;
  fc = _fc;

  last_debug_time = micros();
}

void Debugger::print() {
  if (millis() - last_debug_time > DEBUG_RATE_MILLIS) {

    if (CHART_DEBUG) {
      chart_debug();
    } else {
      print_debug();
    }

    last_debug_time = micros();
  }
}

void Debugger::chart_debug() {
  if (fc->mode == RATE) {
    Serial.print(rc->get(RC_PITCH));
    Serial.print(" ");
    Serial.print(imu->y_rate);
    Serial.print(" ");
    Serial.print(fc->pitch_rate_pid.GetKp());
    Serial.print(" ");
    Serial.print(fc->pitch_rate_pid.GetKi());
    Serial.print(" ");
    Serial.print(fc->pitch_rate_pid.GetKd());
    Serial.print(" ");
    Serial.print(imu->x_angle);
    Serial.print("\r");
  } else {
    Serial.print(rc->get(RC_PITCH));
    Serial.print(" ");
    Serial.print(imu->y_angle);
    Serial.print(" ");
    Serial.print(fc->pitch_angle_pid.GetKp());
    Serial.print(" ");
    Serial.print(fc->pitch_angle_pid.GetKi());
    Serial.print(" ");
    Serial.print(fc->pitch_angle_pid.GetKd());
    Serial.print(" ");
    Serial.print(imu->x_angle);
    Serial.print("\r");
  }
}

void Debugger::print_debug() {
  Serial.print("x_gyro: "); Serial.print(imu->x_rate);
  Serial.print(" \t y_gyro: "); Serial.print(imu->y_rate);
  Serial.print(" \t z_gyro: "); Serial.print(imu->z_rate);
  Serial.print(" \t x_ang: "); Serial.print(imu->x_angle);
  Serial.print(" \t y_ang "); Serial.print(imu->y_angle);
  Serial.print(" \t x_ang_raw: "); Serial.print(imu->acc_x_in);
  Serial.print(" \t y_ang_raw: "); Serial.print(imu->acc_y_in);
  Serial.print(" \t z_ang_raw "); Serial.print(imu->acc_z_in);
  Serial.print(" \t z_gyro_raw "); Serial.print(imu->gyro_z_rate);
  Serial.println();

  Serial.print("thrttl: "); Serial.print(rc->get(RC_THROTTLE));
  Serial.print("\t x_tar: "); Serial.print(rc->get(RC_ROLL));
  Serial.print("\t y_tar: "); Serial.print(rc->get(RC_PITCH));
  Serial.print("\t z_tar: "); Serial.print(rc->get(RC_YAW));
  Serial.print("\t pot_a: "); Serial.print(rc->get(RC_POT_A));
  Serial.print("\t pot_b: "); Serial.print(rc->get(RC_POT_B));
  Serial.println();

  Serial.print("M1_out: "); Serial.print(fc->motors.outputs[M1]);
  Serial.print("\t M2_out: "); Serial.print(fc->motors.outputs[M2]);
  Serial.print("\t M3_out: "); Serial.print(fc->motors.outputs[M3]);
  Serial.print("\t M4_out: "); Serial.print(fc->motors.outputs[M4]);
  Serial.println();

  Serial.print("roll_rate_pid: "); Serial.print(fc->pid_outputs[PID_ROLL_RATE]);
  Serial.print("\t pitch_rate_pid: "); Serial.print(fc->pid_outputs[PID_PITCH_RATE]);
  Serial.print("\t yaw_rate_pid: "); Serial.print(fc->pid_outputs[PID_YAW_RATE]);
  Serial.print("\t <kp: "); Serial.print(fc->pitch_rate_pid.GetKp(), 5);
  Serial.print("\t <ki: "); Serial.print(fc->pitch_rate_pid.GetKi(), 5);
  Serial.print("\t <kd: "); Serial.print(fc->pitch_rate_pid.GetKd(), 5);
  Serial.println();

  Serial.print("roll_angle_pid: "); Serial.print(fc->pid_outputs[PID_ROLL_ANGLE]);
  Serial.print("\t pitch_angle_pid: "); Serial.print(fc->pid_outputs[PID_PITCH_ANGLE]);
  Serial.print("\t <kp: "); Serial.print(fc->pitch_angle_pid.GetKp(), 5);
  Serial.print("\t <ki: "); Serial.print(fc->pitch_angle_pid.GetKi(), 5);
  Serial.print("\t <kd: "); Serial.print(fc->pitch_angle_pid.GetKd(), 5);
  Serial.println();

  if (fc->emergency_stopped) Serial.println("\t EMERGENCY STOPPED!");
}
