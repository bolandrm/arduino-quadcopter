#include "flight_controller.h"

void FlightController::init(RemoteControl *_rc, IMU *_imu) {
  rc = _rc;
  imu = _imu;

  safety_mode = UNARMED;
  emergency_stopped = false;
  gyro_freeze_counter = 0;
  logging = false;

  for (int i = 0; i < NUM_PIDS; i++) {
    pid_inputs[i] = 0.0;
    pid_outputs[i] = 0.0;
    pid_setpoints[i] = 0.0;
  }

  roll_angle_pid.SetMode(AUTOMATIC);
  roll_angle_pid.SetSampleTime(3);
  pitch_angle_pid.SetMode(AUTOMATIC);
  pitch_angle_pid.SetSampleTime(3);

  motors.init();
}

void FlightController::process(bool debug) {
  set_safety_mode();

  set_pid_output_limits();
  adjust_pid_tuning();
  compute_pids();

  if (safety_mode == ARMED) {
    compute_motor_outputs();
    adjust_for_bounds();
  } else {
    motors.command_all_off();
  }

  safety_check();
  motors.command();

  if (debug) { debug_output(); }
}

void FlightController::adjust_for_bounds() {
  int16_t motor_fix = 0;
  uint16_t motor_min = motors.outputs[0];
  uint16_t motor_max = motors.outputs[0];

  for(int i = 1; i < NUM_MOTORS; i++) {
    if (motors.outputs[i] < motor_min) motor_min = motors.outputs[i];
    if (motors.outputs[i] > motor_max) motor_max = motors.outputs[i];
  }

  if (motor_min < MOTOR_MIN) {
    motor_fix = MOTOR_MIN - motor_min;
  } else if (motor_max > MOTOR_MAX) {
    motor_fix = MOTOR_MAX - motor_max;
  }

  for(int i = 1; i < NUM_MOTORS; i++) {
    motors.outputs[i] += motor_fix;
  }
}

void FlightController::compute_motor_outputs() {
  double m1_r_out = rc->get(RC_THROTTLE) - pid_outputs[PID_ROLL_ANGLE];
  double m2_l_out = rc->get(RC_THROTTLE) + pid_outputs[PID_ROLL_ANGLE];
  double m3_f_out = rc->get(RC_THROTTLE) - pid_outputs[PID_PITCH_ANGLE];
  double m4_b_out = rc->get(RC_THROTTLE) + pid_outputs[PID_PITCH_ANGLE];

  motors.outputs[M1] = (int16_t)(m1_r_out + 0.5);
  motors.outputs[M2] = (int16_t)(m2_l_out + 0.5);
  motors.outputs[M3] = (int16_t)(m3_f_out + 0.5);
  motors.outputs[M4] = (int16_t)(m4_b_out + 0.5);
}

void FlightController::reset_pids() {
  // this will reset the I term to avoid windup
  roll_angle_pid.SetOutputLimits(0.0, 0.01);
  pitch_angle_pid.SetOutputLimits(0.0, 0.01);
}

void FlightController::adjust_pid_tuning() {
  if (Serial.available() <= 0) return;
  byte incomingByte = Serial.read();

  double kp, kd, ki;

  kp = roll_angle_pid.GetKp();
  ki = roll_angle_pid.GetKi();
  kd = roll_angle_pid.GetKd();

  if (incomingByte == 'a') {
    if (kp <= 0.05) kp = 0;
    else kp -= 0.05;
  } else if (incomingByte == 's') {
    if (ki <= 0.05) ki = 0;
    else ki -= 0.05;
  } else if (incomingByte == 'd') {
    if (kd <= 0.0001) kd = 0;
    else kd -= 0.0001;
  } else if (incomingByte == 'q') {
    if (kp == 0) kp = 0.01;
    else kp += 0.05;
  } else if (incomingByte == 'w') {
    if (ki == 0) ki = 0.01;
    else ki += 0.05;
  } else if (incomingByte == 'e') {
    if (kd == 0) kd = 0.0001;
    else kd += 0.0002;
  }

  roll_angle_pid.SetTunings(kp, ki, kd);
  pitch_angle_pid.SetTunings(kp, ki, kd);
}

void FlightController::compute_pids() {
  pid_setpoints[PID_ROLL_ANGLE] = rc->get(RC_ROLL);
  pid_setpoints[PID_PITCH_ANGLE] = rc->get(RC_PITCH);

  pid_inputs[PID_ROLL_ANGLE] = imu->x_angle;
  pid_inputs[PID_PITCH_ANGLE] = imu->y_angle;

  roll_angle_pid.Compute();
  pitch_angle_pid.Compute();
}

void FlightController::debug_output() {
  Serial.print("x_gyro: "); Serial.print(imu->x_rate);
  Serial.print(" \t y_gyro: "); Serial.print(imu->y_rate);
  Serial.print(" \t x_ang: "); Serial.print(imu->x_angle);
  Serial.print(" \t y_ang "); Serial.print(imu->y_angle);
  Serial.print(" \t x_ang_raw: "); Serial.print(imu->acc_x_in);
  Serial.print(" \t y_ang_raw: "); Serial.print(imu->acc_y_in);
  Serial.print(" \t z_ang_raw "); Serial.print(imu->acc_z_in);
  Serial.println();

  Serial.print("thrttl: "); Serial.print(rc->get(RC_THROTTLE));
  Serial.print("\t x_tar: "); Serial.print(rc->get(RC_ROLL));
  Serial.print("\t y_tar: "); Serial.print(rc->get(RC_PITCH));
  Serial.print("\t z_tar: "); Serial.print(rc->get(RC_YAW));
  Serial.print("\t pot_a: "); Serial.print(rc->get(RC_POT_A));
  Serial.print("\t pot_b: "); Serial.print(rc->get(RC_POT_B));
  Serial.println();

  Serial.print("M1_out: "); Serial.print(motors.outputs[M1]);
  Serial.print("\t M2_out: "); Serial.print(motors.outputs[M2]);
  Serial.print("\t M3_out: "); Serial.print(motors.outputs[M3]);
  Serial.print("\t M4_out: "); Serial.print(motors.outputs[M4]);
  Serial.println();

  Serial.print("roll_angle_pid: "); Serial.print(pid_outputs[PID_ROLL_ANGLE]);
  Serial.print("\t pitch_angle_pid: "); Serial.print(pid_outputs[PID_PITCH_ANGLE]);
  Serial.print("\t <kp: "); Serial.print(pitch_angle_pid.GetKp(), 5);
  Serial.print("\t <ki: "); Serial.print(pitch_angle_pid.GetKi(), 5);
  Serial.print("\t <kd: "); Serial.print(pitch_angle_pid.GetKd(), 5);
  Serial.println();

  if (emergency_stopped) Serial.println("\t EMERGENCY STOPPED!");
}

void FlightController::set_pid_output_limits() {
  roll_angle_pid.SetOutputLimits(-1000.0, 1000.0);
  pitch_angle_pid.SetOutputLimits(-1000.0, 1000.0);
}

void FlightController::emergency_stop() {
  emergency_stopped = true;
  motors.command_all_off();
}

void FlightController::safety_check() {
  // watchdog to prevent stale imu values
  if (imu->x_rate == last_gyro_value) {
    gyro_freeze_counter++;
    if (gyro_freeze_counter == 500) emergency_stop();
  } else {
    gyro_freeze_counter = 0;
    last_gyro_value = imu->x_rate;
  }

  if (imu->x_angle > 45.0 || imu->x_angle < -45.0
       || imu->y_angle > 45.0 || imu->y_angle < -45.0) {
    emergency_stop();
  }

  for(int i = 0; i < NUM_MOTORS; i++) {
    if (motors.outputs[i] > INDOOR_SAFE_MOTOR_SPEED) emergency_stop();
  }
}

void FlightController::set_safety_mode() {
  bool throttle_high = rc->get(RC_THROTTLE) > RC_THROTTLE_CUTOFF;

  if (throttle_high && !emergency_stopped) {
    if (safety_mode == UNARMED) {
      safety_mode = ARMED;
      reset_pids();
    }
  } else {
    safety_mode = UNARMED;
  }
}

FlightController::FlightController() :
   roll_angle_pid(&pid_inputs[PID_ROLL_ANGLE],
                  &pid_outputs[PID_ROLL_ANGLE],
                  &pid_setpoints[PID_ROLL_ANGLE],
                  0.0, 0.0, 0.0, REVERSE),
   pitch_angle_pid(&pid_inputs[PID_PITCH_ANGLE],
                   &pid_outputs[PID_PITCH_ANGLE],
                   &pid_setpoints[PID_PITCH_ANGLE],
                   0.0, 0.0, 0.0, REVERSE)
{}
