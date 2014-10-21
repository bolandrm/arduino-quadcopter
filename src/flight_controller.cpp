#include "flight_controller.h"

void FlightController::init(RemoteControl *_rc, IMU *_imu) {
  rc = _rc;
  imu = _imu;

  mode = RATE;
  safety_mode = UNARMED;
  emergency_stopped = false;
  gyro_freeze_counter = 0;

  for (int i = 0; i < NUM_PIDS; i++) {
    pid_inputs[i] = 0.0;
    pid_outputs[i] = 0.0;
    pid_setpoints[i] = 0.0;
  }

  roll_rate_pid.SetMode(AUTOMATIC);
  roll_rate_pid.SetResolution(MICROS);
  roll_rate_pid.SetSampleTime(CONTINUOUS);
  //roll_rate_pid.SetITermMax(25.0);
  pitch_rate_pid.SetMode(AUTOMATIC);
  pitch_rate_pid.SetResolution(MICROS);
  pitch_rate_pid.SetSampleTime(CONTINUOUS);
  //pitch_rate_pid.SetITermMax(25.0);

  roll_angle_pid.SetMode(AUTOMATIC);
  roll_angle_pid.SetResolution(MICROS);
  roll_angle_pid.SetSampleTime(CONTINUOUS);
  pitch_angle_pid.SetMode(AUTOMATIC);
  pitch_angle_pid.SetResolution(MICROS);
  pitch_angle_pid.SetSampleTime(CONTINUOUS);

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
  double m1_fr_out = rc->get(RC_THROTTLE) - pid_outputs[PID_ROLL_RATE] - pid_outputs[PID_PITCH_RATE];
  double m2_bl_out = rc->get(RC_THROTTLE) + pid_outputs[PID_ROLL_RATE] + pid_outputs[PID_PITCH_RATE];
  double m3_fl_out = rc->get(RC_THROTTLE) + pid_outputs[PID_ROLL_RATE] - pid_outputs[PID_PITCH_RATE];
  double m4_br_out = rc->get(RC_THROTTLE) - pid_outputs[PID_ROLL_RATE] + pid_outputs[PID_PITCH_RATE];

  motors.outputs[M1] = (int16_t)(m1_fr_out + 0.5);
  motors.outputs[M2] = (int16_t)(m2_bl_out + 0.5);
  motors.outputs[M3] = (int16_t)(m3_fl_out + 0.5);
  motors.outputs[M4] = (int16_t)(m4_br_out + 0.5);
}

void FlightController::reset_pids() {
  // this will reset the I term to avoid windup
  roll_rate_pid.SetOutputLimits(0.0, 0.01);
  pitch_rate_pid.SetOutputLimits(0.0, 0.01);
  roll_angle_pid.SetOutputLimits(0.0, 0.01);
  pitch_angle_pid.SetOutputLimits(0.0, 0.01);
}

void FlightController::adjust_pid_tuning() {
  if (Serial.available() <= 0) return;
  byte incomingByte = Serial.read();

  double kp, kd, ki;

  if (mode == STABILIZE) {
    kp = roll_angle_pid.GetKp();
    ki = roll_angle_pid.GetKi();
    kd = roll_angle_pid.GetKd();
  } else {
    kp = roll_rate_pid.GetKp();
    ki = roll_rate_pid.GetKi();
    kd = roll_rate_pid.GetKd();
  }

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

  if (mode == STABILIZE) {
    roll_angle_pid.SetTunings(kp, ki, kd);
    pitch_angle_pid.SetTunings(kp, ki, kd);
  } else {
    roll_rate_pid.SetTunings(kp, ki, kd);
    pitch_rate_pid.SetTunings(kp, ki, kd);
  }
}

void FlightController::compute_pids() {
  if (mode == STABILIZE) compute_angle_pids();
  compute_rate_pids();
}

void FlightController::compute_angle_pids() {
  pid_setpoints[PID_ROLL_ANGLE] = rc->get(RC_ROLL);
  pid_setpoints[PID_PITCH_ANGLE] = rc->get(RC_PITCH);

  pid_inputs[PID_ROLL_ANGLE] = imu->x_angle;
  pid_inputs[PID_PITCH_ANGLE] = imu->y_angle;

  roll_angle_pid.Compute();
  pitch_angle_pid.Compute();
}

void FlightController::compute_rate_pids() {
  if (mode == STABILIZE) {
    pid_setpoints[PID_ROLL_RATE] = pid_outputs[PID_ROLL_ANGLE];
    pid_setpoints[PID_PITCH_RATE] = pid_outputs[PID_PITCH_ANGLE];
  } else {
    pid_setpoints[PID_ROLL_RATE] = rc->get(RC_ROLL);
    pid_setpoints[PID_PITCH_RATE] = rc->get(RC_PITCH);
  }

  pid_inputs[PID_ROLL_RATE] = imu->x_rate;
  pid_inputs[PID_PITCH_RATE] = imu->y_rate;

  roll_rate_pid.Compute();
  pitch_rate_pid.Compute();
}

void FlightController::debug_output() {
  Serial.print("x_gyro: "); Serial.print(imu->x_rate);
  Serial.print(" \t y_gyro: "); Serial.print(imu->y_rate);
  Serial.print(" \t x_ang: "); Serial.print(imu->x_angle);
  Serial.print(" \t y_ang "); Serial.print(imu->y_angle);
  Serial.println();
  Serial.print("thrttl: "); Serial.print(rc->get(RC_THROTTLE));
  Serial.print("\t x_tar: "); Serial.print(rc->get(RC_ROLL));
  Serial.print("\t y_tar: "); Serial.print(rc->get(RC_PITCH));
  Serial.print("\t pot_a: "); Serial.print(rc->get(RC_POT_A));
  Serial.print("\t M1_out: "); Serial.print(motors.outputs[M1]);
  Serial.print("\t M2_out: "); Serial.print(motors.outputs[M2]);
  Serial.print("\t M3_out: "); Serial.print(motors.outputs[M3]);
  Serial.print("\t M4_out: "); Serial.print(motors.outputs[M4]);
  Serial.print("\t <kp: "); Serial.print(pitch_angle_pid.GetKp(), 5);
  Serial.print("\t <ki: "); Serial.print(pitch_angle_pid.GetKi(), 5);
  Serial.print("\t <kd: "); Serial.print(pitch_angle_pid.GetKd(), 5);
  Serial.print("\t rate_kp: "); Serial.print(pitch_rate_pid.GetKp(), 5);
  Serial.print("\t rate_ki: "); Serial.print(pitch_rate_pid.GetKi(), 5);
  Serial.print("\t rate_kd: "); Serial.print(pitch_rate_pid.GetKd(), 5);
  Serial.println();
  Serial.print("roll_rate_pid: "); Serial.print(pid_outputs[PID_ROLL_RATE]);
  Serial.print("\t pitch_rate_pid: "); Serial.print(pid_outputs[PID_PITCH_RATE]);
  Serial.print("\t roll_angle_pid: "); Serial.print(pid_outputs[PID_ROLL_ANGLE]);
  Serial.print("\t pitch_angle_pid: "); Serial.print(pid_outputs[PID_PITCH_ANGLE]);
  if (emergency_stopped) Serial.print("\t EMERGENCY STOPPED!");
  Serial.println();
}

void FlightController::set_pid_output_limits() {
  roll_rate_pid.SetOutputLimits(-1000.0, 1000.0);
  pitch_rate_pid.SetOutputLimits(-1000.0, 1000.0);
  roll_angle_pid.SetOutputLimits(-1000.0, 1000.0);
  pitch_angle_pid.SetOutputLimits(-1000.0, 1000.0);
}

void FlightController::emergency_stop() {
  emergency_stopped = true;
  imu->reset();
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
   roll_rate_pid(&pid_inputs[PID_ROLL_RATE],
                 &pid_outputs[PID_ROLL_RATE],
                 &pid_setpoints[PID_ROLL_RATE],
                 1.59, 2.13, 0.0002, REVERSE),
   pitch_rate_pid(&pid_inputs[PID_PITCH_RATE],
                  &pid_outputs[PID_PITCH_RATE],
                  &pid_setpoints[PID_PITCH_RATE],
                  1.59, 2.13, 0.0002, REVERSE),
   roll_angle_pid(&pid_inputs[PID_ROLL_ANGLE],
                  &pid_outputs[PID_ROLL_ANGLE],
                  &pid_setpoints[PID_ROLL_ANGLE],
                  0.0, 0.0, 0.0, DIRECT),
   pitch_angle_pid(&pid_inputs[PID_PITCH_ANGLE],
                   &pid_outputs[PID_PITCH_ANGLE],
                   &pid_setpoints[PID_PITCH_ANGLE],
                   0.0, 0.0, 0.0, REVERSE)
{}
