#include "flight_controller.h"

void FlightController::init(RemoteControl *_rc, IMU *_imu) {
  rc = _rc;
  imu = _imu;

  mode = STABILIZE;
  safety_mode = UNARMED;
  emergency_stopped = false;
  gyro_freeze_counter = 0;

  for (int i = 0; i < NUM_PIDS; i++) {
    pid_inputs[i] = 0.0;
    pid_outputs[i] = 0.0;
    pid_setpoints[i] = 0.0;
    pid_p_debugs[i] = 0.0;
    pid_i_debugs[i] = 0.0;
    pid_d_debugs[i] = 0.0;
  }

  roll_rate_pid.SetMode(AUTOMATIC);
  roll_rate_pid.SetSampleTime(3);
  roll_rate_pid.SetDebugParams(&pid_p_debugs[PID_ROLL_RATE],
                               &pid_i_debugs[PID_ROLL_RATE],
                               &pid_d_debugs[PID_ROLL_RATE]);

  pitch_rate_pid.SetMode(AUTOMATIC);
  pitch_rate_pid.SetSampleTime(3);
  pitch_rate_pid.SetDebugParams(&pid_p_debugs[PID_PITCH_RATE],
                                &pid_i_debugs[PID_PITCH_RATE],
                                &pid_d_debugs[PID_PITCH_RATE]);

  yaw_rate_pid.SetMode(AUTOMATIC);
  yaw_rate_pid.SetSampleTime(3);
  yaw_rate_pid.SetDebugParams(&pid_p_debugs[PID_YAW_RATE],
                                &pid_i_debugs[PID_YAW_RATE],
                                &pid_d_debugs[PID_YAW_RATE]);

  roll_angle_pid.SetMode(AUTOMATIC);
  roll_angle_pid.SetSampleTime(3);
  roll_angle_pid.SetDebugParams(&pid_p_debugs[PID_ROLL_ANGLE],
                                &pid_i_debugs[PID_ROLL_ANGLE],
                                &pid_d_debugs[PID_ROLL_ANGLE]);

  pitch_angle_pid.SetMode(AUTOMATIC);
  pitch_angle_pid.SetSampleTime(3);
  pitch_angle_pid.SetDebugParams(&pid_p_debugs[PID_PITCH_ANGLE],
                                 &pid_i_debugs[PID_PITCH_ANGLE],
                                 &pid_d_debugs[PID_PITCH_ANGLE]);

  motors.init();
}

void FlightController::process() {
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

  for(int i = 0; i < NUM_MOTORS; i++) {
    motors.outputs[i] += motor_fix;
  }
}

void FlightController::compute_motor_outputs() {

  double m1_r_out = rc->get(RC_THROTTLE) + pid_outputs[PID_ROLL_RATE] + pid_outputs[PID_YAW_RATE];
  double m2_l_out = rc->get(RC_THROTTLE) - pid_outputs[PID_ROLL_RATE] + pid_outputs[PID_YAW_RATE];
  double m3_f_out = rc->get(RC_THROTTLE) - pid_outputs[PID_PITCH_RATE] - pid_outputs[PID_YAW_RATE];
  double m4_b_out = rc->get(RC_THROTTLE) + pid_outputs[PID_PITCH_RATE] - pid_outputs[PID_YAW_RATE];

  // double m1_r_out = rc->get(RC_THROTTLE) + pid_outputs[PID_ROLL_RATE];
  // double m2_l_out = rc->get(RC_THROTTLE) - pid_outputs[PID_ROLL_RATE];
  // double m3_f_out = rc->get(RC_THROTTLE) - pid_outputs[PID_PITCH_RATE];
  // double m4_b_out = rc->get(RC_THROTTLE) + pid_outputs[PID_PITCH_RATE];

  motors.outputs[M1] = (int16_t)(m1_r_out + 0.5);
  motors.outputs[M2] = (int16_t)(m2_l_out + 0.5);
  motors.outputs[M3] = (int16_t)(m3_f_out + 0.5);
  motors.outputs[M4] = (int16_t)(m4_b_out + 0.5);
}

void FlightController::reset_pids() {
  // this will reset the I term to avoid windup
  roll_rate_pid.SetOutputLimits(0.0, 0.01);
  pitch_rate_pid.SetOutputLimits(0.0, 0.01);
  yaw_rate_pid.SetOutputLimits(0.0, 0.01);
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
    if (kd <= 0.05) kd = 0;
    else kd -= 0.05;
  } else if (incomingByte == 'q') {
    if (kp == 0) kp = 0.01;
    else kp += 0.05;
  } else if (incomingByte == 'w') {
    if (ki == 0) ki = 0.01;
    else ki += 0.05;
  } else if (incomingByte == 'e') {
    if (kd == 0) kd = 0.01;
    else kd += 0.05;
  }


  if (mode == STABILIZE) {
    roll_angle_pid.SetTunings(kp, ki, kd);
    pitch_angle_pid.SetTunings(kp, ki, kd);
  } else {
    roll_rate_pid.SetTunings(kp, ki, kd);
    pitch_rate_pid.SetTunings(kp, ki, kd);
  }
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
  pid_setpoints[PID_YAW_RATE] = rc->get(RC_YAW);

  pid_inputs[PID_ROLL_RATE] = imu->x_rate;
  pid_inputs[PID_PITCH_RATE] = imu->y_rate;
  pid_inputs[PID_YAW_RATE] = imu->z_rate;

  roll_rate_pid.Compute();
  pitch_rate_pid.Compute();
  yaw_rate_pid.Compute();
}

void FlightController::compute_pids() {
  if (mode == STABILIZE) {
    compute_angle_pids();
    pid_setpoints[PID_ROLL_RATE] = pid_outputs[PID_ROLL_ANGLE];
    pid_setpoints[PID_PITCH_RATE] = pid_outputs[PID_PITCH_ANGLE];
  } else {
    pid_setpoints[PID_ROLL_RATE] = rc->get(RC_ROLL);
    pid_setpoints[PID_PITCH_RATE] = rc->get(RC_PITCH);
  }

  compute_rate_pids();
}

void FlightController::debug_output() {
}

void FlightController::set_pid_output_limits() {
  roll_rate_pid.SetOutputLimits(-1000.0, 1000.0);
  pitch_rate_pid.SetOutputLimits(-1000.0, 1000.0);
  yaw_rate_pid.SetOutputLimits(-1000.0, 1000.0);
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
   roll_rate_pid(&pid_inputs[PID_ROLL_RATE],
                  &pid_outputs[PID_ROLL_RATE],
                  &pid_setpoints[PID_ROLL_RATE],
                  1.86, 0.36, 0.0, REVERSE),
   pitch_rate_pid(&pid_inputs[PID_PITCH_RATE],
                   &pid_outputs[PID_PITCH_RATE],
                   &pid_setpoints[PID_PITCH_RATE],
                   1.86, 0.36, 0.0, REVERSE),
   yaw_rate_pid(&pid_inputs[PID_YAW_RATE],
                &pid_outputs[PID_YAW_RATE],
                &pid_setpoints[PID_YAW_RATE],
                3.00, 0.00, 0.0, REVERSE),
                //1.00, 0.20, 0.0, REVERSE),
   roll_angle_pid(&pid_inputs[PID_ROLL_ANGLE],
                  &pid_outputs[PID_ROLL_ANGLE],
                  &pid_setpoints[PID_ROLL_ANGLE],
                  0.98, 0.08, 0.0, DIRECT),
   pitch_angle_pid(&pid_inputs[PID_PITCH_ANGLE],
                  &pid_outputs[PID_PITCH_ANGLE],
                  &pid_setpoints[PID_PITCH_ANGLE],
                  0.98, 0.08, 0.0, DIRECT)
{}
