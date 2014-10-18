#include "flight_controller.h"

void FlightController::init(RemoteControl *_rc, IMU *_imu) {
  rc = _rc;
  imu = _imu;

  mode = UNARMED;
  emergency_stopped = false;

  for (int i = 0; i < NUM_PIDS; i++) {
    pid_inputs[i] = 0.0;
    pid_outputs[i] = 0.0;
    pid_setpoints[i] = 0.0;
  }

  roll_pid.SetMode(AUTOMATIC);
  pitch_pid.SetMode(AUTOMATIC);
  roll_pid.SetResolution(MICROS);
  pitch_pid.SetResolution(MICROS);
  roll_pid.SetSampleTime(CONTINUOUS);
  pitch_pid.SetSampleTime(CONTINUOUS);

  motors.init();
}

void FlightController::process(bool debug) {
  set_safety_mode();

  set_pid_output_limits();
  adjust_pid_tuning();
  compute_pids();

  if (mode == ARMED) {
    compute_motor_outputs();
    adjust_for_bounds();
  } else {
    motors.command_all_off();
  }

  motors.command();

  if (debug) { debug_output(); }
}

void FlightController::set_safety_mode() {
  bool throttle_high = rc->get(RC_THROTTLE) > RC_THROTTLE_CUTOFF;

  if (throttle_high && !emergency_stopped) {
    if (mode == UNARMED) {
      mode = ARMED;
      reset_pids();
    }
  } else {
    mode = UNARMED;
  }
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
  double m1_fr_out = rc->get(RC_THROTTLE) + pid_outputs[PID_ROLL] + pid_outputs[PID_PITCH];
  double m2_bl_out = rc->get(RC_THROTTLE) - pid_outputs[PID_ROLL] - pid_outputs[PID_PITCH];
  double m3_fl_out = rc->get(RC_THROTTLE) - pid_outputs[PID_ROLL] + pid_outputs[PID_PITCH];
  double m4_br_out = rc->get(RC_THROTTLE) + pid_outputs[PID_ROLL] - pid_outputs[PID_PITCH];

  motors.outputs[M1] = (int16_t)(m1_fr_out + 0.5);
  motors.outputs[M2] = (int16_t)(m2_bl_out + 0.5);
  motors.outputs[M3] = (int16_t)(m3_fl_out + 0.5);
  motors.outputs[M4] = (int16_t)(m4_br_out + 0.5);
}

void FlightController::reset_pids() {
  // this will reset the I term to avoid windup
  roll_pid.SetOutputLimits(0.0, 0.01);
  pitch_pid.SetOutputLimits(0.0, 0.01);
}

void FlightController::adjust_pid_tuning() {
  double kp = rc->get(RC_POT_A) / 100.0;
  double kd = 0.0; // 0.18; //0.5 * rc->get(RC_POT_B) / 100.0;
  double ki = 0.5 * rc->get(RC_POT_B) / 100.0;

  roll_pid.SetTunings(kp, ki, kd);
  pitch_pid.SetTunings(kp, ki, kd);
}

void FlightController::compute_pids() {
  pid_setpoints[PID_ROLL] = rc->get(RC_ROLL);
  pid_setpoints[PID_PITCH] = rc->get(RC_PITCH);

  pid_inputs[PID_ROLL] = imu->x_angle;
  pid_inputs[PID_PITCH] = imu->y_angle;

  roll_pid.Compute();
  pitch_pid.Compute();
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
  Serial.print("\t M1_out: "); Serial.print(motors.outputs[M1]);
  Serial.print("\t M2_out: "); Serial.print(motors.outputs[M2]);
  Serial.print("\t M3_out: "); Serial.print(motors.outputs[M3]);
  Serial.print("\t M4_out: "); Serial.print(motors.outputs[M4]);
  Serial.print("\t kp: "); Serial.print(roll_pid.GetKp());
  Serial.print("\t kd: "); Serial.print(roll_pid.GetKd());
  Serial.print("\t ki: "); Serial.print(roll_pid.GetKi());
  Serial.println();
}

void FlightController::set_pid_output_limits() {
  roll_pid.SetOutputLimits(-1000.0, 1000.0);
  pitch_pid.SetOutputLimits(-1000.0, 1000.0);
}

void FlightController::emergency_stop() {
  emergency_stopped = true;
  motors.command_all_off();
}

FlightController::FlightController() :
   roll_pid(&pid_inputs[PID_ROLL],
            &pid_outputs[PID_ROLL],
            &pid_setpoints[PID_ROLL],
            0.0, 0.0, 0.0, DIRECT),
   pitch_pid(&pid_inputs[PID_PITCH],
             &pid_outputs[PID_PITCH],
             &pid_setpoints[PID_PITCH],
             0.0, 0.0, 0.0, DIRECT)
{}
