#include "flight_controller.h"

void FlightController::init(RemoteControl *_rc, IMU *_imu) {
  rc = _rc;
  imu = _imu;

  mode = UNARMED;

  for (int i = 0; i < NUM_PIDS; i++) {
    pid_inputs[i] = 0.0;
    pid_outputs[i] = 0.0;
    pid_setpoints[i] = 0.0;
  }
}

void FlightController::process(bool debug) {
  set_safety_mode();

  if (mode == ARMED) {
    set_pid_output_limits();
    adjust_pid_tuning();
    compute_pids();
    compute_motor_outputs();
    //adjust_for_bounds();
  } else {
    zero_motor_outputs();
  }

  // command_motors();

  if (debug) { debug_output(); }
}

void FlightController::set_safety_mode() {
  if (rc->get(RC_THROTTLE) > RC_THROTTLE_CUTOFF) {
    if (mode == UNARMED) {
      mode = ARMED;
      reset_pids();
    }
  } else {
    mode = UNARMED;
  }
}

void FlightController::zero_motor_outputs() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motor_outputs[i] = MOTOR_SAFE_OFF;
  }
}

void FlightController::compute_motor_outputs() {
  double m1_fr_out = rc->get(RC_THROTTLE) + pid_outputs[PID_ROLL] + pid_outputs[PID_PITCH];
  double m2_bl_out = rc->get(RC_THROTTLE) - pid_outputs[PID_ROLL] - pid_outputs[PID_PITCH];
  double m3_fl_out = rc->get(RC_THROTTLE) - pid_outputs[PID_ROLL] + pid_outputs[PID_PITCH];
  double m4_br_out = rc->get(RC_THROTTLE) + pid_outputs[PID_ROLL] - pid_outputs[PID_PITCH];

  motor_outputs[M1] = (int16_t)(m1_fr_out + 0.5);
  motor_outputs[M2] = (int16_t)(m2_bl_out + 0.5);
  motor_outputs[M3] = (int16_t)(m3_fl_out + 0.5);
  motor_outputs[M4] = (int16_t)(m4_br_out + 0.5);
}

void FlightController::reset_pids() {
  // this will reset the I term to avoid windup
  roll_pid.SetOutputLimits(0.0, 0.01);
  pitch_pid.SetOutputLimits(0.0, 0.01);
}

void FlightController::adjust_pid_tuning() {
  double kp = 0; // 2 * rc.get(RC_POT_A) / 100.0;     // .92
  double kd = 0; // 0.5 * rc.get(RC_POT_B) / 100.0;   // .015
  double ki = 0; // 0.01;

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
  Serial.print(" fc:x angle: "); Serial.print(imu->x_angle);
  Serial.print(" fc:y angle: "); Serial.print(imu->y_angle);
  Serial.print(" fc:throttle: "); Serial.print(rc->get(RC_THROTTLE));
  Serial.println();
}

void FlightController::set_pid_output_limits() {
  roll_pid.SetOutputLimits(-1000.0, 1000.0);
  pitch_pid.SetOutputLimits(-1000.0, 1000.0);
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
