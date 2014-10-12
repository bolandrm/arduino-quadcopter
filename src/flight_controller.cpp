#include "flight_controller.h"

void FlightController::init(RemoteControl remote) {
  rc = remote;

  for (int i = 0; i < NUM_PIDS; i++) {
    pid_inputs[i] = 0.0;
    pid_outputs[i] = 0.0;
    pid_setpoints[i] = 0.0;
  }
}

void FlightController::process() {
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
