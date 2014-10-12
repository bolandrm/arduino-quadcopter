#ifndef flight_controller_h
#define flight_controller_h

#include "remote_control.h"
#include "PID_v1.h"

#define NUM_PIDS 2
#define PID_ROLL 0
#define PID_PITCH 1

class FlightController {
  public:
    FlightController();

    void process();
    void init(RemoteControl);

  private:
    RemoteControl rc;
    PID roll_pid, pitch_pid;

    double pid_inputs[2];
    double pid_outputs[2];
    double pid_setpoints[2];
};

#endif
