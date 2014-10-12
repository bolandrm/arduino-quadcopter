#ifndef flight_controller_h
#define flight_controller_h

#include "remote_control.h"
#include "imu.h"
#include "PID_v1.h"

#define NUM_PIDS 2
#define NUM_MOTORS 4
#define PID_ROLL 0
#define PID_PITCH 1
#define ARMED 1
#define UNARMED 0
#define MOTOR_SAFE_OFF 600
#define MOTOR_MIN 1070
#define RC_THROTTLE_CUTOFF 800
#define M1 0
#define M2 1
#define M3 2
#define M4 3

class FlightController {
  public:
    FlightController();

    void process(bool debug);
    void init(RemoteControl *, IMU *);

    int mode;

  private:
    void set_pid_output_limits();
    void adjust_pid_tuning();
    void set_safety_mode();
    void reset_pids();
    void compute_pids();
    void compute_motor_outputs();
    void zero_motor_outputs();
    void set_motor_outputs();
    void command_motors();
    void debug_output();

    RemoteControl *rc;
    IMU *imu;
    PID roll_pid, pitch_pid;

    double pid_inputs[2];
    double pid_outputs[2];
    double pid_setpoints[2];
    double motor_outputs[4];
};

#endif
