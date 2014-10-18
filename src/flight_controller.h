#ifndef flight_controller_h
#define flight_controller_h

#include "remote_control.h"
#include "imu.h"
#include "motor_controller.h"
#include "PID_v1.h"

#define NUM_PIDS 2
#define PID_ROLL 0
#define PID_PITCH 1
#define ARMED 1
#define UNARMED 0
#define RC_THROTTLE_CUTOFF 800
#define CONTINUOUS 0
#define INDOOR_SAFE_MOTOR_SPEED 1300

class FlightController {
  public:
    FlightController();

    void process(bool debug);
    void init(RemoteControl *, IMU *);
    void emergency_stop();

    int mode;

  private:
    void set_pid_output_limits();
    void adjust_pid_tuning();
    void set_safety_mode();
    void safety_check();
    void reset_pids();
    void compute_pids();
    void adjust_for_bounds();
    void compute_motor_outputs();
    void zero_motor_outputs();
    void set_motor_outputs();
    void command_motors();
    void debug_output();

    RemoteControl *rc;
    IMU *imu;
    PID roll_pid, pitch_pid;
    MotorController motors;

    double pid_inputs[2];
    double pid_outputs[2];
    double pid_setpoints[2];
    bool emergency_stopped;
};

#endif
