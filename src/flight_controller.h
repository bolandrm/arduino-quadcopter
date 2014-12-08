#ifndef flight_controller_h
#define flight_controller_h

#include "remote_control.h"
#include "imu.h"
#include "motor_controller.h"
#include "PID_v1.h"

#define RATE 0
#define STABILIZE 1

#define NUM_PIDS 2
#define PID_ROLL_ANGLE 0
#define PID_PITCH_ANGLE 1

#define ARMED 1
#define UNARMED 0
#define RC_THROTTLE_CUTOFF 800
#define INDOOR_SAFE_MOTOR_SPEED 1800 //5000 //1800

class FlightController {
  public:
    FlightController();

    void process(bool debug);
    void init(RemoteControl *, IMU *);
    void emergency_stop();

    int safety_mode;

  private:
    void set_pid_output_limits();
    void adjust_pid_tuning();
    void set_safety_mode();
    void safety_check();
    void reset_pids();
    void compute_pids();
    void compute_angle_pids();
    void compute_rate_pids();
    void adjust_for_bounds();
    void compute_motor_outputs();
    void zero_motor_outputs();
    void set_motor_outputs();
    void command_motors();
    void debug_output();

    RemoteControl *rc;
    IMU *imu;
    PID roll_angle_pid, pitch_angle_pid;
    MotorController motors;

    uint16_t gyro_freeze_counter;
    float last_gyro_value;
    double pid_inputs[NUM_PIDS];
    double pid_outputs[NUM_PIDS];
    double pid_setpoints[NUM_PIDS];
    bool emergency_stopped;
    bool logging;
};

#endif
