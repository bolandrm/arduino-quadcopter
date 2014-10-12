#ifndef motor_controller_h
#define motor_controller_h

#define PWM_FREQUENCY 400   // in Hz (300 other val)
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

#define MOTOR_SAFE_OFF 600
#define MOTOR_MIN 1070
#define MOTOR_MAX 1860

#define NUM_MOTORS 4
#define M1 0
#define M2 1
#define M3 2
#define M4 3

#include <Arduino.h>

class MotorController {
  public:
    MotorController();
    void command_all_off();
    void command();

    uint16_t outputs[4];

  private:
    void zero_outputs();
};

#endif
