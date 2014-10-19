#include "motor_controller.h"

void MotorController::command() {
  #ifdef ALLOW_MOTORS
    OCR3C = outputs[M1] * 2;
    OCR3A = outputs[M2] * 2;
    //OCR4A = outputs[M3] * 2;
    //OCR4B = outputs[M4] * 2;
    OCR4A = 0;
    OCR4B = 0;
  #else
    OCR3C = 0;
    OCR3A = 0;
    OCR4A = 0;
    OCR4B = 0;
  #endif
}

void MotorController::command_all_off() {
  zero_outputs();
  command();
}

void MotorController::zero_outputs() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    outputs[i] = MOTOR_SAFE_OFF;
  }
}

void MotorController::init() {
  zero_outputs();

  pinMode(3, OUTPUT);  // motor 1
  pinMode(5, OUTPUT);  // motor 2
  pinMode(6, OUTPUT);  // motor 3
  pinMode(7, OUTPUT);  // motor 4

  // Timer 3
  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
  // Prescaler set to 8, that gives us a resolution of 0.5us
  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
  ICR3 = PWM_COUNTER_PERIOD;

  // Timer 4
  TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PWM_COUNTER_PERIOD;
}
