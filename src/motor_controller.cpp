#include "motor_controller.h"

#define M1_PIN 3
#define M2_PIN 9
#define M3_PIN 10
#define M4_PIN 11

#define M1_OUTPUT_REG OCR2B
#define M2_OUTPUT_REG OCR1A
#define M3_OUTPUT_REG OCR1B
#define M4_OUTPUT_REG OCR2A

void MotorController::command() {
  outputs[M1] = 2000;
  outputs[M2] = 2000;
  outputs[M3] = 1500;
  outputs[M4] = 1500;

  #ifdef ALLOW_MOTORS
    M1_OUTPUT_REG = outputs[M1] / 16;
    M2_OUTPUT_REG = outputs[M2] * 2;
    M3_OUTPUT_REG = outputs[M3] * 2;
    M4_OUTPUT_REG = outputs[M4] / 16;
  #else
    M1_OUTPUT_REG = 0;
    M2_OUTPUT_REG = 0;
    M3_OUTPUT_REG = 0;
    M4_OUTPUT_REG = 0;
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

  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(M3_PIN, OUTPUT);
  pinMode(M4_PIN, OUTPUT);

  // Init PWM Timer 1  16 bit
  // // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
  // // Prescaler set to 8, that gives us a resolution of 0.5us
  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1 = PWM_COUNTER_PERIOD;

  // Init PWM Timer 2   8bit                                 
  // WGMn1 WGMn2 = Mode ? Fast PWM, TOP = 0xFF ,Update of OCRnx at BOTTOM
  // Clear OCnA/OCnB on compare match, set OCnA/OCnB at BOTTOM (non-inverting mode)
  // Prescaler set to 256, that gives us a resolution of 16us
  // Output_PWM_Frequency = 244hz = 16000000/(256*(1+255)) = Clock_Speed / (Prescaler * (1 + TOP))
  // TOP is fixed at 255                                     
  TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);
  TCCR2B = (1<<CS22)|(1<<CS21);
}
