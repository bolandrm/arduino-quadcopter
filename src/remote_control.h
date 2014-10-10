#ifndef remote_control_h
#define remote_control_h

#include <Arduino.h>
#include <PinChangeInt.h>

#define  RC_CH1_INPUT  A10
#define  RC_CH2_INPUT  A11
#define  RC_CH3_INPUT  A12
#define  RC_CH4_INPUT  A13
#define  RC_CH5_INPUT  A14
#define  RC_CH6_INPUT  A15

#define  RC_CH1_IN_MIN   1154
#define  RC_CH1_IN_MAX   1898
#define  RC_CH1_OUT_MIN  -25
#define  RC_CH1_OUT_MAX  25

#define  RC_CH2_IN_MIN   1192
#define  RC_CH2_IN_MAX   1824
#define  RC_CH2_OUT_MIN  -25
#define  RC_CH2_OUT_MAX  25

#define  RC_CH3_IN_MIN   1000
#define  RC_CH3_IN_MAX   1812
#define  RC_CH3_OUT_MIN  600
#define  RC_CH3_OUT_MAX  1600 // 1864 is max motor input

#define  RC_CH4_IN_MIN   1171
#define  RC_CH4_IN_MAX   1871
#define  RC_CH4_OUT_MIN  -150
#define  RC_CH4_OUT_MAX  150

#define  RC_CH5_IN_MIN   996
#define  RC_CH5_IN_MAX   2000
#define  RC_CH5_OUT_MIN  0
#define  RC_CH5_OUT_MAX  100

#define  RC_CH6_IN_MIN   996
#define  RC_CH6_IN_MAX   2000
#define  RC_CH6_OUT_MIN  0
#define  RC_CH6_OUT_MAX  100

#define  RC_CH1  0
#define  RC_CH2  1
#define  RC_CH3  2
#define  RC_CH4  3
#define  RC_CH5  4
#define  RC_CH6  5

#define RC_ROLL     RC_CH1
#define RC_PITCH    RC_CH2
#define RC_THROTTLE RC_CH3
#define RC_YAW      RC_CH4
#define RC_POT_A    RC_CH5
#define RC_POT_B    RC_CH6

uint32_t rc_start[6];
uint32_t rc_compare;

volatile uint16_t rc_shared[6];
volatile uint8_t rc_update_flags;
volatile uint8_t rc_update_flags_shared;

int16_t rc_in_min[] = { RC_CH1_IN_MIN, RC_CH2_IN_MIN, RC_CH3_IN_MIN,
                        RC_CH4_IN_MIN, RC_CH5_IN_MIN, RC_CH6_IN_MIN };
int16_t rc_in_max[] = { RC_CH1_IN_MAX, RC_CH2_IN_MAX, RC_CH3_IN_MAX,
                        RC_CH4_IN_MAX, RC_CH5_IN_MAX, RC_CH6_IN_MAX };
int16_t rc_out_min[] = { RC_CH1_OUT_MIN, RC_CH2_OUT_MIN, RC_CH3_OUT_MIN,
                         RC_CH4_OUT_MIN, RC_CH5_OUT_MIN, RC_CH6_OUT_MIN };
int16_t rc_out_max[] = { RC_CH1_OUT_MAX, RC_CH2_OUT_MAX, RC_CH3_OUT_MAX,
                         RC_CH4_OUT_MAX, RC_CH5_OUT_MAX, RC_CH6_OUT_MAX };


class RemoteControl {
  private:
    static uint16_t rc_values[6];

  public:
    RemoteControl();
    void read_values();
    static void calc_input(int, int);
    static void calc_ch_1();
    static void calc_ch_2();
    static void calc_ch_3();
    static void calc_ch_4();
    static void calc_ch_5();
    static void calc_ch_6();
    static int16_t get(int);
};

uint16_t RemoteControl::rc_values[] = {0};

void RemoteControl::read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  rc_update_flags = rc_update_flags_shared;
  rc_update_flags_shared = 0;
  interrupts();
}

int16_t RemoteControl::get(int channel) {
  int16_t value = rc_values[channel];
  value = constrain(value, rc_in_min[channel], rc_in_max[channel]);
  value = map(value, rc_in_min[channel], rc_in_max[channel], rc_out_min[channel], rc_out_max[channel]);

  if ((channel == RC_CH1 || channel == RC_CH2 || channel == RC_CH4)
       && (value > 0 && value < 5 || value < 0 && value > -5) ) {
    value = 0;
  }

  if (channel == RC_CH1 || channel == RC_CH2) {
    value = -value; // invert roll & pitch
  }

  if (channel == RC_CH3 && value < 1070 && value > 750) {
    value = 1070;
  }

  return value;
}

void RemoteControl::calc_input(int channel, int input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void RemoteControl::calc_ch_1() { RemoteControl::calc_input(RC_CH1, RC_CH1_INPUT); }
void RemoteControl::calc_ch_2() { RemoteControl::calc_input(RC_CH2, RC_CH2_INPUT); }
void RemoteControl::calc_ch_3() { RemoteControl::calc_input(RC_CH3, RC_CH3_INPUT); }
void RemoteControl::calc_ch_4() { RemoteControl::calc_input(RC_CH4, RC_CH4_INPUT); }
void RemoteControl::calc_ch_5() { RemoteControl::calc_input(RC_CH5, RC_CH5_INPUT); }
void RemoteControl::calc_ch_6() { RemoteControl::calc_input(RC_CH6, RC_CH6_INPUT); }

RemoteControl::RemoteControl() {
  PCintPort::attachInterrupt(RC_CH1_INPUT, this->calc_ch_1, CHANGE);
  PCintPort::attachInterrupt(RC_CH2_INPUT, this->calc_ch_2, CHANGE);
  PCintPort::attachInterrupt(RC_CH3_INPUT, this->calc_ch_3, CHANGE);
  PCintPort::attachInterrupt(RC_CH4_INPUT, this->calc_ch_4, CHANGE);
  PCintPort::attachInterrupt(RC_CH5_INPUT, this->calc_ch_5, CHANGE);
  PCintPort::attachInterrupt(RC_CH6_INPUT, this->calc_ch_6, CHANGE);
}

#endif
