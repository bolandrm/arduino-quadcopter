#include <PinChangeInt.h>

#define  RC_CH1_INPUT  A10
#define  RC_CH2_INPUT  A11
#define  RC_CH3_INPUT  A12
#define  RC_CH4_INPUT  A13
#define  RC_CH5_INPUT  A14
#define  RC_CH6_INPUT  A15

void calc_ch_1() { RemoteControl::calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch_2() { RemoteControl::calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch_3() { RemoteControl::calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch_4() { RemoteControl::calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch_5() { RemoteControl::calc_input(RC_CH5, RC_CH5_INPUT); }
void calc_ch_6() { RemoteControl::calc_input(RC_CH6, RC_CH6_INPUT); }

void bind_rc_interrupts(RemoteControl rc) {
  PCintPort::attachInterrupt(RC_CH1_INPUT, calc_ch_1, CHANGE);
  PCintPort::attachInterrupt(RC_CH2_INPUT, calc_ch_2, CHANGE);
  PCintPort::attachInterrupt(RC_CH3_INPUT, calc_ch_3, CHANGE);
  PCintPort::attachInterrupt(RC_CH4_INPUT, calc_ch_4, CHANGE);
  PCintPort::attachInterrupt(RC_CH5_INPUT, calc_ch_5, CHANGE);
  PCintPort::attachInterrupt(RC_CH6_INPUT, calc_ch_6, CHANGE);
}
