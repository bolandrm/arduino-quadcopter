#ifndef debugger_h
#define debugger_h

#define DEBUG true
#define CHART_DEBUG false

#if DEBUG
  #if CHART_DEBUG
  #define DEBUG_RATE_MILLIS 50
  #else
  #define DEBUG_RATE_MILLIS 500
  #endif
#endif

#include "remote_control.h"
#include "flight_controller.h"
#include "imu.h"

class Debugger {
  public:
    void init(RemoteControl*, IMU*, FlightController*);
    void print();

  private:
    RemoteControl *rc;
    IMU *imu;
    FlightController *fc;

    void chart_debug();
    void print_debug();

    uint32_t last_debug_time;
    uint32_t loop_time;
    uint32_t loop_start_time;
};

#endif
