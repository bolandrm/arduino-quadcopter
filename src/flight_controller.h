#ifndef flight_controller_h
#define flight_controller_h

#include "remote_control.h"

class FlightController {
  public:
    FlightController();

    void process();
    void init(RemoteControl);

  private:
    RemoteControl rc;
};

#endif
