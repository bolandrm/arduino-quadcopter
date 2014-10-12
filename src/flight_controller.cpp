#include "flight_controller.h"

FlightController::FlightController() {}

void FlightController::init(RemoteControl remote) {
  rc = remote;
}

void FlightController::process() {
}
