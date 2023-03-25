#include "pressureSensor.h"

SDP8XXSensor sdp;


void sdpSetup() {
  Wire.begin();
  return;
}

float sdpGetPressure() {
  int ret = sdp.readSample();
  if (ret == 0) {
    return sdp.getDifferentialPressure();
  } else {
    return NULL;
  }
}
