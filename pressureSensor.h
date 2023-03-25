#ifndef PRESSURESENSSOR_H
#define PRESSURESENSSOR_H

#include "pressureSensor.h"

#include <Wire.h>
#include <sdpsensor.h>


void sdpSetup();
float sdpGetPressure();

#endif
