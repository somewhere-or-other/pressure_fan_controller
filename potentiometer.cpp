#include "potentiometer.h"


Adafruit_DS3502 potentiometerSetup() {
  Adafruit_DS3502 ds3502 = Adafruit_DS3502();
  if (!ds3502.begin()) {
    Serial.println("Couldn't find DS3502 chip");
    while (1);
  }
  return ds3502;
}
