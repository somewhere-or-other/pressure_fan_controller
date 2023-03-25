#include <Adafruit_DS3502.h>
#include <Wire.h>
#include <sdpsensor.h>

#define FANMIN 0
#define FANMAX 127
#define PRESSURETOLERANCE 2.0
#define PRESSURESETPOINT 0.0
#define PRESSUREMIN 0.0
#define PRESSUREMAX 100.0 //corresponds to maximum fan output

SDP8XXSensor sdp;

Adafruit_DS3502 ds3502 = Adafruit_DS3502();


void potentiometerSetup() {
  
  if (!ds3502.begin()) {
    Serial.println("Couldn't find DS3502 chip");
    while (1);
  }
  
}



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

void setPotentiometer(int set) {
  ds3502.setWiper(set);
}

void setup() {
  Serial.begin(115200);
  // Wait until serial port is opened
  while (!Serial) { delay(1); }

  potentiometerSetup();
  sdpSetup();

}

int floatToIntMap(float x, float in_min, float in_max, int out_min, int out_max) {
  return (x - in_min) * ((float)out_max - (float)out_min) / (in_max - in_min) + (float)out_min;
}

int getPotentiometerSetting(float differentialPressure) {
  if ((differentialPressure-PRESSURESETPOINT) <= PRESSURETOLERANCE) { // If pressure is at or below PRESSURESETPOINT (within PRESSURETOLERANCE)
    return FANMIN;
  } else if (differentialPressure > (PRESSURESETPOINT-PRESSURETOLERANCE)) {
    return FANMAX;
  } else {
    // TODO: this is where to integrate the PID controller mapping
    return floatToIntMap(differentialPressure, PRESSUREMIN, PRESSUREMAX, FANMIN, FANMAX);
  }
}

void loop() {

  float pressure = sdpGetPressure();
  float potentiometerSetting = getPotentiometerSetting(pressure);

  Serial.print("Mesured_pressure: ");
  Serial.print(pressure);
  Serial.print(" pa | potentiometerSetting: ");
  Serial.println(potentiometerSetting);
  
  setPotentiometer(potentiometerSetting);

  delay(500);
  

} 
