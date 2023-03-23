#include <Adafruit_DS3502.h>
#include <Wire.h>
#include <sdpsensor.h>

#define FANMIN 0
#define FANMAX 127
#define PRESSURETOLERANCE 0.5
#define PRESSURESETPOINT 0.0
#define PRESSUREMIN 0.0
#define PRESSUREMAX 100.0 //corresponds to maximum fan output

SDP8XXSensor sdp;

Adafruit_DS3502 ds3502 = Adafruit_DS3502();
/* For this example, make the following connections:
    * DS3502 RH to 5V
    * DS3502 RL to GND
    * DS3502 RW to the pin specified by WIPER_VALUE_PIN
*/

#define WIPER_VALUE_PIN A0

void potentiometerSetup() {
  Serial.println("Adafruit DS3502 Test");

  if (!ds3502.begin()) {
    Serial.println("Couldn't find DS3502 chip");
    while (1);
  }
  Serial.println("Found DS3502 chip");
}

void potentiometerLoop() {
  Serial.print("Wiper voltage with wiper set to 0: ");
  ds3502.setWiper(0);
  float wiper_value = analogRead(WIPER_VALUE_PIN);
  wiper_value *= 5.0;
  wiper_value /= 1024;
  Serial.print(wiper_value);
  Serial.println(" V");

  Serial.println();
  delay(1000);

  Serial.print("Wiper voltage with wiper set to 63: ");
  ds3502.setWiper(63);
  wiper_value = analogRead(WIPER_VALUE_PIN);
  wiper_value *= 5.0;
  wiper_value /= 1024;
  Serial.print(wiper_value);
  Serial.println(" V");

  Serial.println();
  delay(1000);

  Serial.print("Wiper voltage with wiper set to 127: ");
  ds3502.setWiper(127);
  wiper_value = analogRead(WIPER_VALUE_PIN);
  wiper_value *= 5.0;
  wiper_value /= 1024;
  Serial.print(wiper_value);
  Serial.println(" V");

  Serial.println();
  delay(1000);
}

void sdpSetup() {
  Wire.begin();
  return;
}

void sdpLoop() {
  int ret = sdp.readSample();
  if (ret == 0) {
    Serial.print("Differential pressure: ");
    Serial.print(sdp.getDifferentialPressure());
    Serial.print("Pa | ");

    Serial.print("Temp: ");
    Serial.print(sdp.getTemperature());
    Serial.print("C\n");
  } else {
    Serial.print("Error in readSample(), ret = ");
    Serial.println(ret);
  }

  delay(500);  
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
  } else {
    // TODO: this is where to integrate the PID controller mapping
    return floatToIntMap(differentialPressure, PRESSUREMIN, PRESSUREMAX, FANMIN, FANMAX);
  }
}

void loop() {
  // potentiometerLoop();

  // sdpLoop();

  float pressure = sdpGetPressure();
  float potentiometerSetting = getPotentiometerSetting(pressure);

  Serial.print("Mesured_pressure: ");
  Serial.print(pressure);
  Serial.print(" pa | potentiometerSetting: ");
  Serial.println(potentiometerSetting);
  
  setPotentiometer(potentiometerSetting);

  delay(500);
  

}