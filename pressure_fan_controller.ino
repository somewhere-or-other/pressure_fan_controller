#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C display = LiquidCrystal_I2C(0x27, 20, 4); //default 0x27 I2C address, 20 columns, 4 rows

//For pressure sensor
#include <Wire.h>
#include <sdpsensor.h>
SDP8XXSensor sdp;

//For potentiometer
#include <Adafruit_DS3502.h>
Adafruit_DS3502 ds3502 = Adafruit_DS3502();

void potentiometerSetup() {
  if (!ds3502.begin()) {
    Serial.println("Couldn't find DS3502 chip");
    while (1);
  }
}


void pressureSensorSetup() {
  Wire.begin();
}

float pressureSensorGet() {
  int ret = sdp.readSample();
  if (ret == 0) {
    return sdp.getDifferentialPressure();
  } else {
    return NULL;
  }
}


void setup() {
  pressureSensorSetup();
  potentiometerSetup();
  
}

void loop() {

  

} 
