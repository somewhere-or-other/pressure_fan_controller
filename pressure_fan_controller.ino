#define FANMIN 0
#define FANMAX 127
#define PRESSURETOLERANCE 2.0
#define PRESSURESETPOINT 0.0
#define PRESSUREMIN 0.0
#define PRESSUREMAX 100.0 //corresponds to maximum fan output

#define LCD_I2C_ADDR 0x27
#define LCD_COLUMNS 20
#define LCD_ROWS 4


//for display
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C display = LiquidCrystal_I2C(LCD_I2C_ADDR, LCD_COLUMNS, LCD_ROWS); 

//For pressure sensor
#include <Wire.h>
#include <sdpsensor.h>
SDP8XXSensor sdp;

//For potentiometer
#include <Adafruit_DS3502.h>
Adafruit_DS3502 potentiometer = Adafruit_DS3502();


float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int getPotentiometerSetting(float differentialPressure) {
  if (differentialPressure < (PRESSUREMIN+PRESSURETOLERANCE)) { // If pressure is at or below PRESSURESETPOINT (within PRESSURETOLERANCE)
    return FANMIN;
  } else if (differentialPressure > (PRESSUREMAX-PRESSURETOLERANCE)) {
    return FANMAX;
  } else {
    // TODO: this is where to integrate the PID controller mapping
    return floatMap(differentialPressure, PRESSUREMIN, PRESSUREMAX, FANMIN, FANMAX);
  }
}

void potentiometerSetup() {
  if (!potentiometer.begin()) {
    Serial.println("Couldn't find potentiometer chip");
    while (1);
  }
}

void potentiometerSet(uint8_t setting) {
  potentiometer.setWiper(setting);
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

void displaySetup() {
  display.backlight(); //turn on backlight
  display.clear();

  display.setCursor(0,0);
  display.print("Fan Speed Controller");
}

void displayToSerial(float pressure, uint8_t potentiometerSetting) {
  Serial.print("Measured_pressure: ");
  Serial.print(pressure);
  Serial.print(" pa | potentiometerSetting: ");
  Serial.println(potentiometerSetting);
}

void displayToLCD(float pressure, uint8_t potentiometerSetting) {
  char buffer[LCD_COLUMNS];
  snprintf(buffer, LCD_COLUMNS, "Pressure: %0.2f pa", pressure);
  display.setCursor(1,0);
  display.print(buffer);

  snprintf(buffer, LCD_COLUMNS, "Potentiometer: %u", potentiometerSetting);
  display.setCursor(2,0);
  display.print(buffer);
  
}
void setup() {
  Serial.begin(115200);
  // Wait until serial port is opened
  while (!Serial) { delay(1); }

  pressureSensorSetup();
  potentiometerSetup();

}



void loop() {

  float pressure = pressureSensorGet();
  float potentiometerSetting = getPotentiometerSetting(pressure);

  potentiometerSet(potentiometerSetting);

  displayToSerial(pressure, potentiometerSetting);
  displayToLCD(pressure, potentiometerSetting);
  
  

  delay(1000);
  

} 
