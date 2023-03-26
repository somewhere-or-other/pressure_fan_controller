#define OUTPUTMIN 0
#define OUTPUTMAX 127
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


//for PID functionality
#include <PID_v2.h>
double Kp = (OUTPUTMAX-OUTPUTMIN)/(PRESSUREMAX-PRESSUREMIN) + OUTPUTMIN;
double Ki = 0;
double Kd = 1;
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);

char buffer[LCD_COLUMNS];
char doublebuffer[10];


double doubleMap(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double outputPercentageFromSetting(int x) {
  return doubleMap(x, OUTPUTMIN, OUTPUTMAX, 0, 100);
}

double getPotentiometerSetting(double differentialPressure) {
    double output = myPID.Run(differentialPressure); 
      

  if (output < OUTPUTMIN) {
    return OUTPUTMIN;
  } else if (output > OUTPUTMAX) {
    return OUTPUTMAX;
  } else {
      return output;
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
  
}

double pressureSensorGet() {
  int ret = sdp.readSample();
  if (ret == 0) {
    return sdp.getDifferentialPressure();
  } else {
    return NULL;
  }
}

void displaySetup() {
  display.init();
  display.clear();
  display.noAutoscroll();
  display.backlight(); //turn on backlight

  display.setCursor(0,0);
  display.print("Fan Speed Controller");

}

void displayValuesToSerial(double pressure, uint8_t potentiometerSetting) {
  Serial.print("PressureSetpoint_pa:");
  Serial.print(PRESSURESETPOINT);
  Serial.print(",MeasuredPressure_pa:");
  Serial.print(pressure);
  Serial.print(",outputPercent:");
  Serial.println(outputPercentageFromSetting(potentiometerSetting));
}

void displayValuesToLCD(double pressure, uint8_t potentiometerSetting) {

  memset(doublebuffer, '\0' , strlen(doublebuffer));
  dtostrf(pressure, 7, 2, doublebuffer);

  snprintf(buffer, LCD_COLUMNS, "Pressure:%s pa", doublebuffer);
  display.setCursor(0,2);
  display.print(buffer);

  memset(doublebuffer, '\0', strlen(doublebuffer));
  dtostrf(outputPercentageFromSetting(potentiometerSetting),7, 2, doublebuffer);
  snprintf(buffer, LCD_COLUMNS, "Output:  %s %%", doublebuffer);
  display.setCursor(0,3);
  display.print(buffer);
  
}
void setup() {
  Serial.begin(115200);
  // Wait until serial port is opened
  while (!Serial) { delay(1); }
  Wire.begin();

  memset(buffer, '\0' , strlen(buffer));
  memset(doublebuffer, '\0' , strlen(doublebuffer));

  displaySetup();
  pressureSensorSetup();
  potentiometerSetup();


  myPID.Start(pressureSensorGet(),  // input
            OUTPUTMIN,                      // current output
            PRESSURESETPOINT);                   // setpoint
            
}



void loop() {

  double pressure = pressureSensorGet();
  double potentiometerSetting = getPotentiometerSetting(pressure);
  

  // potentiometerSet(potentiometerSetting);

  displayValuesToSerial(pressure, potentiometerSetting);
  displayValuesToLCD(pressure, potentiometerSetting);
  
  

  delay(1000);
  

} 
