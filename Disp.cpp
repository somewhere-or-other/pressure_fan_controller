#include "Disp.h"


// LiquidCrystal_I2C lcdSetup() {
//   // Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered)
//   return LiquidCrystal_I2C(0x27, 20, 4); // Change to (0x27,16,2) for 16x2 LCD.
// }


// void lcdUpdate(LiquidCrystal_I2C lcd, float pressure, int potentiometerSetting) {

//   lcd.setCursor(0,0); 
//   lcd.print("Makeup Fan Controller");
  
//   lcd.setCursor(1,0);
//   lcd.print("Measured Pressure Delta: ");
//   lcd.print(pressure, 2);
//   lcd.print(" pa");

//   lcd.setCursor(3,0);
//   lcd.print("Potentiometer setting: ");
//   lcd.print(potentiometerSetting, 0);
  

// }


Disp::Disp() {
  this->display = LiquidCrystal_I2C(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
}


void Disp::displayText(char *text) {
  displayText(text, LCD_DEFAULT_ROW, LCD_DEFAULT_COL);
}

void Disp::displayText(char *text, int row) {
  displayText(text, row, LCD_DEFAULT_COL);
}

void Disp::displayText(char *text, int row, int column) {
  display.setCursor(row, column);
  display.print(text);
}

void Disp::displayReadings(float pressure, float output) {
  char buffer=[LCD_COLS];

  snprintf(buffer, LCD_COLS, "Pressure: %f", pressure);
  displayText(buffer, 0);

  snprintf(buffer, LCD_COLS, "Output: %0.2f", output);
  displayText(buffer, 1);
}
