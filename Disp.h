#ifdef LCD_H
#define LCD_H

#define LCD_I2C_ADDR 0x27
#define LCD_ROWS 4
#define LCD_COLS 20

#define LCD_DEFAULT_ROW 0 //row to use when not defined
#define LCD_DEFAULT_COL 0 //column to use when not defined

// Include the libraries:
// LiquidCrystal_I2C.h: https://github.com/johnrickman/LiquidCrystal_I2C
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD

// LiquidCrystal_I2C lcdSetup();
// void lcdUpdate(LiquidCrystal_I2C lcd, float pressure, int potentiometerSetting);






class lcd {
  
  private:
    LiquidCrystal_I2C display;
    
  public:
    lcd();
    void displayText(char *text);
    void displayText(char *text, int row);
    void displayText(char *text, int row, int column);
    void displayReadings(float pressure, float output);
    
};



#endif
