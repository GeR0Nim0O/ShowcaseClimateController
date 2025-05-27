#ifndef DISPLAY_H
#define DISPLAY_H

#include "Device.h"

#define LCD_DEFAULT_ADDRESS 0x27  // Common I2C address for PCF8574 LCD backpack
#define LCD_COLS 16
#define LCD_ROWS 2

// LCD Commands (HD44780 compatible)
#define LCD_CLEARDISPLAY   0x01
#define LCD_RETURNHOME     0x02
#define LCD_ENTRYMODESET   0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT    0x10
#define LCD_FUNCTIONSET    0x20
#define LCD_SETCGRAMADDR   0x40
#define LCD_SETDDRAMADDR   0x80

// Flags for display entry mode
#define LCD_ENTRYRIGHT     0x00
#define LCD_ENTRYLEFT      0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Flags for display on/off control
#define LCD_DISPLAYON      0x04
#define LCD_DISPLAYOFF     0x00
#define LCD_CURSORON       0x02
#define LCD_CURSOROFF      0x00
#define LCD_BLINKON        0x01
#define LCD_BLINKOFF       0x00

// Flags for display/cursor shift
#define LCD_DISPLAYMOVE    0x08
#define LCD_CURSORMOVE     0x00
#define LCD_MOVERIGHT      0x04
#define LCD_MOVELEFT       0x00

// Flags for function set
#define LCD_8BITMODE       0x10
#define LCD_4BITMODE       0x00
#define LCD_2LINE          0x08
#define LCD_1LINE          0x00
#define LCD_5x10DOTS       0x04
#define LCD_5x8DOTS        0x00

// PCF8574 pin mapping for LCD
#define LCD_RS     0x01  // Register select
#define LCD_EN     0x04  // Enable
#define LCD_D4     0x10  // Data 4
#define LCD_D5     0x20  // Data 5
#define LCD_D6     0x40  // Data 6
#define LCD_D7     0x80  // Data 7
#define LCD_BACKLIGHT 0x08  // Backlight control

class Display : public Device {
public:
    Display(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    
    // Display methods
    void clear();
    void home();
    void setCursor(int col, int row);
    void print(const String& text);
    void print(char c);
    void print(int value);
    void print(float value, int decimals = 2);
    
    // Display control
    void displayOn();
    void displayOff();
    void cursorOn();
    void cursorOff();
    void blinkOn();
    void blinkOff();
    void backlightOn();
    void backlightOff();
    void scrollLeft();
    void scrollRight();
    void leftToRight();
    void rightToLeft();
    void autoscroll();
    void noAutoscroll();
    
    // Climate display specific methods
    void displayClimateStatus(float temp, float hum, float tempSetpoint, float humSetpoint);
    void displaySystemStatus(const String& status);
    void displayError(const String& error);

private:
    int currentCol, currentRow;
    bool displayInitialized;
    bool backlightState;
    
    void initializeDisplay();
    void sendCommand(uint8_t command);
    void sendData(uint8_t data);
};

#endif // DISPLAY_H
