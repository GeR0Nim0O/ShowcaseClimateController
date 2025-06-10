#ifndef DFR0554DISPLAY_H
#define DFR0554DISPLAY_H

#include "Device.h"

// DFRobot DFR0554 Gravity I2C 16x2 LCD with RGB Font Display
#define DFR0554_LCD_ADDRESS 0x6B  // LCD module address
#define DFR0554_RGB_ADDRESS 0x2D  // RGB LED controller address
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

// RGB color registers
#define RGB_REG_RED        0x04
#define RGB_REG_GREEN      0x03
#define RGB_REG_BLUE       0x02
#define RGB_MODE1          0x00
#define RGB_MODE2          0x01
#define RGB_OUTPUT         0x08

class DFR0554Display : public Device {
public:
    DFR0554Display(TwoWire* wire, uint8_t address, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    std::map<String, String> readData() override;
    
    // Override pure virtual methods from Device
    std::map<String, String> getChannels() const override { return channels; }
    float getThreshold(const String& channelKey = "") const override { return threshold; }
    
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
    void scrollLeft();
    void scrollRight();
    void leftToRight();
    void rightToLeft();
    void autoscroll();
    void noAutoscroll();
    
    // RGB LED control
    void setRGB(uint8_t red, uint8_t green, uint8_t blue);
    void setColor(uint8_t red, uint8_t green, uint8_t blue);
    void setColorWhite();
    void setColorRed();
    void setColorGreen();
    void setColorBlue();
    
    // Climate display specific methods
    void displayClimateStatus(float temp, float hum, float tempSetpoint, float humSetpoint);
    void displaySystemStatus(const String& status);
    void displayError(const String& error);

private:
    uint8_t address;  // Single I2C address for both LCD and RGB
    int currentCol, currentRow;
    bool displayInitialized;
    
    void initializeDisplay();
    void initializeRGB();
    void lcdCommand(uint8_t command);
    void lcdWrite(uint8_t data);
    void rgbWrite(uint8_t reg, uint8_t data);
    bool testLCDConnection();
    bool testRGBConnection();
};

#endif // DFR0554DISPLAY_H
