#ifndef DISPLAY_H
#define DISPLAY_H

#include "Device.h"

#define SSD1306_DEFAULT_ADDRESS 0x3C

class Display : public Device {
public:
    Display(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    
    // Display methods
    void clear();
    void setCursor(int x, int y);
    void print(const String& text);
    void println(const String& text);
    void setTextSize(int size);
    void drawPixel(int x, int y);
    void drawLine(int x0, int y0, int x1, int y1);
    void drawRect(int x, int y, int width, int height);
    void fillRect(int x, int y, int width, int height);
    void display();
    
    // Climate display specific methods
    void displayClimateStatus(float temp, float hum, float tempSetpoint, float humSetpoint);
    void displaySystemStatus(const String& status);
    void displayError(const String& error);

private:
    int currentX, currentY;
    int textSize;
    bool displayInitialized;
    
    void initializeDisplay();
    void sendCommand(uint8_t command);
    void sendData(uint8_t data);
};

#endif // DISPLAY_H
