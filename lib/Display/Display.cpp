#include "Display.h"

// SSD1306 Commands
#define SSD1306_DISPLAYOFF         0xAE
#define SSD1306_DISPLAYON          0xAF
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETMULTIPLEX       0xA8
#define SSD1306_SETDISPLAYOFFSET   0xD3
#define SSD1306_SETSTARTLINE       0x40
#define SSD1306_CHARGEPUMP         0x8D
#define SSD1306_MEMORYMODE         0x20
#define SSD1306_SEGREMAP           0xA1
#define SSD1306_COMSCANDEC         0xC8
#define SSD1306_SETCOMPINS         0xDA
#define SSD1306_SETCONTRAST        0x81
#define SSD1306_SETPRECHARGE       0xD9
#define SSD1306_SETVCOMDETECT      0xDB

Display::Display(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(i2cAddress, tcaChannel, deviceName, deviceIndex),
      currentX(0), currentY(0), textSize(1), displayInitialized(false) {
    type = "Display";
}

bool Display::begin() {
    selectTCAChannel(tcaChannel);
    
    if (!testI2CConnection()) {
        Serial.println("Display not found!");
        return false;
    }
    
    initializeDisplay();
    
    initialized = true;
    displayInitialized = true;
    Serial.println("Display initialized successfully");
    
    // Show startup message
    clear();
    setCursor(0, 0);
    print("Climate Controller");
    setCursor(0, 16);
    print("Initializing...");
    display();
    
    return true;
}

bool Display::isConnected() {
    return testI2CConnection();
}

void Display::update() {
    // Periodic display updates can be implemented here
    // For now, this is handled by explicit calls to display methods
}

void Display::initializeDisplay() {
    selectTCAChannel(tcaChannel);
    
    // Initialize sequence for 128x64 OLED
    sendCommand(SSD1306_DISPLAYOFF);
    sendCommand(SSD1306_SETDISPLAYCLOCKDIV);
    sendCommand(0x80);
    sendCommand(SSD1306_SETMULTIPLEX);
    sendCommand(0x3F);
    sendCommand(SSD1306_SETDISPLAYOFFSET);
    sendCommand(0x0);
    sendCommand(SSD1306_SETSTARTLINE | 0x0);
    sendCommand(SSD1306_CHARGEPUMP);
    sendCommand(0x14);
    sendCommand(SSD1306_MEMORYMODE);
    sendCommand(0x00);
    sendCommand(SSD1306_SEGREMAP | 0x1);
    sendCommand(SSD1306_COMSCANDEC);
    sendCommand(SSD1306_SETCOMPINS);
    sendCommand(0x12);
    sendCommand(SSD1306_SETCONTRAST);
    sendCommand(0xCF);
    sendCommand(SSD1306_SETPRECHARGE);
    sendCommand(0xF1);
    sendCommand(SSD1306_SETVCOMDETECT);
    sendCommand(0x40);
    sendCommand(SSD1306_DISPLAYON);
}

void Display::clear() {
    if (!displayInitialized) return;
    
    selectTCAChannel(tcaChannel);
    
    // Clear display buffer - simplified implementation
    // In a real implementation, you would clear the frame buffer
    currentX = 0;
    currentY = 0;
}

void Display::setCursor(int x, int y) {
    currentX = x;
    currentY = y;
}

void Display::print(const String& text) {
    if (!displayInitialized) return;
    
    selectTCAChannel(tcaChannel);
    
    // Simplified text printing - in real implementation,
    // you would render text to frame buffer
    Serial.print("Display: ");
    Serial.print(text);
}

void Display::println(const String& text) {
    print(text);
    currentY += 8 * textSize; // Move to next line
    currentX = 0;
    Serial.println();
}

void Display::setTextSize(int size) {
    textSize = size;
}

void Display::display() {
    if (!displayInitialized) return;
    
    selectTCAChannel(tcaChannel);
    
    // In real implementation, this would send the frame buffer to the display
    Serial.println(" [DISPLAYED]");
}

void Display::displayClimateStatus(float temp, float hum, float tempSetpoint, float humSetpoint) {
    if (!displayInitialized) return;
    
    clear();
    
    // Line 1: Current temperature
    setCursor(0, 0);
    print("Temp: ");
    print(String(temp, 1));
    print("C (");
    print(String(tempSetpoint, 1));
    print("C)");
    
    // Line 2: Current humidity
    setCursor(0, 16);
    print("Humidity: ");
    print(String(hum, 1));
    print("% (");
    print(String(humSetpoint, 1));
    print("%)");
    
    // Line 3: Status indicators
    setCursor(0, 32);
    print("Status: ACTIVE");
    
    display();
}

void Display::displaySystemStatus(const String& status) {
    if (!displayInitialized) return;
    
    clear();
    setCursor(0, 0);
    print("System Status:");
    setCursor(0, 16);
    print(status);
    display();
}

void Display::displayError(const String& error) {
    if (!displayInitialized) return;
    
    clear();
    setCursor(0, 0);
    print("ERROR:");
    setCursor(0, 16);
    print(error);
    display();
}

void Display::sendCommand(uint8_t command) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x00); // Command mode
    Wire.write(command);
    Wire.endTransmission();
}

void Display::sendData(uint8_t data) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x40); // Data mode
    Wire.write(data);
    Wire.endTransmission();
}

// Simplified implementations for drawing methods
void Display::drawPixel(int x, int y) {
    // In real implementation, set pixel in frame buffer
}

void Display::drawLine(int x0, int y0, int x1, int y1) {
    // In real implementation, draw line in frame buffer
}

void Display::drawRect(int x, int y, int width, int height) {
    // In real implementation, draw rectangle in frame buffer
}

void Display::fillRect(int x, int y, int width, int height) {
    // In real implementation, fill rectangle in frame buffer
}
