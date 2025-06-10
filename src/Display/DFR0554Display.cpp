#include "DFR0554Display.h"
#include "I2CHandler.h"

DFR0554Display::DFR0554Display(TwoWire* wire, uint8_t address, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, threshold, channels, address, tcaChannel, deviceIndex),
      lcdAddress(DFR0554_LCD_ADDRESS),  // Use LCD address 0x3E
      rgbAddress(DFR0554_RGB_ADDRESS),  // Use RGB address 0x2D  
      currentCol(0), currentRow(0), displayInitialized(false) {
    type = "DFR0554Display";
}

bool DFR0554Display::begin() {
    I2CHandler::selectTCA(getTCAChannel());
    
    // Test LCD communication
    if (!testLCDConnection()) {
        Serial.println("DFR0554: LCD communication failed");
        initialized = false;
        displayInitialized = false;
        return false;
    }
    
    // Test RGB communication  
    if (!testRGBConnection()) {
        Serial.println("DFR0554: RGB LED communication failed");
        initialized = false;
        displayInitialized = false;
        return false;
    }
    
    // Initialize LCD
    initializeDisplay();
    
    // Initialize RGB
    initializeRGB();
    
    // Set default white color
    setColorWhite();
    
    initialized = true;
    displayInitialized = true;
    Serial.println("DFR0554 Display initialized");
    
    // Show startup message
    clear();
    setCursor(0, 0);
    print("Climate Control");
    setCursor(0, 1);
    print("Starting...");
    delay(2000);
    
    return true;
}

bool DFR0554Display::testLCDConnection() {
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(lcdAddress);
    return (wire->endTransmission() == 0);
}

bool DFR0554Display::testRGBConnection() {
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(rgbAddress);
    return (wire->endTransmission() == 0);
}

bool DFR0554Display::isConnected() {
    return testLCDConnection() && testRGBConnection();
}

void DFR0554Display::update() {
    // Display doesn't need periodic updates
}

std::map<String, String> DFR0554Display::readData() {
    std::map<String, String> result;
    // Display doesn't have sensor data to read
    for (const auto& channel : channels) {
        result[channel.first] = "0";
    }
    return result;
}

void DFR0554Display::initializeDisplay() {
    I2CHandler::selectTCA(getTCAChannel());
    
    delay(50); // Power-up delay
    
    // Function set: 4-bit mode, 2 lines, 5x8 dots (following official library)
    lcdCommand(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    delay(5);
    
    // Second try (following HD44780 initialization)
    lcdCommand(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    delay(5);
    
    // Third try
    lcdCommand(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    
    // Display control: display on, cursor off, blink off
    lcdCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    
    // Clear display
    lcdCommand(LCD_CLEARDISPLAY);
    delay(2);
    
    // Entry mode: left to right, no shift
    lcdCommand(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
    
    // Return home
    lcdCommand(LCD_RETURNHOME);
    delay(2);
}

void DFR0554Display::initializeRGB() {
    I2CHandler::selectTCA(getTCAChannel());
    
    // Initialize RGB controller
    rgbWrite(RGB_MODE1, 0x00);
    rgbWrite(RGB_MODE2, 0x00);
    rgbWrite(RGB_OUTPUT, 0xFF);
}

void DFR0554Display::lcdCommand(uint8_t command) {
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(address);
    wire->write(0x80); // Command mode
    wire->write(command);
    wire->endTransmission();
}

void DFR0554Display::lcdWrite(uint8_t data) {
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(address);
    wire->write(0x40); // Data mode
    wire->write(data);
    wire->endTransmission();
}

void DFR0554Display::rgbWrite(uint8_t reg, uint8_t data) {
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(address);
    wire->write(reg);
    wire->write(data);
    wire->endTransmission();
}

void DFR0554Display::clear() {
    if (!displayInitialized) return;
    
    lcdCommand(LCD_CLEARDISPLAY);
    delay(2);
    currentCol = 0;
    currentRow = 0;
}

void DFR0554Display::home() {
    if (!displayInitialized) return;
    
    lcdCommand(LCD_RETURNHOME);
    delay(2);
    currentCol = 0;
    currentRow = 0;
}

void DFR0554Display::setCursor(int col, int row) {
    if (!displayInitialized) return;
    if (col >= LCD_COLS || row >= LCD_ROWS) return;
    
    currentCol = col;
    currentRow = row;
    
    uint8_t address = (row == 0) ? 0x00 : 0x40;
    address += col;
    lcdCommand(LCD_SETDDRAMADDR | address);
}

void DFR0554Display::print(const String& text) {
    if (!displayInitialized) return;
    
    for (int i = 0; i < text.length(); i++) {
        if (currentCol >= LCD_COLS) {
            // Auto-wrap to next line
            currentRow++;
            currentCol = 0;
            if (currentRow >= LCD_ROWS) {
                currentRow = 0; // Wrap to first line
            }
            setCursor(currentCol, currentRow);
        }
        
        lcdWrite(text[i]);
        currentCol++;
    }
}

void DFR0554Display::print(char c) {
    if (!displayInitialized) return;
    
    if (currentCol >= LCD_COLS) {
        currentRow++;
        currentCol = 0;
        if (currentRow >= LCD_ROWS) {
            currentRow = 0;
        }
        setCursor(currentCol, currentRow);
    }
    
    lcdWrite(c);
    currentCol++;
}

void DFR0554Display::print(int value) {
    print(String(value));
}

void DFR0554Display::print(float value, int decimals) {
    print(String(value, decimals));
}

void DFR0554Display::displayOn() {
    lcdCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
}

void DFR0554Display::displayOff() {
    lcdCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF);
}

void DFR0554Display::cursorOn() {
    lcdCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKOFF);
}

void DFR0554Display::cursorOff() {
    lcdCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
}

void DFR0554Display::blinkOn() {
    lcdCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKON);
}

void DFR0554Display::blinkOff() {
    lcdCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
}

void DFR0554Display::scrollLeft() {
    lcdCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void DFR0554Display::scrollRight() {
    lcdCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void DFR0554Display::leftToRight() {
    lcdCommand(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
}

void DFR0554Display::rightToLeft() {
    lcdCommand(LCD_ENTRYMODESET | LCD_ENTRYRIGHT | LCD_ENTRYSHIFTDECREMENT);
}

void DFR0554Display::autoscroll() {
    lcdCommand(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTINCREMENT);
}

void DFR0554Display::noAutoscroll() {
    lcdCommand(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
}

void DFR0554Display::setRGB(uint8_t red, uint8_t green, uint8_t blue) {
    rgbWrite(RGB_REG_RED, red);
    rgbWrite(RGB_REG_GREEN, green);
    rgbWrite(RGB_REG_BLUE, blue);
}

void DFR0554Display::setColor(uint8_t red, uint8_t green, uint8_t blue) {
    setRGB(red, green, blue);
}

void DFR0554Display::setColorWhite() {
    setRGB(255, 255, 255);
}

void DFR0554Display::setColorRed() {
    setRGB(255, 0, 0);
}

void DFR0554Display::setColorGreen() {
    setRGB(0, 255, 0);
}

void DFR0554Display::setColorBlue() {
    setRGB(0, 0, 255);
}

void DFR0554Display::displayClimateStatus(float temp, float hum, float tempSetpoint, float humSetpoint) {
    if (!displayInitialized) return;
    
    clear();
    
    // Line 1: Current values
    setCursor(0, 0);
    print("T:" + String(temp, 1) + " RH:" + String(hum, 0) + "%");
    
    // Line 2: Setpoints
    setCursor(0, 1);
    print("S:" + String(tempSetpoint, 1) + " S:" + String(humSetpoint, 0) + "%");
}

void DFR0554Display::displaySystemStatus(const String& status) {
    if (!displayInitialized) return;
    
    clear();
    setCursor(0, 0);
    print("System Status:");
    setCursor(0, 1);
    
    // Truncate status if too long
    String truncatedStatus = status;
    if (truncatedStatus.length() > LCD_COLS) {
        truncatedStatus = truncatedStatus.substring(0, LCD_COLS);
    }
    print(truncatedStatus);
}

void DFR0554Display::displayError(const String& error) {
    if (!displayInitialized) return;
    
    clear();
    setCursor(0, 0);
    print("ERROR:");
    setCursor(0, 1);
    
    // Truncate error if too long
    String truncatedError = error;
    if (truncatedError.length() > LCD_COLS) {
        truncatedError = truncatedError.substring(0, LCD_COLS);
    }
    print(truncatedError);
    
    // Set red color for error
    setColorRed();
}
