#include "Display.h"

Display::Display(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(wire, i2cAddress, tcaChannel, deviceName, deviceIndex),
      currentCol(0), currentRow(0), displayInitialized(false), backlightState(true) {
    type = "LCD_Display";
}

bool Display::begin() {
    selectTCAChannel(tcaChannel);
    
    if (!testI2CConnection()) {
        Serial.println("LCD Display not found!");
        return false;
    }
    
    initializeDisplay();
    
    initialized = true;
    displayInitialized = true;
    Serial.println("LCD Display initialized successfully");
    
    // Show startup message
    clear();
    setCursor(0, 0);
    print("Climate Control");
    setCursor(0, 1);
    print("Starting...");
    delay(2000);
    
    return true;
}

bool Display::isConnected() {
    return testI2CConnection();
}

void Display::update() {
    // LCD doesn't need periodic updates
    // Keep connection alive by reading status if needed
}

void Display::initializeDisplay() {
    selectTCAChannel(tcaChannel);
    
    // Initialize LCD in 4-bit mode
    delay(50); // Wait for LCD to power up
    
    // Set backlight on
    expanderWrite(LCD_BACKLIGHT);
    delay(1000);
    
    // Start in 8-bit mode, then switch to 4-bit
    write4bits(0x03 << 4);
    delayMicroseconds(4500);
    
    write4bits(0x03 << 4);
    delayMicroseconds(4500);
    
    write4bits(0x03 << 4);
    delayMicroseconds(150);
    
    // Set to 4-bit mode
    write4bits(0x02 << 4);
    
    // Function set: 4-bit mode, 2 lines, 5x8 dots
    command(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    
    // Display control: display on, cursor off, blink off
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    
    // Clear display
    clear();
    
    // Entry mode: left to right
    command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
    
    home();
}

void Display::clear() {
    if (!displayInitialized) return;
    
    command(LCD_CLEARDISPLAY);
    delayMicroseconds(2000);
    currentCol = 0;
    currentRow = 0;
}

void Display::home() {
    if (!displayInitialized) return;
    
    command(LCD_RETURNHOME);
    delayMicroseconds(2000);
    currentCol = 0;
    currentRow = 0;
}

void Display::setCursor(int col, int row) {
    if (!displayInitialized) return;
    if (col >= LCD_COLS || row >= LCD_ROWS) return;
    
    currentCol = col;
    currentRow = row;
    
    uint8_t address = (row == 0) ? 0x00 : 0x40;
    address += col;
    command(LCD_SETDDRAMADDR | address);
}

void Display::print(const String& text) {
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
        
        writeChar(text[i]);
        currentCol++;
    }
}

void Display::print(char c) {
    if (!displayInitialized) return;
    
    if (currentCol >= LCD_COLS) {
        currentRow++;
        currentCol = 0;
        if (currentRow >= LCD_ROWS) {
            currentRow = 0;
        }
        setCursor(currentCol, currentRow);
    }
    
    writeChar(c);
    currentCol++;
}

void Display::print(int value) {
    print(String(value));
}

void Display::print(float value, int decimals) {
    print(String(value, decimals));
}

void Display::displayOn() {
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
}

void Display::displayOff() {
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF);
}

void Display::cursorOn() {
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKOFF);
}

void Display::cursorOff() {
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
}

void Display::blinkOn() {
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKON);
}

void Display::blinkOff() {
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
}

void Display::backlightOn() {
    backlightState = true;
    expanderWrite(LCD_BACKLIGHT);
}

void Display::backlightOff() {
    backlightState = false;
    expanderWrite(0);
}

void Display::scrollLeft() {
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void Display::scrollRight() {
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void Display::leftToRight() {
    command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
}

void Display::rightToLeft() {
    command(LCD_ENTRYMODESET | LCD_ENTRYRIGHT | LCD_ENTRYSHIFTDECREMENT);
}

void Display::autoscroll() {
    command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTINCREMENT);
}

void Display::noAutoscroll() {
    command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
}

void Display::displayClimateStatus(float temp, float hum, float tempSetpoint, float humSetpoint) {
    if (!displayInitialized) return;
    
    clear();
    
    // Line 1: Temperature info
    setCursor(0, 0);
    print("T:");
    print(temp, 1);
    print("C S:");
    print(tempSetpoint, 1);
    
    // Line 2: Humidity info
    setCursor(0, 1);
    print("H:");
    print(hum, 1);
    print("% S:");
    print(humSetpoint, 1);
}

void Display::displaySystemStatus(const String& status) {
    if (!displayInitialized) return;
    
    clear();
    setCursor(0, 0);
    print("System:");
    setCursor(0, 1);
    
    // Truncate status if too long
    String truncatedStatus = status;
    if (truncatedStatus.length() > LCD_COLS) {
        truncatedStatus = truncatedStatus.substring(0, LCD_COLS);
    }
    print(truncatedStatus);
}

void Display::displayError(const String& error) {
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
}

void Display::send(uint8_t value, uint8_t mode) {
    uint8_t highnib = value & 0xF0;
    uint8_t lownib = (value << 4) & 0xF0;
    write4bits(highnib | mode);
    write4bits(lownib | mode);
}

void Display::write4bits(uint8_t value) {
    expanderWrite(value);
    pulseEnable(value);
}

void Display::expanderWrite(uint8_t data) {
    selectTCAChannel(tcaChannel);
    
    if (backlightState) {
        data |= LCD_BACKLIGHT;
    }
    
    Wire.beginTransmission(i2cAddress);
    Wire.write(data);
    Wire.endTransmission();
}

void Display::pulseEnable(uint8_t data) {
    expanderWrite(data | LCD_EN);
    delayMicroseconds(1);
    expanderWrite(data & ~LCD_EN);
    delayMicroseconds(50);
}

void Display::command(uint8_t value) {
    send(value, 0);
}

void Display::writeChar(uint8_t value) {
    send(value, LCD_RS);
}
