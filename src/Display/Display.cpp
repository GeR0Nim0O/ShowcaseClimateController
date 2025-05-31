#include "Display.h"

Display::Display(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(wire, i2cAddress, tcaChannel, deviceName, deviceIndex),
      currentCol(0), currentRow(0), displayInitialized(false), backlightState(true) {
    type = "Display";
}

bool Display::begin() {
    Serial.println("Starting Display initialization...");
    selectTCAChannel(tcaChannel);
      // Test I2C connection to the PCF8574T backpack
    Serial.print("Testing I2C connection to PCF8574T LCD at address 0x");
    Serial.print(i2cAddress, HEX);
    Serial.print(" on TCA channel ");
    Serial.println(tcaChannel);
    
    // PCF8574T specific testing - test multiple values
    Serial.println("Testing PCF8574T communication patterns...");
    
    // Test pattern 1: All zeros
    wire->beginTransmission(i2cAddress);
    wire->write(0x00);
    uint8_t error1 = wire->endTransmission();
    
    // Test pattern 2: Backlight only
    wire->beginTransmission(i2cAddress);
    wire->write(LCD_BACKLIGHT);
    uint8_t error2 = wire->endTransmission();
    
    // Test pattern 3: All high
    wire->beginTransmission(i2cAddress);
    wire->write(0xFF);
    uint8_t error3 = wire->endTransmission();
      if (error1 != 0 || error2 != 0 || error3 != 0) {
        Serial.print("PCF8574T LCD communication failed. Errors: ");
        Serial.print(error1);
        Serial.print(", ");
        Serial.print(error2);
        Serial.print(", ");
        Serial.println(error3);
        initialized = false;
        displayInitialized = false;
        return false;
    }
    
    Serial.println("PCF8574T communication tests successful");
    
    // Try initialization with retry mechanism
    int retryCount = 0;
    const int maxRetries = 3;
    
    while (retryCount < maxRetries) {
        Serial.print("LCD initialization attempt ");
        Serial.print(retryCount + 1);
        Serial.print(" of ");
        Serial.println(maxRetries);
          try {
            initializeDisplay();
            
            // Test if LCD is working by sending a simple command
            delay(10);
            command(LCD_CLEARDISPLAY);
            delay(5);
            
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
            
        } catch (...) {
            Serial.print("Exception occurred during LCD initialization attempt ");
            Serial.println(retryCount + 1);
        }
        
        retryCount++;
        if (retryCount < maxRetries) {
            Serial.println("Retrying LCD initialization...");
            delay(500); // Wait before retry
        }
    }
    
    Serial.println("LCD Display initialization failed after all attempts");
    initialized = false;
    displayInitialized = false;
    return false;
}

bool Display::isConnected() {
    return testI2CConnection();
}

void Display::update() {
    // LCD doesn't need periodic updates
    // Keep connection alive by reading status if needed
}

void Display::initializeDisplay() {
    Serial.println("Initializing LCD display sequence for PCF8574T...");
    selectTCAChannel(tcaChannel);
    
    // PCF8574T specific initialization - longer delays for stability
    delay(250); // Extended power-up delay for PCF8574T
    
    // First, ensure all pins are in known state
    Serial.println("Resetting PCF8574T pins...");
    expanderWrite(0x00); // All pins low
    delay(50);
    
    // Set backlight on and establish baseline
    Serial.println("Setting backlight on...");
    expanderWrite(LCD_BACKLIGHT);
    delay(150); // Allow backlight to stabilize
    
    // PCF8574T requires more careful timing for HD44780 initialization
    Serial.println("Starting HD44780 initialization sequence for PCF8574T...");
    
    // Initial reset sequence - send 0x30 three times with proper timing
    Serial.println("Sending initial reset commands...");
    
    // First 0x30 command - must wait >15ms after power on
    write4bits(0x30);
    delay(20); // Wait more than 15ms
    
    // Second 0x30 command - must wait >4.1ms
    write4bits(0x30);
    delay(10); // Wait more than 4.1ms
    
    // Third 0x30 command - must wait >100us
    write4bits(0x30);
    delay(2); // Wait more than 100us
    
    // Now switch to 4-bit mode
    Serial.println("Setting 4-bit mode...");
    write4bits(0x20);
    delay(2); // Allow mode switch to complete
    
    // From here on, use normal 4-bit commands
    // Function set: 4-bit mode, 2 lines, 5x8 dots
    Serial.println("Configuring function set...");
    command(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    delay(2);
    
    // Display control: display off initially
    Serial.println("Setting display control off...");
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF);
    delay(2);
    
    // Clear display
    Serial.println("Clearing display...");
    command(LCD_CLEARDISPLAY);
    delay(5); // Clear command needs more time
    
    // Entry mode: left to right, no shift
    Serial.println("Setting entry mode...");
    command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
    delay(2);
    
    // Finally turn display on
    Serial.println("Turning display on...");
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    delay(2);
    
    // Return home
    Serial.println("Returning home...");
    command(LCD_RETURNHOME);
    delay(5); // Home command needs more time
    
    Serial.println("PCF8574T LCD initialization sequence complete");
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
    // Ensure backlight state is preserved
    if (backlightState) {
        value |= LCD_BACKLIGHT;
    }
    expanderWrite(value);
    pulseEnable(value);
}

void Display::expanderWrite(uint8_t data) {
    selectTCAChannel(tcaChannel);
    
    // Ensure backlight state is always preserved
    if (backlightState) {
        data |= LCD_BACKLIGHT;
    }
    
    wire->beginTransmission(i2cAddress);
    wire->write(data);
    uint8_t error = wire->endTransmission();
    
    if (error != 0) {
        Serial.print("Warning: I2C transmission error during LCD write: ");
        Serial.println(error);
    }
}

void Display::pulseEnable(uint8_t data) {
    // Enable pulse must be at least 450ns wide for HD44780
    expanderWrite(data | LCD_EN);
    delayMicroseconds(2); // Increased from 1us for better reliability
    expanderWrite(data & ~LCD_EN);
    delayMicroseconds(100); // Increased from 50us for better reliability
}

void Display::command(uint8_t value) {
    send(value, 0);
}

void Display::writeChar(uint8_t value) {
    send(value, LCD_RS);
}
