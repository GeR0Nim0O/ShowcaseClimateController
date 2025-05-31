#include "Display.h"

Display::Display(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(wire, i2cAddress, tcaChannel, deviceName, deviceIndex),
      currentCol(0), currentRow(0), displayInitialized(false), backlightState(true) {
    type = "Display";
}

bool Display::begin() {
    Serial.println("Starting Display initialization...");
    selectTCAChannel(tcaChannel);
    
    // Test I2C connection to the PCF8574 backpack
    Serial.print("Testing I2C connection to LCD at address 0x");
    Serial.print(i2cAddress, HEX);
    Serial.print(" on TCA channel ");
    Serial.println(tcaChannel);
    
    // Try to communicate with PCF8574 - test with a simple write
    wire->beginTransmission(i2cAddress);
    wire->write(0x00); // Send zero to test communication
    uint8_t error = wire->endTransmission();
    
    if (error != 0) {
        Serial.print("LCD Display I2C communication failed with error: ");
        Serial.println(error);
        return false;
    }
    
    Serial.println("I2C communication with LCD successful");
    
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
}

bool Display::isConnected() {
    return testI2CConnection();
}

void Display::update() {
    // LCD doesn't need periodic updates
    // Keep connection alive by reading status if needed
}

void Display::initializeDisplay() {
    Serial.println("Initializing LCD display sequence...");
    selectTCAChannel(tcaChannel);
    
    // Power-on delay - HD44780 needs at least 40ms after power on
    delay(150); // Increased delay for stability
    
    // Set backlight on first and ensure PCF8574 is in known state
    Serial.println("Setting backlight and initializing PCF8574...");
    expanderWrite(LCD_BACKLIGHT);
    delay(100);
    
    // Reset sequence - put all pins low except backlight
    expanderWrite(LCD_BACKLIGHT);
    delay(10);
    
    // HD44780 initialization sequence for 4-bit mode (more robust)
    Serial.println("Starting HD44780 initialization sequence...");
    
    // First initialization - send 0x30 (8-bit mode command)
    Serial.println("First 0x30 command...");
    write4bits(0x30 | LCD_BACKLIGHT);
    delay(10); // Wait more than 4.1ms
    
    // Second initialization - send 0x30 again
    Serial.println("Second 0x30 command...");
    write4bits(0x30 | LCD_BACKLIGHT);
    delayMicroseconds(200); // Wait more than 100us
    
    // Third initialization - send 0x30 again
    Serial.println("Third 0x30 command...");
    write4bits(0x30 | LCD_BACKLIGHT);
    delayMicroseconds(200);
    
    // Set to 4-bit mode
    Serial.println("Setting 4-bit mode...");
    write4bits(0x20 | LCD_BACKLIGHT);
    delayMicroseconds(200);
    
    // Now we can use command() function for remaining setup
    // Function set: 4-bit mode, 2 lines, 5x8 dots
    Serial.println("Configuring function set...");
    command(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    delayMicroseconds(50);
    
    // Display control: display off initially
    Serial.println("Setting display off...");
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF);
    delayMicroseconds(50);
    
    // Clear display
    Serial.println("Clearing display...");
    command(LCD_CLEARDISPLAY);
    delay(3); // Clear command takes longer
    
    // Entry mode: left to right, no shift
    Serial.println("Setting entry mode...");
    command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
    delayMicroseconds(50);
    
    // Display control: display on, cursor off, blink off
    Serial.println("Turning display on...");
    command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    delayMicroseconds(50);
    
    // Return home
    Serial.println("Returning home...");
    command(LCD_RETURNHOME);
    delay(3); // Home command takes longer
    
    Serial.println("LCD initialization sequence complete");
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
