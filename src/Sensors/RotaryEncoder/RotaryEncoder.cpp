#include "RotaryEncoder.h"

RotaryEncoder::RotaryEncoder(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, threshold, channels, i2cAddress, tcaChannel, deviceIndex),
      _address(i2cAddress), _currentValue(0), _lastValue(0), 
      _buttonPressed(false), _lastButtonPressed(false) {
    type = "VisualRotaryEncoder";
    memset(&basicInfo, 0, sizeof(BasicInfo));
}

bool RotaryEncoder::begin() {
    Serial.printf("RotaryEncoder::begin() - Starting initialization on TCA channel %d, address 0x%02X\n", tcaChannel, i2cAddress);
    
    selectTCAChannel(tcaChannel);
    
    // Basic I2C connection test
    wire->beginTransmission(i2cAddress);
    uint8_t error = wire->endTransmission();
    Serial.printf("RotaryEncoder::begin() - I2C connection test result: %d (0=success)\n", error);
    
    if (error != 0) {
        Serial.printf("DFRobot Visual Rotary Encoder not found at address 0x%02X! I2C error: %d\n", i2cAddress, error);
        return false;
    }
    Serial.printf("RotaryEncoder::begin() - I2C connection test passed\n");
    
    // Read PID register directly with detailed logging
    Serial.printf("RotaryEncoder::begin() - Reading PID register 0x%02X\n", VISUAL_ROTARY_ENCODER_PID_MSB_REG);
    wire->beginTransmission(i2cAddress);
    wire->write(VISUAL_ROTARY_ENCODER_PID_MSB_REG);
    error = wire->endTransmission();
    Serial.printf("RotaryEncoder::begin() - PID register write result: %d\n", error);
    
    if (error != 0) {
        Serial.printf("Failed to write PID register address. Error: %d\n", error);
        return false;
    }
    
    wire->requestFrom(i2cAddress, (uint8_t)2);
    if (wire->available() >= 2) {
        uint8_t pidMsb = wire->read();
        uint8_t pidLsb = wire->read();
        uint16_t pid = (pidMsb << 8) | pidLsb;
        Serial.printf("RotaryEncoder::begin() - Read PID: MSB=0x%02X, LSB=0x%02X, Combined=0x%04X\n", pidMsb, pidLsb, pid);
        Serial.printf("RotaryEncoder::begin() - Expected PID: 0x%04X\n", VISUAL_ROTARY_ENCODER_PID);
        
        if (pid != VISUAL_ROTARY_ENCODER_PID) {
            Serial.printf("PID mismatch! Got: 0x%04X, Expected: 0x%04X\n", pid, VISUAL_ROTARY_ENCODER_PID);
            // Continue anyway to test basic functionality
        }
    } else {
        Serial.printf("Failed to read PID register. Available bytes: %d\n", wire->available());
        return false;
    }
    
    // Test reading encoder count register
    Serial.printf("RotaryEncoder::begin() - Testing encoder count register\n");
    uint16_t initialCount = getEncoderValue();
    Serial.printf("RotaryEncoder::begin() - Initial encoder count: %d\n", initialCount);
    
    // Test button status register
    Serial.printf("RotaryEncoder::begin() - Testing button status register\n");
    bool buttonState = detectButtonDown();
    Serial.printf("RotaryEncoder::begin() - Initial button state: %s\n", buttonState ? "PRESSED" : "NOT PRESSED");
    
    // Read basic device information
    refreshBasicInfo();
    Serial.printf("RotaryEncoder::begin() - Device info - PID: 0x%04X, VID: 0x%04X, Version: 0x%04X, Addr: 0x%02X\n", 
                  basicInfo.PID, basicInfo.VID, basicInfo.version, basicInfo.i2cAddr);
    
    // Set a known encoder value for testing
    Serial.printf("RotaryEncoder::begin() - Setting encoder value to 100 for testing\n");
    setEncoderValue(100);
    uint16_t testCount = getEncoderValue();
    Serial.printf("RotaryEncoder::begin() - Encoder value after setting to 100: %d\n", testCount);
    
    initialized = true;
    Serial.printf("DFRobot Visual Rotary Encoder initialization completed at address 0x%02X\n", i2cAddress);
    
    return true;
    
    return true;
}

bool RotaryEncoder::isConnected() {
    bool connected = testI2CConnection();
    Serial.printf("[RotaryEncoder] Connection test: %s\n", connected ? "CONNECTED" : "DISCONNECTED");
    return connected;
}

void RotaryEncoder::update() {
    selectTCAChannel(tcaChannel);
    
    // Store previous values
    _lastValue = _currentValue;
    _lastButtonPressed = _buttonPressed;
    
    // Read current encoder value
    _currentValue = getEncoderValue();
    
    // Read button status
    _buttonPressed = detectButtonDown();
    
    // Log changes
    if (_currentValue != _lastValue) {
        int16_t change = (int16_t)(_currentValue - _lastValue);
        Serial.printf("[RotaryEncoder] Value changed: %d -> %d (change: %d)\n", 
                      _lastValue, _currentValue, change);
    }
    
    if (_buttonPressed != _lastButtonPressed) {
        Serial.printf("[RotaryEncoder] Button %s\n", 
                      _buttonPressed ? "PRESSED" : "RELEASED");
    }
}

uint16_t RotaryEncoder::getEncoderValue() {
    return readRegister16(VISUAL_ROTARY_ENCODER_COUNT_MSB_REG);
}

void RotaryEncoder::setEncoderValue(uint16_t value) {
    selectTCAChannel(tcaChannel);
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;  // MSB
    data[1] = value & 0xFF;         // LSB
    writeRegisterMulti(VISUAL_ROTARY_ENCODER_COUNT_MSB_REG, data, 2);
    _currentValue = value;
}

bool RotaryEncoder::detectButtonDown() {
    selectTCAChannel(tcaChannel);
    
    uint8_t status = readRegister(VISUAL_ROTARY_ENCODER_KEY_STATUS_REG);
    
    if (status & 0x01) {  // Button pressed when bit 0 is set
        // Clear the button status by writing 0 (following DFRobot library pattern)
        uint8_t clearStatus = 0x00;
        writeRegister(VISUAL_ROTARY_ENCODER_KEY_STATUS_REG, clearStatus);
        return true;
    }
    
    return false;
}

uint8_t RotaryEncoder::getGainCoefficient() {
    return readRegister(VISUAL_ROTARY_ENCODER_GAIN_REG);
}

void RotaryEncoder::setGainCoefficient(uint8_t gainValue) {
    // Ensure gain value is within valid range (1-51)
    if (gainValue < 1) gainValue = 1;
    if (gainValue > 51) gainValue = 51;
    
    writeRegister(VISUAL_ROTARY_ENCODER_GAIN_REG, gainValue);
}

void RotaryEncoder::refreshBasicInfo() {
    selectTCAChannel(tcaChannel);
    
    basicInfo.PID = readRegister16(VISUAL_ROTARY_ENCODER_PID_MSB_REG);
    basicInfo.VID = readRegister16(VISUAL_ROTARY_ENCODER_VID_MSB_REG);
    basicInfo.version = readRegister16(VISUAL_ROTARY_ENCODER_VERSION_MSB_REG);
    basicInfo.i2cAddr = readRegister(VISUAL_ROTARY_ENCODER_ADDR_REG);
}

bool RotaryEncoder::writeRegister(uint8_t reg, uint8_t value) {
    selectTCAChannel(tcaChannel);
    
    wire->beginTransmission(_address);
    wire->write(reg);
    wire->write(value);
    return (wire->endTransmission() == 0);
}

bool RotaryEncoder::writeRegisterMulti(uint8_t reg, uint8_t* pBuf, size_t size) {
    selectTCAChannel(tcaChannel);
    
    wire->beginTransmission(_address);
    wire->write(reg);
    for (size_t i = 0; i < size; i++) {
        wire->write(pBuf[i]);
    }
    return (wire->endTransmission() == 0);
}

uint8_t RotaryEncoder::readRegister(uint8_t reg) {
    selectTCAChannel(tcaChannel);
    
    wire->beginTransmission(_address);
    wire->write(reg);
    if (wire->endTransmission() != 0) {
        return 0;
    }
    
    wire->requestFrom(_address, (uint8_t)1);
    if (wire->available()) {
        return wire->read();
    }
    
    return 0;
}

bool RotaryEncoder::readRegisterMulti(uint8_t reg, uint8_t* pBuf, size_t size) {
    selectTCAChannel(tcaChannel);
    
    wire->beginTransmission(_address);
    wire->write(reg);
    if (wire->endTransmission() != 0) {
        return false;
    }
    
    wire->requestFrom(_address, (uint8_t)size);
    if (wire->available() >= size) {
        for (size_t i = 0; i < size; i++) {
            pBuf[i] = wire->read();
        }
        return true;
    }
    
    return false;
}

uint16_t RotaryEncoder::readRegister16(uint8_t reg) {
    uint8_t data[2];
    if (readRegisterMulti(reg, data, 2)) {
        return (uint16_t)(data[0] << 8) | data[1];
    }
    return 0;
}

std::map<String, String> RotaryEncoder::readData() {
    std::map<String, String> data;
    
    // Update current values
    _currentValue = getEncoderValue();
    _buttonPressed = detectButtonDown();
    
    // Calculate position change since last read
    int16_t positionChange = (int16_t)(_currentValue - _lastValue);
    
    data["encoder_value"] = String(_currentValue);
    data["encoder_change"] = String(positionChange);
    data["button"] = _buttonPressed ? "1" : "0";
    data["button_pressed"] = (_buttonPressed && !_lastButtonPressed) ? "1" : "0";
    data["gain"] = String(getGainCoefficient());
    
    // Store current values as last values for next read
    _lastValue = _currentValue;
    _lastButtonPressed = _buttonPressed;
    
    return data;
}

// End of implementation
