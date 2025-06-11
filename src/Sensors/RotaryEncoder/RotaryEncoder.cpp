#include "RotaryEncoder.h"

RotaryEncoder::RotaryEncoder(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, threshold, channels, i2cAddress, tcaChannel, deviceIndex),
      _address(i2cAddress), _currentValue(0), _lastValue(0), 
      _buttonPressed(false), _lastButtonPressed(false) {
    type = "VisualRotaryEncoder";
    memset(&basicInfo, 0, sizeof(BasicInfo));
}

bool RotaryEncoder::begin() {
    selectTCAChannel(tcaChannel);
    
    if (!testI2CConnection()) {
        Serial.printf("DFRobot Visual Rotary Encoder not found at address 0x%02X!\n", i2cAddress);
        return false;
    }
    
    // Verify PID
    uint16_t pid = readRegister16(VISUAL_ROTARY_ENCODER_PID_MSB_REG);
    if (pid != VISUAL_ROTARY_ENCODER_PID) {
        Serial.printf("Invalid PID: 0x%04X, expected: 0x%04X\n", pid, VISUAL_ROTARY_ENCODER_PID);
        return false;
    }
    
    // Read basic device information
    refreshBasicInfo();
    
    // Initialize encoder value to 0
    setEncoderValue(0);
    
    initialized = true;
    Serial.printf("DFRobot Visual Rotary Encoder initialized successfully at address 0x%02X\n", i2cAddress);
    Serial.printf("Device info - PID: 0x%04X, VID: 0x%04X, Version: 0x%04X\n", 
                  basicInfo.PID, basicInfo.VID, basicInfo.version);
    
    return true;
}

bool RotaryEncoder::isConnected() {
    return testI2CConnection();
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
    uint8_t status = readRegister(VISUAL_ROTARY_ENCODER_KEY_STATUS_REG);
    return (status == 1);  // Button pressed when register value is 1
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
