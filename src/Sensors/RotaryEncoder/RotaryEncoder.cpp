#include "RotaryEncoder.h"

RotaryEncoder::RotaryEncoder(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, i2cAddress, tcaChannel, threshold, channels, deviceIndex),
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
    
    wire->beginTransmission(i2cAddress);
    wire->write(reg);
    wire->write(value);
    return (wire->endTransmission() == 0);
}

bool RotaryEncoder::writeRegister32(uint8_t reg, int32_t value) {
    selectTCAChannel(tcaChannel);
    
    wire->beginTransmission(i2cAddress);
    wire->write(reg);
    wire->write((value >> 24) & 0xFF);
    wire->write((value >> 16) & 0xFF);
    wire->write((value >> 8) & 0xFF);
    wire->write(value & 0xFF);
    return (wire->endTransmission() == 0);
}

uint8_t RotaryEncoder::readRegister(uint8_t reg) {
    selectTCAChannel(tcaChannel);
    
    wire->beginTransmission(i2cAddress);
    wire->write(reg);
    if (wire->endTransmission() != 0) {
        return 0;
    }
    
    wire->requestFrom(i2cAddress, (uint8_t)1);
    if (wire->available()) {
        return wire->read();
    }
    
    return 0;
}

int32_t RotaryEncoder::readRegister32(uint8_t reg) {
    selectTCAChannel(tcaChannel);
    
    wire->beginTransmission(i2cAddress);
    wire->write(reg);
    if (wire->endTransmission() != 0) {
        return 0;
    }
    
    wire->requestFrom(i2cAddress, (uint8_t)4);
    if (wire->available() >= 4) {
        int32_t value = (int32_t)wire->read() << 24;
        value |= (int32_t)wire->read() << 16;
        value |= (int32_t)wire->read() << 8;
        value |= wire->read();
        return value;
    }
    
    return 0;
}

bool RotaryEncoder::writeConfig() {
    uint8_t config = GCONF_INT_DATA; // Use integer data
    
    if (wrapEnabled) {
        config |= GCONF_WRAP_ENABLE;
    }
    
    // Add other configuration options as needed
    config |= GCONF_IPUP_ENABLE;  // Enable internal pull-ups
    config |= GCONF_RMOD_X1;      // X1 resolution
    
    return writeRegister(I2C_ENCODER_GCONF, config);
}

std::map<String, String> RotaryEncoder::readData() {
    std::map<String, String> data;
    data["position"] = String(getPosition());
    data["button"] = isButtonPressed() ? "1" : "0";
    return data;
}

// End of implementation
