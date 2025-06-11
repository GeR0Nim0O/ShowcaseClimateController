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
    // Read current position
    position = readRegister32(I2C_ENCODER_CVAL);
    
    // Read button status
    uint8_t status = readRegister(I2C_ENCODER_ESTATUS);
    lastButtonPressed = buttonPressed;
    buttonPressed = (status & 0x01) != 0; // Button status bit
    
    // Track button press timing
    if (buttonPressed && !lastButtonPressed) {
        buttonPressTime = millis();
    }
    
    if (!buttonPressed) {
        buttonPressTime = 0;
    }
}

int32_t RotaryEncoder::getPosition() {
    return readRegister32(I2C_ENCODER_CVAL);
}

void RotaryEncoder::setPosition(int32_t pos) {
    writeRegister32(I2C_ENCODER_CVAL, pos);
    position = pos;
    lastPosition = pos;
}

int32_t RotaryEncoder::getPositionChange() {
    int32_t currentPos = getPosition();
    int32_t change = currentPos - lastPosition;
    lastPosition = currentPos;
    return change;
}

bool RotaryEncoder::isButtonPressed() {
    uint8_t status = readRegister(I2C_ENCODER_ESTATUS);
    return (status & 0x01) != 0;
}

bool RotaryEncoder::wasButtonPressed() {
    if (buttonPressed && !lastButtonPressed) {
        return true;
    }
    return false;
}

bool RotaryEncoder::isButtonHeld(unsigned long holdTime) {
    if (buttonPressed && buttonPressTime > 0) {
        return (millis() - buttonPressTime) >= holdTime;
    }
    return false;
}

void RotaryEncoder::setMinMax(int32_t minVal, int32_t maxVal) {
    minValue = minVal;
    maxValue = maxVal;
    
    writeRegister32(I2C_ENCODER_CMIN, minVal);
    writeRegister32(I2C_ENCODER_CMAX, maxVal);
    
    // Constrain current position
    int32_t currentPos = getPosition();
    if (currentPos < minValue) setPosition(minValue);
    if (currentPos > maxValue) setPosition(maxValue);
}

void RotaryEncoder::setStepSize(int32_t step) {
    stepValue = step;
    writeRegister32(I2C_ENCODER_ISTEP, step);
}

void RotaryEncoder::enableWrap(bool enable) {
    wrapEnabled = enable;
    writeConfig();
}

void RotaryEncoder::setDirection(bool clockwiseIncrement) {
    // Update configuration and rewrite
    writeConfig();
}

void RotaryEncoder::setLED(uint8_t red, uint8_t green, uint8_t blue) {
    if (rgbEncoder) {
        writeRegister(I2C_ENCODER_RLED, red);
        writeRegister(I2C_ENCODER_GLED, green);
        writeRegister(I2C_ENCODER_BLED, blue);
    }
}

void RotaryEncoder::setLEDFade(uint8_t fade) {
    if (rgbEncoder) {
        writeRegister(I2C_ENCODER_FADERGB, fade);
    }
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
