#include "RotaryEncoder.h"

#define I2C_ENCODER_DEFAULT_ADDRESS 0x61

RotaryEncoder::RotaryEncoder(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(wire, i2cAddress, tcaChannel, deviceName, deviceIndex),
      position(0), lastPosition(0), minValue(-2147483648), maxValue(2147483647),
      stepValue(1), wrapEnabled(false), floatData(false), rgbEncoder(false),
      buttonPressed(false), lastButtonPressed(false), buttonHeld(false),
      buttonPressTime(0) {
    type = "I2C_Encoder";
}

bool RotaryEncoder::begin() {
    selectTCAChannel(tcaChannel);
    
    if (!testI2CConnection()) {
        Serial.println("I2C Encoder not found!");
        return false;
    }
    
    // Reset the encoder
    writeRegister(I2C_ENCODER_GCONF, GCONF_RESET);
    delay(10);
    
    // Configure encoder
    if (!writeConfig()) {
        Serial.println("Failed to configure I2C Encoder");
        return false;
    }
    
    // Set initial values
    setPosition(0);
    setMinMax(-1000, 1000);
    setStepSize(1);
    
    initialized = true;
    Serial.println("I2C Encoder initialized successfully");
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
