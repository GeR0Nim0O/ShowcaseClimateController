#include "PCF8574_GPIO.h"

// Pin definitions for climate control (matching main.cpp)
#define FAN_INTERIOR  0
#define FAN_EXTERIOR  1
#define HUMIDIFY      2
#define DEHUMIDIFY    3
#define TEMP_ENABLE   4
#define COOL          5
#define HEAT          6
#define SPARE2        7

PCF8574_GPIO::PCF8574_GPIO(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(i2cAddress, tcaChannel, deviceName, deviceIndex), currentState(0x00) {
    type = "GPIO_Expander";
}

bool PCF8574_GPIO::begin() {
    selectTCAChannel(tcaChannel);
    
    if (!testI2CConnection()) {
        Serial.println("PCF8574 GPIO Expander not found!");
        return false;
    }
    
    // Initialize all pins to LOW
    currentState = 0x00;
    writeToDevice(currentState);
    
    initialized = true;
    Serial.println("PCF8574 GPIO Expander initialized successfully");
    return true;
}

bool PCF8574_GPIO::isConnected() {
    return testI2CConnection();
}

void PCF8574_GPIO::update() {
    // Read current state from device
    currentState = readFromDevice();
}

bool PCF8574_GPIO::digitalWrite(uint8_t pin, bool state) {
    if (pin > 7) return false;
    
    if (state) {
        currentState |= (1 << pin);
    } else {
        currentState &= ~(1 << pin);
    }
    
    return writeToDevice(currentState);
}

bool PCF8574_GPIO::digitalRead(uint8_t pin) {
    if (pin > 7) return false;
    
    uint8_t data = readFromDevice();
    return (data & (1 << pin)) != 0;
}

bool PCF8574_GPIO::write8(uint8_t value) {
    currentState = value;
    return writeToDevice(currentState);
}

uint8_t PCF8574_GPIO::read8() {
    return readFromDevice();
}

// Climate control specific methods
bool PCF8574_GPIO::setTemperatureEnable(bool enable) {
    return digitalWrite(TEMP_ENABLE, enable);
}

bool PCF8574_GPIO::setTemperatureHeat(bool enable) {
    return digitalWrite(HEAT, enable);
}

bool PCF8574_GPIO::setTemperatureCool(bool enable) {
    return digitalWrite(COOL, enable);
}

bool PCF8574_GPIO::setHumidify(bool enable) {
    return digitalWrite(HUMIDIFY, enable);
}

bool PCF8574_GPIO::setDehumidify(bool enable) {
    return digitalWrite(DEHUMIDIFY, enable);
}

bool PCF8574_GPIO::setFanInterior(bool enable) {
    return digitalWrite(FAN_INTERIOR, enable);
}

bool PCF8574_GPIO::setFanExterior(bool enable) {
    return digitalWrite(FAN_EXTERIOR, enable);
}

bool PCF8574_GPIO::writeToDevice(uint8_t data) {
    selectTCAChannel(tcaChannel);
    
    Wire.beginTransmission(i2cAddress);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
}

uint8_t PCF8574_GPIO::readFromDevice() {
    selectTCAChannel(tcaChannel);
    
    Wire.requestFrom(i2cAddress, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF; // Default high state if read fails
}
