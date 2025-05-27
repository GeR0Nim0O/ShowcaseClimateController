#include "Device.h"
#include "PCF8574gpio.h"
#include "I2CHandler.h"

PCF8574gpio::PCF8574gpio(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, i2cChannel, tcaPort, threshold, channels, deviceIndex), wire(wire), _address(PCF8574_ADDRESS), _gpioState(0xFF) {
    type = "GPIO"; // Fixed type
    Serial.println("PCF8574gpio created:");
    Serial.print("Address: ");
    Serial.println(_address, HEX);
    Serial.print("Threshold: ");
    Serial.println(threshold);
    Serial.print("Number of Channels: ");
    Serial.println(numChannels);
    Serial.print("Type: ");
    Serial.println(type);
    Serial.print("TypeNumber: ");
    Serial.println(typeNumber);
    Serial.print("Device Index: ");
    Serial.println(deviceIndex);
}

bool PCF8574gpio::begin() {
    I2CHandler::selectTCA(tcaPort); // Use tcaPort from class definition
    return writeByte(_gpioState); // Initialize GPIO state
}

std::map<std::string, float> PCF8574gpio::readData() {
    std::map<std::string, float> dataMap;
    uint8_t data;
    if (!readByte(data)) {
        return dataMap;
    }

    for (const auto& channel : channels) {
        uint8_t pin = channel.first.toInt();
        bool state = (data & (1 << pin)) ? false : true; // Correctly invert the state
        float value = state ? 0.0 : 1.0; // Invert the state
        if (lastSensorValues[channel.second.c_str()] != value) {
            Serial.print("Input changed on pin: ");
            Serial.print(pin);
            Serial.print(", new state: ");
            Serial.println(value);
            lastSensorValues[channel.second.c_str()] = value; // Update last known value
        }
        dataMap[channel.second.c_str()] = value;
    }
    return dataMap;
}

bool PCF8574gpio::writeByte(uint8_t data) {
    wire->beginTransmission(_address);
    wire->write(data);
    return wire->endTransmission() == 0;
}

bool PCF8574gpio::readByte(uint8_t &data) {
    if (wire->requestFrom(_address, (uint8_t)1) != 1) {
        return false;
    }
    data = wire->read();
    return true;
}

bool PCF8574gpio::readBit(uint8_t pin, bool &state) {
    uint8_t data;
    if (!readByte(data)) {
        return false;
    }
    state = (data & (1 << pin)) ? false : true; // Correctly invert the state
    return true;
}

bool PCF8574gpio::writeBit(uint8_t pin, bool state) {
    uint8_t data;
    if (!readByte(data)) {
        return false;
    }
    
    if (state) {
        data &= ~(1 << pin); // Set bit low (inverted logic)
    } else {
        data |= (1 << pin);  // Set bit high (inverted logic)
    }
    
    return writeByte(data);
}

bool PCF8574gpio::readPin(uint8_t pin) {
    bool state;
    if (readBit(pin, state)) {
        return state;
    }
    return false;
}

void PCF8574gpio::writePin(uint8_t pin, bool state) {
    writeBit(pin, state);
}

// Implementation of pure virtual methods from Device base class
bool PCF8574gpio::isConnected() {
    wire->beginTransmission(_address);
    return (wire->endTransmission() == 0);
}

void PCF8574gpio::update() {
    // Update GPIO readings
    readData();
}
