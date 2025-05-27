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
    Serial.println(channels.size());
    Serial.print("Type: ");
    Serial.println(type);
    Serial.print("Device Index: ");
    Serial.println(deviceIndex);
}

bool PCF8574gpio::begin() {
    Serial.println("DEBUG: PCF8574gpio::begin() START");
    Serial.println("DEBUG: PCF8574gpio::begin() called");
    I2CHandler::selectTCA(tcaChannel); // Use tcaChannel from Device base class
    
    // First test I2C connection
    if (!isConnected()) {
        Serial.println("PCF8574 GPIO expander not found!");
        return false;
    }
    
    Serial.println("DEBUG: PCF8574 connection test passed, starting GPIO initialization");
    
    // Initialize GPIO state - try multiple times if needed
    int retries = 3;
    bool success = false;
    
    for (int i = 0; i < retries && !success; i++) {
        Serial.print("DEBUG: PCF8574 write attempt ");
        Serial.println(i + 1);
        success = writeByte(_gpioState); // Initialize GPIO state
        if (!success) {
            Serial.print("PCF8574 write attempt ");
            Serial.print(i + 1);
            Serial.println(" failed, retrying...");
            delay(10);
        }
    }
    
    Serial.print("DEBUG: PCF8574 initialization success = ");
    Serial.println(success);
    
    if (success) {
        Serial.println("DEBUG: Setting initialized = true");
        initialized = true; // Set initialized flag to true
        Serial.print("DEBUG: initialized flag is now: ");
        Serial.println(initialized);
        Serial.println("PCF8574 GPIO expander initialized successfully");
    } else {
        Serial.println("PCF8574 GPIO expander initialization failed after retries");
    }
    
    Serial.print("DEBUG: PCF8574gpio::begin() returning ");
    Serial.println(success);
    return success;
}

std::map<String, String> PCF8574gpio::readData() {
    std::map<String, String> dataMap;
    uint8_t data;
    if (!readByte(data)) {
        return dataMap;
    }

    for (const auto& channel : channels) {
        uint8_t pin = channel.first.toInt();
        bool state = (data & (1 << pin)) ? false : true; // Correctly invert the state
        String value = state ? "0.0" : "1.0"; // Convert to String for return type
        dataMap[channel.first] = value;
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
