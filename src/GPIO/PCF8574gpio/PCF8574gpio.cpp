#include "PCF8574gpio.h"

PCF8574gpio::PCF8574gpio(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex, PCF8574Mode mode)
    : Device(wire, threshold, channels, i2cAddress, tcaChannel, deviceIndex), _address(i2cAddress), _gpioState(0xFF), _mode(mode) {
      // Set type - inherited from Device base class
    type = "PCF8574GPIO";
    Serial.println("PCF8574gpio created:");
    Serial.print("Address: 0x");
    Serial.println(_address, HEX);
    Serial.print("Mode: ");
    Serial.println(_mode == PCF8574Mode::INPUT_MODE ? "INPUT_MODE" : "OUTPUT_MODE");
    Serial.print("Threshold: ");
    Serial.println(threshold);
    Serial.print("Number of Channels: ");
    Serial.println(channels.size());
    Serial.print("Device Index: ");
    Serial.println(deviceIndex);
}

bool PCF8574gpio::begin() {
    bool connected = isConnected();
    if (connected) {
        initialized = true;  // Set initialized flag to true when connection is successful
        initializeOutputs();  // Initialize pins based on mode
    }
    return connected;
}

bool PCF8574gpio::isConnected() {
    selectTCAChannel(tcaChannel);  // Select TCA channel before communication
    wire->beginTransmission(_address);
    return (wire->endTransmission() == 0);
}

void PCF8574gpio::update() {
    // Update GPIO state by reading the current state
    readByte(_gpioState);
}

std::map<String, String> PCF8574gpio::readData() {
    std::map<String, String> result;
    update();
    
    // Convert GPIO states to a map of channel values
    for (const auto& channel : channels) {
        int pin = channel.second.toInt();
        if (pin >= 0 && pin <= 7) {
            bool state = (_gpioState & (1 << pin)) != 0;
            result[channel.first] = state ? "1" : "0";
        }
    }
    
    return result;
}

bool PCF8574gpio::writeByte(uint8_t data) {
    // Ensure TCA channel is selected before communication
    selectTCAChannel(tcaChannel);
    delay(2); // Small delay for TCA switching
    
    wire->beginTransmission(_address);
    wire->write(data);
    int result = wire->endTransmission();
    
    if (result == 0) {
        _gpioState = data;
        
        // Verify the write by reading back (optional debug)
        uint8_t readBack;
        if (readByte(readBack)) {
            if (readBack != data) {
                Serial.print("GPIO Write verification failed! Expected: 0x");
                Serial.print(data, HEX);
                Serial.print(", Read: 0x");
                Serial.println(readBack, HEX);
            }
        }
        return true;
    } else {
        Serial.print("GPIO Write failed with I2C error: ");
        Serial.println(result);
        return false;
    }
}

bool PCF8574gpio::readByte(uint8_t &data) {
    // Ensure TCA channel is selected before communication
    selectTCAChannel(tcaChannel);
    delay(2); // Small delay for TCA switching
    
    if (wire->requestFrom(_address, (uint8_t)1) != 1) {
        Serial.println("GPIO Read failed: No data received");
        return false;
    }
    data = wire->read();
    _gpioState = data;
    return true;
}

bool PCF8574gpio::readBit(uint8_t pin, bool &state) {
    if (pin > 7) return false;
    
    uint8_t data;
    if (!readByte(data)) return false;
    
    state = (data & (1 << pin)) != 0;
    return true;
}

bool PCF8574gpio::writeBit(uint8_t pin, bool state) {
    if (pin > 7) {
        Serial.print("GPIO Error: Invalid pin number ");
        Serial.println(pin);
        return false;
    }
    
    // Update the local state
    if (state) {
        _gpioState |= (1 << pin);
    } else {
        _gpioState &= ~(1 << pin);
    }
    
    // Write to hardware with debug output
    Serial.print("GPIO: Setting pin ");
    Serial.print(pin);
    Serial.print(" to ");
    Serial.print(state ? "HIGH" : "LOW");
    Serial.print(" (state: 0x");
    Serial.print(_gpioState, HEX);
    Serial.print(") - ");
    
    bool success = writeByte(_gpioState);
    Serial.println(success ? "SUCCESS" : "FAILED");
    
    return success;
}

bool PCF8574gpio::readPin(uint8_t pin) {
    bool state = false;
    readBit(pin, state);
    return state;
}

void PCF8574gpio::writePin(uint8_t pin, bool state) {
    writeBit(pin, state);
}

// Mode management methods
PCF8574Mode PCF8574gpio::getMode() const {
    return _mode;
}

void PCF8574gpio::setMode(PCF8574Mode mode) {
    _mode = mode;
    Serial.print("PCF8574 mode changed to: ");
    Serial.println(_mode == PCF8574Mode::INPUT_MODE ? "INPUT_MODE" : "OUTPUT_MODE");
    
    // If switching to output mode, initialize all outputs to false
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        initializeOutputs();
    }
}

bool PCF8574gpio::isOutputMode() const {
    return _mode == PCF8574Mode::OUTPUT_MODE;
}

bool PCF8574gpio::isInputMode() const {
    return _mode == PCF8574Mode::INPUT_MODE;
}

void PCF8574gpio::initializeOutputs() {
    Serial.print("PCF8574: Initializing outputs for mode: ");
    Serial.println(_mode == PCF8574Mode::INPUT_MODE ? "INPUT_MODE" : "OUTPUT_MODE");
    
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        // Set all outputs to false (0x00)
        Serial.println("PCF8574: Setting all outputs to LOW (0x00)");
        bool success = writeByte(0x00);
        if (success) {
            Serial.println("PCF8574: Output initialization SUCCESS");
        } else {
            Serial.println("PCF8574: Output initialization FAILED");
        }
    } else {
        // For input mode, set all pins high to enable pull-ups
        Serial.println("PCF8574: Enabling pull-ups (0xFF)");
        bool success = writeByte(0xFF);
        if (success) {
            Serial.println("PCF8574: Input initialization SUCCESS");
        } else {
            Serial.println("PCF8574: Input initialization FAILED");
        }
    }
}
