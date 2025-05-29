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
    // CRITICAL FIX: Do NOT update (read) GPIO state if device is in OUTPUT mode
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        // In output mode, we trust our internal state and don't read from device
        return;
    }
    
    // Only read from device if in INPUT mode
    readByte(_gpioState);
}

std::map<String, String> PCF8574gpio::readData() {
    std::map<String, String> result;
    
    // CRITICAL FIX: Do NOT read from GPIO device if it's in OUTPUT mode
    // Reading from an output-mode PCF8574 can interfere with the current output states
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        
        // Instead of reading, return the current known state
        for (const auto& channel : channels) {
            int pin = channel.second.toInt();
            if (pin >= 0 && pin <= 7) {
                bool state = (_gpioState & (1 << pin)) != 0;
                result[channel.first] = state ? "1" : "0";
            }
        }
        return result;
    }
    
    // Only perform actual I2C read if in INPUT mode
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
    selectTCAChannel(tcaChannel);
    delay(2);
    
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        wire->beginTransmission(_address);
        wire->write(data);
        int result = wire->endTransmission();
        
        if (result == 0) {
            _gpioState = data;
            return true;
        } else {
            return false;
        }
    }
    
    // For input mode
    for (int attempt = 0; attempt < 3; attempt++) {
        wire->beginTransmission(_address);
        wire->write(data);
        int result = wire->endTransmission();
        
        if (result == 0) {
            _gpioState = data;
            delay(10);
            
            uint8_t readBack;
            if (readByte(readBack)) {
                if (readBack == data) {
                    return true;
                } else {
                    delay(50);
                }
            } else {
                delay(20);
            }
        } else {
            delay(20);
        }
    }
    
    return false;
}

bool ClimateController::readByte(uint8_t &data) {
    selectTCAChannel(tcaChannel);
    delay(5);
    
    for (int attempt = 0; attempt < 3; attempt++) {
        if (wire->requestFrom(_address, (uint8_t)1) == 1) {
            data = wire->read();
            _gpioState = data;
            return true;
        } else {
            delay(10);
        }
    }
    
    return false;
}

bool PCF8574gpio::writeBit(uint8_t pin, bool state) {
    if (pin > 7) {
        return false;
    }
    
    uint8_t currentState = _gpioState;
    
    if (state) {
        currentState |= (1 << pin);
    } else {
        currentState &= ~(1 << pin);
    }
    
    if (currentState != _gpioState) {
        return writeByte(currentState);
    } else {
        return true;
    }
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
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        selectTCAChannel(tcaChannel);
        delay(5);
        
        wire->beginTransmission(_address);
        wire->write(0x00);
        int result = wire->endTransmission();
        
        if (result == 0) {
            _gpioState = 0x00;
            delay(50);
        }
    } else {
        writeByte(0xFF);
    }
}

void PCF8574gpio::refreshOutputState() {
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        selectTCAChannel(tcaChannel);
        delay(2);
        
        wire->beginTransmission(_address);
        wire->write(_gpioState);
        wire->endTransmission();
    }
}

void PCF8574gpio::forceOutputMode() {
    _mode = PCF8574Mode::OUTPUT_MODE;
    
    selectTCAChannel(tcaChannel);
    delay(5);
    
    for (int i = 0; i < 2; i++) {
        wire->beginTransmission(_address);
        wire->write(_gpioState);
        wire->endTransmission();
        delay(10);
    }
}
