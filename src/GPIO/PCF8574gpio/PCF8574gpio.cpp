#include "PCF8574gpio.h"

PCF8574gpio::PCF8574gpio(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex, PCF8574Mode mode)
    : Device(threshold, channels, i2cChannel, tcaPort, deviceIndex), wire(wire), _address(PCF8574_ADDRESS), _gpioState(0xFF), _mode(mode) {
    
    // Set type - inherited from Device base class
    type = "PCF8574GPIO";    Serial.println("PCF8574gpio created:");
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
    wire->begin();
    bool connected = isConnected();
    if (connected) {
        initialized = true;  // Set initialized flag to true when connection is successful
        initializeOutputs();  // Initialize pins based on mode
    }
    return connected;
}

bool PCF8574gpio::isConnected() {
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
    wire->beginTransmission(_address);
    wire->write(data);
    _gpioState = data;
    return (wire->endTransmission() == 0);
}

bool PCF8574gpio::readByte(uint8_t &data) {
    if (wire->requestFrom(_address, (uint8_t)1) != 1) {
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
    if (pin > 7) return false;
    
    if (state) {
        _gpioState |= (1 << pin);
    } else {
        _gpioState &= ~(1 << pin);
    }
    
    return writeByte(_gpioState);
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
    Serial.println(_mode == PCF8574Mode::INPUT ? "INPUT" : "OUTPUT");
    
    // If switching to output mode, initialize all outputs to false
    if (_mode == PCF8574Mode::OUTPUT) {
        initializeOutputs();
    }
}

bool PCF8574gpio::isOutputMode() const {
    return _mode == PCF8574Mode::OUTPUT;
}

bool PCF8574gpio::isInputMode() const {
    return _mode == PCF8574Mode::INPUT;
}

void PCF8574gpio::initializeOutputs() {
    if (_mode == PCF8574Mode::OUTPUT) {
        // Set all outputs to false (0x00)
        writeByte(0x00);
        Serial.println("PCF8574 outputs initialized to false (0x00)");
    } else {
        // For input mode, set all pins high to enable pull-ups
        writeByte(0xFF);
        Serial.println("PCF8574 inputs initialized with pull-ups enabled (0xFF)");
    }
}
