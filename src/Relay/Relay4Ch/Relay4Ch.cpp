#include "Relay4Ch.h"

Relay4Ch::Relay4Ch(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex, Relay4ChMode mode)
    : Device(wire, threshold, channels, i2cAddress, tcaChannel, deviceIndex), _address(i2cAddress), _relayState(0x00), _ledState(0x00), _mode(mode) {
    
    // Set type - inherited from Device base class
    type = "Relay4Ch";
    Serial.println("Relay4Ch created:");
    Serial.print("Address: 0x");
    Serial.println(_address, HEX);
    Serial.print("Mode: ");
    Serial.println(_mode == Relay4ChMode::SYNC ? "SYNC" : "ASYNC");
    Serial.print("Threshold: ");
    Serial.println(threshold);
    Serial.print("Number of Channels: ");
    Serial.println(channels.size());
    Serial.print("Device Index: ");
    Serial.println(deviceIndex);
}

bool Relay4Ch::begin() {
    bool connected = isConnected();
    if (connected) {
        // Initialize the relay module with the specified mode (M5Stack-style initialization)
        // Set mode register first
        if (!write1Byte(UNIT_4RELAY_REG, static_cast<uint8_t>(_mode))) {
            Serial.println("Failed to set relay mode during begin");
            return false;
        }
        
        // Turn off all relays and LEDs
        if (!write1Byte(UNIT_4RELAY_RELAY_REG, 0x00)) {
            Serial.println("Failed to turn off relays during begin");
            return false;
        }
        
        _relayState = 0x00;
        _ledState = 0x00;
        initialized = true;  // Mark as initialized after successful setup
        
        Serial.print("Relay4Ch initialization successful at address 0x");
        Serial.print(_address, HEX);
        Serial.print(" with mode: ");
        Serial.println(_mode == Relay4ChMode::SYNC ? "SYNC" : "ASYNC");
    } else {
        Serial.print("Relay4Ch connection failed at address 0x");
        Serial.println(_address, HEX);
    }
    return connected;
}

bool Relay4Ch::isConnected() {
    selectTCAChannel(tcaChannel);  // Select TCA channel before communication
    wire->beginTransmission(_address);
    return (wire->endTransmission() == 0);
}

void Relay4Ch::update() {
    // Read current state from device
    updateInternalState();
}

std::map<String, String> Relay4Ch::readData() {
    std::map<String, String> result;
    
    // Don't constantly read from hardware as it may interfere with relay operation
    // Use our tracked internal state instead
    
    // Convert relay states to a map of channel values based on our tracked state
    for (const auto& channel : channels) {
        // Parse channel number from channel value (e.g., "0", "1", "2", "3")
        int relayChannel = channel.second.toInt();
        if (validateChannel(relayChannel)) {
            bool state = getRelayState(relayChannel);
            result[channel.first] = state ? "1" : "0";
        }
    }
    
    return result;
}

bool Relay4Ch::relayWrite(uint8_t channel, bool state) {
    if (!validateChannel(channel)) {
        Serial.print("Invalid relay channel: ");
        Serial.println(channel);
        return false;
    }
    
    // Note: The relay can only be controlled in synchronous mode
    if (_mode != Relay4ChMode::SYNC) {
        Serial.println("Warning: Relay control only works in SYNC mode");
        return false;
    }
    
    uint8_t currentState = _relayState;
    
    if (state) {
        currentState |= (1 << channel);   // Set bit
    } else {
        currentState &= ~(1 << channel);  // Clear bit
    }
    
    // Write to relay control register
    if (write1Byte(UNIT_4RELAY_RELAY_REG, currentState)) {
        _relayState = currentState;
        Serial.print("Relay ");
        Serial.print(channel);
        Serial.print(" set to ");
        Serial.println(state ? "ON" : "OFF");
        return true;
    }
    
    Serial.print("Failed to control relay ");
    Serial.println(channel);
    return false;
}

bool Relay4Ch::relayAll(bool state) {
    uint8_t newState = state ? 0x0F : 0x00;  // All 4 relays on or off
    
    if (_mode != Relay4ChMode::SYNC) {
        Serial.println("Warning: Relay control only works in SYNC mode");
        return false;
    }
    
    if (write1Byte(UNIT_4RELAY_RELAY_REG, newState)) {
        _relayState = newState;
        Serial.print("All relays set to ");
        Serial.println(state ? "ON" : "OFF");
        return true;
    }
    
    Serial.println("Failed to control all relays");
    return false;
}

bool Relay4Ch::ledWrite(uint8_t channel, bool state) {
    if (!validateChannel(channel)) {
        Serial.print("Invalid LED channel: ");
        Serial.println(channel);
        return false;
    }
    
    uint8_t currentState = _ledState;
    
    if (state) {
        currentState |= (1 << (channel + 4));   // LED bits are 4-7
    } else {
        currentState &= ~(1 << (channel + 4));  // Clear LED bit
    }
    
    // Write to relay control register (LEDs are in upper 4 bits)
    uint8_t combinedState = (_ledState & 0xF0) | (_relayState & 0x0F);
    
    if (write1Byte(UNIT_4RELAY_RELAY_REG, combinedState)) {
        _ledState = currentState;
        Serial.print("LED ");
        Serial.print(channel);
        Serial.print(" set to ");
        Serial.println(state ? "ON" : "OFF");
        return true;
    }
    
    Serial.print("Failed to control LED ");
    Serial.println(channel);
    return false;
}

bool Relay4Ch::ledAll(bool state) {
    uint8_t newLedState = state ? 0xF0 : 0x00;  // All 4 LEDs on or off (bits 4-7)
    uint8_t combinedState = newLedState | (_relayState & 0x0F);
    
    if (write1Byte(UNIT_4RELAY_RELAY_REG, combinedState)) {
        _ledState = newLedState;
        Serial.print("All LEDs set to ");
        Serial.println(state ? "ON" : "OFF");
        return true;
    }
    
    Serial.println("Failed to control all LEDs");
    return false;
}

bool Relay4Ch::getRelayState(uint8_t channel) {
    if (!validateChannel(channel)) {
        return false;
    }
    return (_relayState & (1 << channel)) != 0;
}

uint8_t Relay4Ch::getAllRelayStates() {
    return _relayState & 0x0F;  // Only return lower 4 bits (relay states)
}

Relay4ChMode Relay4Ch::getMode() const {
    return _mode;
}

void Relay4Ch::setMode(Relay4ChMode mode) {
    _mode = mode;
    if (initialized) {
        write1Byte(UNIT_4RELAY_REG, static_cast<uint8_t>(_mode));
        Serial.print("Relay mode changed to ");
        Serial.println(_mode == Relay4ChMode::SYNC ? "SYNC" : "ASYNC");
    }
}

bool Relay4Ch::isSyncMode() const {
    return _mode == Relay4ChMode::SYNC;
}

bool Relay4Ch::isAsyncMode() const {
    return _mode == Relay4ChMode::ASYNC;
}

void Relay4Ch::initializeRelays() {
    // Turn off all relays and LEDs
    _relayState = 0x00;
    _ledState = 0x00;
    
    if (initialized) {
        write1Byte(UNIT_4RELAY_RELAY_REG, 0x00);
        Serial.println("All relays and LEDs initialized to OFF");
    }
}

void Relay4Ch::refreshRelayState() {
    if (initialized) {
        updateInternalState();
    }
}

bool Relay4Ch::write1Byte(uint8_t register_address, uint8_t data) {
    selectTCAChannel(tcaChannel);
    delay(2);  // Small delay for I2C stability
    
    wire->beginTransmission(_address);
    wire->write(register_address);
    wire->write(data);
    int result = wire->endTransmission();
    
    if (result == 0) {
        return true;
    } else {
        Serial.print("I2C write error: ");
        Serial.println(result);
        return false;
    }
}

uint8_t Relay4Ch::read1Byte(uint8_t register_address) {
    selectTCAChannel(tcaChannel);
    delay(2);
    
    wire->beginTransmission(_address);
    wire->write(register_address);
    if (wire->endTransmission() != 0) {
        Serial.println("I2C write error during read setup");
        return 0;
    }
    
    wire->requestFrom(_address, (uint8_t)1);
    if (wire->available()) {
        return wire->read();
    } else {
        Serial.println("No data available from I2C read");
        return 0;
    }
}

void Relay4Ch::updateInternalState() {
    // Note: Many relay modules are write-only and don't support reading back relay states
    // We rely on our internal _relayState tracking instead of reading from hardware
    // This prevents I2C errors and ensures consistent state tracking
    
    if (initialized) {
        // Optional: Try to read from hardware but don't rely on it
        // Some relay modules return invalid data or don't support read operations
        Serial.print("Current tracked relay state: 0x");
        Serial.println(_relayState, HEX);
    }
}

bool Relay4Ch::validateChannel(uint8_t channel) {
    return (channel >= 0 && channel <= 3);  // Valid channels are 0, 1, 2, 3
}
