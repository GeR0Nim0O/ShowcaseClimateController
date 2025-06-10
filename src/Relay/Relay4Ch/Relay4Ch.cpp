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
        initialized = true;  // Set initialized flag to true when connection is successful
        
        // Initialize the relay module with the specified mode
        // Set mode and turn off all relays
        if (!write1Byte(UNIT_4RELAY_REG, static_cast<uint8_t>(_mode))) {
            Serial.println("Failed to set relay mode");
            return false;
        }
        
        initializeRelays();  // Initialize all relays to OFF
        
        Serial.println("Relay4Ch initialization successful");
    } else {
        Serial.println("Relay4Ch initialization failed - device not connected");
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
    
    // Update internal state first
    updateInternalState();
    
    // Convert relay states to a map of channel values
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

void Relay4Ch::Init(bool mode) {
    // M5Stack-compatible initialization
    // mode: 0 = ASYNC, 1 = SYNC
    _mode = mode ? Relay4ChMode::SYNC : Relay4ChMode::ASYNC;
    
    // Set mode register
    if (!write1Byte(UNIT_4RELAY_REG, static_cast<uint8_t>(_mode))) {
        Serial.println("Failed to set relay mode during Init");
        return;
    }
    
    // Turn off all relays and LEDs
    if (!write1Byte(UNIT_4RELAY_RELAY_REG, 0x00)) {
        Serial.println("Failed to turn off relays during Init");
        return;
    }
    
    _relayState = 0x00;
    _ledState = 0x00;
    initialized = true;  // Mark as initialized after successful Init
    
    Serial.print("Relay4Ch Init completed with mode: ");
    Serial.println(_mode == Relay4ChMode::SYNC ? "SYNC" : "ASYNC");
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
    if (initialized) {
        uint8_t state = read1Byte(UNIT_4RELAY_RELAY_REG);
        _relayState = state & 0x0F;  // Lower 4 bits are relay states
        _ledState = state & 0xF0;    // Upper 4 bits are LED states
    }
}

bool Relay4Ch::validateChannel(uint8_t channel) {
    return (channel >= 0 && channel <= 3);  // Valid channels are 0, 1, 2, 3
}
