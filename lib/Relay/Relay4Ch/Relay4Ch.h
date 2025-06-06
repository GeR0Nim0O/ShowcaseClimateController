#ifndef RELAY4CH_H
#define RELAY4CH_H

#include "Device.h"
#include "I2CHandler.h"
#include <map>
#include <string>

#define UNIT_4RELAY_ADDR      0x26
#define UNIT_4RELAY_REG       0x10
#define UNIT_4RELAY_RELAY_REG 0x11

enum class Relay4ChMode {
    ASYNC = 0,  // LED & Relay Asynchronous
    SYNC = 1    // LED & Relay Synchronous
};

class Relay4Ch : public Device {
public:
    Relay4Ch(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex, Relay4ChMode mode = Relay4ChMode::SYNC);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    std::map<String, String> readData() override; // Return a map of relay states
    
    // Override pure virtual methods from Device
    float getThreshold(const String& channelKey) const override { return threshold; }
    
    // Core relay control methods
    bool relayWrite(uint8_t channel, bool state);
    bool relayAll(bool state);
    bool ledWrite(uint8_t channel, bool state);
    bool ledAll(bool state);
    
    // Relay state methods
    bool getRelayState(uint8_t channel);
    uint8_t getAllRelayStates();
    
    // Mode management
    Relay4ChMode getMode() const;
    void setMode(Relay4ChMode mode);
    bool isSyncMode() const;
    bool isAsyncMode() const;
    
    // Initialization and utility
    void initializeRelays(); // Initialize all relays to OFF
    void refreshRelayState();
    
private:
    uint8_t _address;
    uint8_t _relayState;    // Current relay state (bits 0-3 for relays 1-4)
    uint8_t _ledState;      // Current LED state (bits 0-3 for LEDs 1-4)
    Relay4ChMode _mode;
    std::map<std::string, float> lastSensorValues;
    
    // Low-level I2C communication
    bool write1Byte(uint8_t register_address, uint8_t data);
    uint8_t read1Byte(uint8_t register_address);
    
    // Internal state management
    void updateInternalState();
    bool validateChannel(uint8_t channel);
};

#endif // RELAY4CH_H
