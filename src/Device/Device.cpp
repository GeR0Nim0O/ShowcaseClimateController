#include "Device.h"

#define PCA9548A_ADDRESS 0x70

Device::Device(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : wire(wire), i2cAddress(i2cAddress), tcaChannel(tcaChannel), deviceName(deviceName), 
      deviceIndex(deviceIndex), type("Generic"), typeNumber(""), initialized(false), threshold(0.0f) {
}

// Constructor for devices that specify threshold and channels first
Device::Device(TwoWire* wire, float threshold, std::map<String, String> channels, uint8_t i2cAddress, uint8_t tcaChannel, int deviceIndex)
    : wire(wire), i2cAddress(i2cAddress), tcaChannel(tcaChannel), deviceName(""), 
      deviceIndex(deviceIndex), type("Generic"), typeNumber(""), initialized(false), 
      threshold(threshold), channels(channels) {
    // This constructor is used by devices that specify threshold and channels first
}

void Device::selectTCAChannel(uint8_t channel) {
    if (channel > 7) return; // PCA9548A has 8 channels (0-7)
    
    wire->beginTransmission(PCA9548A_ADDRESS);
    wire->write(1 << channel);
    wire->endTransmission();
}

bool Device::testI2CConnection() {
    selectTCAChannel(tcaChannel);
    
    wire->beginTransmission(i2cAddress);
    return (wire->endTransmission() == 0);
}

// Add safe method to set initialized state
void Device::forceInitialized(bool state) {
    initialized = state;
}

// Add method to check if device is connected to I2C bus
bool Device::isConnected() {
    return testI2CConnection();
}

// Safe method to get channel threshold with error handling
float Device::getThreshold(const String& channelKey) const {
    // First try to find a channel-specific threshold
    auto it = channelThresholds.find(channelKey);
    if (it != channelThresholds.end()) {
        return it->second;
    }
    
    // If not found, return the default threshold
    return threshold;
}
