#include "Device.h"

#define PCA9548A_ADDRESS 0x70

// Primary constructor
Device::Device(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : wire(wire), i2cAddress(i2cAddress), tcaChannel(tcaChannel), 
      deviceIndex(deviceIndex), initialized(false), deviceName(deviceName), threshold(0.0) {
}

// Alternative constructor for legacy devices that specify threshold and channels first
Device::Device(TwoWire* wire, float threshold, std::map<String, String> channels, uint8_t i2cAddress, uint8_t tcaChannel, int deviceIndex)
    : wire(wire), i2cAddress(i2cAddress), tcaChannel(tcaChannel), 
      deviceIndex(deviceIndex), initialized(false), deviceName(""), 
      channels(channels), threshold(threshold) {
    // Set the threshold for all channels
    for (const auto& channel : channels) {
        channelThresholds[channel.first] = threshold;
    }
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

bool Device::begin() {
    initialized = isConnected();
    return initialized;
}

bool Device::isConnected() {
    return testI2CConnection();
}

void Device::update() {
    // Default implementation - do nothing
    // Override in derived classes as needed
}

std::map<String, String> Device::readData() {
    // Default implementation - return empty map
    // Override in derived classes as needed
    return std::map<String, String>();
}
