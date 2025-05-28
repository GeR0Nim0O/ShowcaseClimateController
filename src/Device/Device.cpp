#include "Device.h"

#define PCA9548A_ADDRESS 0x70

// Primary constructor
Device::Device(TwoWire* wire, const String& type, const String& deviceName, uint8_t i2cAddress, uint8_t tcaChannel, int deviceIndex)
    : wire(wire), type(type), i2cAddress(i2cAddress), tcaChannel(tcaChannel), 
      deviceIndex(deviceIndex), initialized(false), deviceName(deviceName) {
}

// Alternative constructor for legacy devices that specify threshold and channels first
Device::Device(TwoWire* wire, float threshold, std::map<String, String> channels, uint8_t i2cAddress, uint8_t tcaChannel, int deviceIndex)
    : wire(wire), type("Generic"), i2cAddress(i2cAddress), tcaChannel(tcaChannel), 
      deviceIndex(deviceIndex), initialized(false), deviceName(""), 
      channels(channels) {
    // Set a default threshold for all channels if needed
    for (const auto& channel : channels) {
        thresholds[channel.first] = threshold;
    }
}

// Constructor for devices that only need basic parameters
Device::Device(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : wire(wire), type("Generic"), i2cAddress(i2cAddress), tcaChannel(tcaChannel), 
      deviceIndex(deviceIndex), initialized(false), deviceName(deviceName) {
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

float Device::getThreshold(const String& channelId) const {
    auto it = thresholds.find(channelId);
    return (it != thresholds.end()) ? it->second : 0.0f;
}

void Device::addChannel(const String& channelId, const String& channelType, float threshold) {
    channels[channelId] = channelType;
    thresholds[channelId] = threshold;
}

void Device::removeChannel(const String& channelId) {
    channels.erase(channelId);
    thresholds.erase(channelId);
}

void Device::clearChannels() {
    channels.clear();
    thresholds.clear();
}

std::map<String, String> Device::readData() {
    return std::map<String, String>();
}

bool Device::writeData(const std::map<String, String>& data) {
    return true;
}

bool Device::connect() {
    return isConnected();
}

bool Device::disconnect() {
    return true;
}
