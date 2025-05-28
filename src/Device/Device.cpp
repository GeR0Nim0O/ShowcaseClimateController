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
    
    Wire.beginTransmission(PCA9548A_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

bool Device::testI2CConnection() {
    selectTCAChannel(tcaChannel);
    
    Wire.beginTransmission(i2cAddress);
    return (Wire.endTransmission() == 0);
}
