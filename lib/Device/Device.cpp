#include "Device.h"

#define PCA9548A_ADDRESS 0x70

Device::Device(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : i2cAddress(i2cAddress), tcaChannel(tcaChannel), deviceName(deviceName), 
      deviceIndex(deviceIndex), type("Generic"), initialized(false), threshold(0.0f) {
}

// Constructor for sensor devices with threshold and channels
Device::Device(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex)
    : i2cAddress(i2cChannel), tcaChannel(tcaPort), deviceName(""), 
      deviceIndex(deviceIndex), type("Generic"), initialized(false), 
      threshold(threshold), channels(channels) {
    // This constructor is used by sensor classes that handle their own addressing
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
