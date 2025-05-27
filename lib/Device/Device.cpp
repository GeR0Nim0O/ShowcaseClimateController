#include "Device.h"

// Implementation of the new constructor
Device::Device(float threshold, std::map<String, String> channels, uint8_t i2cAddress, uint8_t tcaChannel, int deviceIndex)
    : i2cAddress(i2cAddress), tcaChannel(tcaChannel), deviceIndex(deviceIndex),
      threshold(threshold), channels(channels), initialized(false) {
    deviceName = "Device" + String(deviceIndex);
    type = "GenericDevice";
    typeNumber = "";
}

// If this file already exists, make sure to add the implementation above
// but preserve other existing constructor implementations and methods
