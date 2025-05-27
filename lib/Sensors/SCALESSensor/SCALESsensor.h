#ifndef SCALESSENSOR_H
#define SCALESSENSOR_H

#include "Device.h"
#include "I2CHandler.h"
#include <map>
#include <string>

#define SCALES_ADDRESS 0x26

class SCALESsensor : public Device {
public:
    SCALESsensor(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex);
    bool begin() override;
    std::map<std::string, float> readData() override; // Return a map of sensor data
    float getWeight() const;
    uint8_t getAddress() const { return _address; } // Add getAddress function
    float getThreshold() const { return threshold; } // Add getThreshold function
    int getDeviceIndex() const { return deviceIndex; } // Add getDeviceIndex function

private:
    bool writeCommand(uint8_t command);
    bool readBytes(uint8_t *data, uint8_t length);
    TwoWire* wire;
    uint8_t _address;
    float _weight;
    std::map<String, String> channels; // Add channels property
    int deviceIndex; // Add deviceIndex property
};

#endif // SCALESSENSOR_H
