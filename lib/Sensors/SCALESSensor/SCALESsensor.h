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
    bool isConnected() override;
    void update() override;
    std::map<String, String> readData() override; // Return a map of sensor data    
    // Override pure virtual methods from Device
    std::map<String, String> getChannels() const override { return channels; }
    float getThreshold(const String& channelKey = "") const override { return threshold; }
    
    float getWeight() const;

private:
    bool writeCommand(uint8_t command);    bool readBytes(uint8_t *data, uint8_t length);
    uint8_t _address;
    float _weight;
};

#endif // SCALESSENSOR_H
