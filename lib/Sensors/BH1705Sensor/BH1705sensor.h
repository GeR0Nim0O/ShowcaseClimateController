#ifndef BH1705SENSOR_H
#define BH1705SENSOR_H

#include "Device.h"
#include "I2CHandler.h"
#include <map>
#include <string>

#define BH1705_ADDRESS 0x23

class BH1705sensor : public Device {
public:
    BH1705sensor(uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex);
    bool begin() override;
    bool isConnected() override;
    void update() override;
    std::map<String, String> readData() override; // Return a map of sensor data    
    // Override pure virtual methods from Device
    std::map<String, String> getChannels() const override { return channels; }
    float getThreshold(const String& channelKey = "") const override { return threshold; }
    
    float getLux() const;

private:
    bool writeCommand(uint8_t command);    bool readBytes(uint8_t *data, uint8_t length);
    uint8_t _address;
    float _lux;
};

#endif // BH1705SENSOR_H
