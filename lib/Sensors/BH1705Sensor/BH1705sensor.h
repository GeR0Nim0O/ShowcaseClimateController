#ifndef BH1705SENSOR_H
#define BH1705SENSOR_H

#include "Device.h"
#include "I2CHandler.h"
#include <map>
#include <string>

#define BH1705_ADDRESS 0x23

class BH1705sensor : public Device {
public:
    BH1705sensor(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex);
    bool begin() override;
    std::map<std::string, float> readData() override; // Return a map of sensor data
    float getLux() const;
    uint8_t getAddress() const { return _address; } // Add getAddress function
    float getThreshold() const { return threshold; } // Add getThreshold function
    int getDeviceIndex() const { return deviceIndex; } // Add deviceIndex function
    std::string getType() const { return "sensor"; } // Inherit getType method
    std::string getTypeNumber() const { return "BH1705"; } // Inherit getTypeNumber method

private:
    bool writeCommand(uint8_t command);
    bool readBytes(uint8_t *data, uint8_t length);
    TwoWire* wire;
    uint8_t _address;
    float _lux;
    std::map<String, String> channels; // Add channels property
    int deviceIndex; // Add deviceIndex property
};

#endif // BH1705SENSOR_H
