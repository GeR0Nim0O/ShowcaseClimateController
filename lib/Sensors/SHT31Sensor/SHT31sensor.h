#ifndef SHT31SENSOR_H
#define SHT31SENSOR_H

#include "Device.h"
#include "I2CHandler.h"
#include <map>
#include <string>

#define SHT31_ADDRESS 0x44

class SHT31sensor : public Device {
public:
    SHT31sensor(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex);
    bool begin() override;
    std::map<std::string, float> readData() override; // Return a map of sensor data
    float getTemperature() const;
    float getHumidity() const;
    uint32_t getSerialNumber(); // Add method to read serial number
    uint16_t readStatus(); // Add method to read status register
    bool clearStatus(); // Add method to clear status register
    bool setMeasurementMode(uint16_t mode); // Add method to set measurement mode
    bool setHeater(bool enable); // Add method to enable/disable heater
    uint8_t getAddress() const { return _address; } // Add getAddress function
    float getThreshold() const { return threshold; } // Add getThreshold function

private:
    bool readRawData(uint16_t &rawTemperature, uint16_t &rawHumidity);
    bool writeCommand(uint16_t command);
    bool readBytes(uint8_t *data, uint8_t length);
    TwoWire* wire;
    uint8_t _address;
    float _temperature;
    float _humidity;
    uint8_t numChannels; // Add numChannels property
    String type; // Add type property
    std::map<String, String> channels; // Add channels property
};

#endif // SHT31SENSOR_H
