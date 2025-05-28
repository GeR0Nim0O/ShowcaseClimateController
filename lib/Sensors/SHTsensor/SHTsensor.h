#ifndef SHTSENSOR_H
#define SHTSENSOR_H

#include "Device.h"
#include "I2CHandler.h"
#include <map>
#include <string>

#define SHT_ADDRESS 0x44

class SHTsensor : public Device {
public:
    SHTsensor(uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex);
    
    bool begin() override;    bool isConnected() override;
    void update() override;    std::map<String, String> readData() override; // Return a map of sensor data
    // Override pure virtual methods from Device - using base class getThreshold for channel-specific thresholds
    
    float getTemperature() const;
    float getHumidity() const;
    uint32_t getSerialNumber(); // Add method to read serial number
    uint16_t readStatus(); // Add method to read status register
    bool clearStatus(); // Add method to clear status register
    bool setMeasurementMode(uint16_t mode); // Add method to set measurement mode
    bool setHeater(bool enable); // Add method to enable/disable heater
    uint8_t getAddress() const { return _address; } // Add getAddress function
    
private:
    bool readRawData(uint16_t &rawTemperature, uint16_t &rawHumidity);
    bool writeCommand(uint16_t command);
    bool readBytes(uint8_t *data, uint8_t length);
    TwoWire* wire;
    uint8_t _address;
    float _temperature;
    float _humidity;
};

#endif // SHTSENSOR_H
