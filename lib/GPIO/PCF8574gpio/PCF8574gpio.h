#ifndef PCF8574GPIO_H
#define PCF8574GPIO_H

#include "Device.h"
#include "I2CHandler.h"
#include <map>
#include <string>

#define PCF8574_ADDRESS 0x20

class PCF8574gpio : public Device {
public:
    PCF8574gpio(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex);
    bool begin() override;
    std::map<std::string, float> readData() override; // Return a map of GPIO states
    uint8_t getAddress() const { return _address; } // Add getAddress function
    float getThreshold() const { return threshold; } // Add getThreshold function
    int getDeviceIndex() const { return deviceIndex; } // Add deviceIndex function
    std::string getType() const { return "gpio"; } // Inherit getType method
    std::string getTypeNumber() const { return "PCF8574"; } // Inherit getTypeNumber method
    bool readBit(uint8_t pin, bool &state); // Add readBit function declaration

private:
    bool writeByte(uint8_t data);
    bool readByte(uint8_t &data);
    TwoWire* wire;
    uint8_t _address;
    uint8_t _gpioState;
    std::map<String, String> channels; // Add channels property
    int deviceIndex; // Add deviceIndex property
    std::map<std::string, float> lastSensorValues; // Ensure lastSensorValues is declared
};

#endif // PCF8574GPIO_H
