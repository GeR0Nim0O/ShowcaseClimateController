#ifndef PCF8574GPIO_H
#define PCF8574GPIO_H

#include "Device.h"
#include "I2CHandler.h"
#include <map>
#include <string>

#define PCF8574_ADDRESS 0x20

enum class PCF8574Mode {
    INPUT_MODE,
    OUTPUT_MODE
};

class PCF8574gpio : public Device {
public:
    PCF8574gpio(uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex, PCF8574Mode mode = PCF8574Mode::OUTPUT_MODE);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    std::map<String, String> readData() override; // Return a map of GPIO states
      // Override pure virtual methods from Device
    float getThreshold(const String& channelKey) const override { return threshold; }
    
    bool writeByte(uint8_t data);
    bool readByte(uint8_t &data);
    bool readBit(uint8_t pin, bool &state);
    bool writeBit(uint8_t pin, bool state);    bool readPin(uint8_t pin);
    void writePin(uint8_t pin, bool state);
    uint8_t getGPIOState() const { return _gpioState; }      // New methods for mode management
    PCF8574Mode getMode() const;
    void setMode(PCF8574Mode mode);
    bool isOutputMode() const;
    bool isInputMode() const;
    void initializeOutputs(); // Initialize all outputs to LOW

private:
    TwoWire* wire;
    uint8_t _address;
    uint8_t _gpioState;
    PCF8574Mode _mode;
    std::map<std::string, float> lastSensorValues; // Ensure lastSensorValues is declared
};

#endif // PCF8574GPIO_H
