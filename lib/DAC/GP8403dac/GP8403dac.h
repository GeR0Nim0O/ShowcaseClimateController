#ifndef GP8403DAC_H
#define GP8403DAC_H

#include "Device.h"

// Default I2C address for GP8403
#define DAC_DEFAULT_ADDRESS 0x58

// Register addresses for GP8403 DAC - Based on actual DFRobot library
#define GP8403_CONFIG_CURRENT_REG   0x02
#define GP8403_STORE_TIMING_HEAD    0x02
#define GP8403_STORE_TIMING_ADDR    0x10
#define GP8403_STORE_TIMING_CMD1    0x03
#define GP8403_STORE_TIMING_CMD2    0x00

// Output range register and values - Based on actual DFRobot implementation
#define OUTPUT_RANGE            0x01
#define OUTPUT_RANGE_5V         0x00    // 0-5V output range (corrected back to 0x00)
#define OUTPUT_RANGE_10V        0x11    // 0-10V output range

// Channel addresses - Using the actual DFRobot register format
#define GP8403_CHANNEL_A        0x02
#define GP8403_CHANNEL_B        0x03

// DAC Configuration bits
#define DAC_CONFIG_READY        0x80
#define DAC_CONFIG_BUSY         0x00
#define DAC_GAIN_1X             0x00
#define DAC_GAIN_2X             0x01

// These are kept for backward compatibility with existing code
#define DAC_REG_CONFIG          0x00
#define DAC_REG_CHANNEL_A       0x01
#define DAC_REG_CHANNEL_B       0x02
#define DAC_REG_VREF            0x03

class GP8403dac : public Device {
public:    GP8403dac(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex);
    GP8403dac(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    std::map<String, String> readData() override;
    
    // Override pure virtual methods from Device
    std::map<String, String> getChannels() const override { return channels; }
    float getThreshold(const String& channelKey) const override { return threshold; }
      
    // Universal DAC control methods - no domain-specific names
    bool setChannelA(uint16_t value);
    bool setChannelB(uint16_t value);
    bool setChannelVoltage(uint8_t channel, float voltage);
    bool setBothChannels(uint16_t valueA, uint16_t valueB);
    
    // Configuration
    bool setGain(uint8_t channel, bool gain2x);
    bool setVRef(uint16_t vref);
    
    // Status
    uint16_t getChannelA() const { return channelAValue; }
    uint16_t getChannelB() const { return channelBValue; }
    bool isReady();
      // Validation methods
    bool validateDAC();
    bool validateDACGentle();
    bool initializeLimitedMode();

private:
    uint16_t channelAValue;
    uint16_t channelBValue;
    uint16_t vrefValue;
    bool gain2xA;
    bool gain2xB;
    
    bool writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
    bool writeConfig();
    uint16_t voltageToDAC(float voltage);
    float dacToVoltage(uint16_t dacValue);
};

#endif // GP8403DAC_H
