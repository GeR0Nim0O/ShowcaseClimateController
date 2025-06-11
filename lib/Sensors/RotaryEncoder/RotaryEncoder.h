#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "Device.h"
#include "I2CHandler.h"
#include <map>
#include <string>

// DFRobot Visual Rotary Encoder Register addresses
#define VISUAL_ROTARY_ENCODER_DEFAULT_I2C_ADDR            0x54
#define VISUAL_ROTARY_ENCODER_PID                         0x01F6

// Register definitions
#define VISUAL_ROTARY_ENCODER_PID_MSB_REG                 0x00
#define VISUAL_ROTARY_ENCODER_PID_LSB_REG                 0x01
#define VISUAL_ROTARY_ENCODER_VID_MSB_REG                 0x02
#define VISUAL_ROTARY_ENCODER_VID_LSB_REG                 0x03
#define VISUAL_ROTARY_ENCODER_VERSION_MSB_REG             0x04
#define VISUAL_ROTARY_ENCODER_VERSION_LSB_REG             0x05
#define VISUAL_ROTARY_ENCODER_ADDR_REG                    0x07
#define VISUAL_ROTARY_ENCODER_COUNT_MSB_REG               0x08
#define VISUAL_ROTARY_ENCODER_COUNT_LSB_REG               0x09
#define VISUAL_ROTARY_ENCODER_KEY_STATUS_REG              0x0A
#define VISUAL_ROTARY_ENCODER_GAIN_REG                    0x0B

// Error codes
#define NO_ERR                                            0
#define ERR_DATA_BUS                                      -1
#define ERR_IC_VERSION                                    -2

class RotaryEncoder : public Device {
public:
    // Structure for basic device information
    struct BasicInfo {
        uint16_t PID;        // Product ID
        uint16_t VID;        // Vendor ID  
        uint16_t version;    // Firmware version
        uint8_t i2cAddr;     // I2C address
    };

    RotaryEncoder(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    std::map<String, String> readData() override;
    
    // Core encoder methods
    uint16_t getEncoderValue();
    void setEncoderValue(uint16_t value);
    
    // Button methods
    bool detectButtonDown();
    
    // Configuration methods
    uint8_t getGainCoefficient();
    void setGainCoefficient(uint8_t gainValue);
    void refreshBasicInfo();
    
    // Get device information
    BasicInfo getBasicInfo() const { return basicInfo; }
    uint8_t getAddress() const { return _address; }

private:
    uint8_t _address;
    uint16_t _currentValue;
    uint16_t _lastValue;
    bool _buttonPressed;
    bool _lastButtonPressed;
    BasicInfo basicInfo;
    
    // I2C communication methods
    bool writeRegister(uint8_t reg, uint8_t value);
    bool writeRegisterMulti(uint8_t reg, uint8_t* pBuf, size_t size);
    uint8_t readRegister(uint8_t reg);
    bool readRegisterMulti(uint8_t reg, uint8_t* pBuf, size_t size);
    uint16_t readRegister16(uint8_t reg);
};

#endif // ROTARY_ENCODER_H
