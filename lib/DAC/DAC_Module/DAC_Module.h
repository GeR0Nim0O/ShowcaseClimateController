#ifndef DAC_MODULE_H
#define DAC_MODULE_H

#include "Device.h"

#define MCP4725_DEFAULT_ADDRESS 0x62

class DAC_Module : public Device {
public:
    DAC_Module(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    
    // DAC control methods
    bool setVoltage(float voltage);
    bool setVoltagePercent(float percent);
    bool setRawValue(uint16_t value);
    
    // Temperature/Power control specific methods
    bool setTemperaturePower(float powerPercent);
    float getCurrentVoltage() const { return currentVoltage; }
    uint16_t getCurrentRawValue() const { return currentRawValue; }
    
    // Configuration
    void setReferenceVoltage(float vref) { referenceVoltage = vref; }
    float getReferenceVoltage() const { return referenceVoltage; }

private:
    float referenceVoltage;
    float currentVoltage;
    uint16_t currentRawValue;
    
    bool writeDAC(uint16_t value);
    bool writeEEPROM(uint16_t value);
    uint16_t readDAC();
};

#endif // DAC_MODULE_H
