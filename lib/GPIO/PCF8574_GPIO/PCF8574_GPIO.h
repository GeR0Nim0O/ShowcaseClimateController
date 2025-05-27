#ifndef PCF8574_GPIO_H
#define PCF8574_GPIO_H

#include "Device.h"

#define PCF8574_DEFAULT_ADDRESS 0x20

class PCF8574_GPIO : public Device {
public:
    PCF8574_GPIO(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    
    // GPIO control methods
    bool digitalWrite(uint8_t pin, bool state);
    bool digitalRead(uint8_t pin);
    bool write8(uint8_t value);
    uint8_t read8();
    
    // Climate control specific methods
    bool setTemperatureEnable(bool enable);
    bool setTemperatureHeat(bool enable);
    bool setTemperatureCool(bool enable);
    bool setHumidify(bool enable);
    bool setDehumidify(bool enable);
    bool setFanInterior(bool enable);
    bool setFanExterior(bool enable);

private:
    uint8_t currentState;
    bool writeToDevice(uint8_t data);
    uint8_t readFromDevice();
};

#endif // PCF8574_GPIO_H
