#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "Device.h"

// I2CEncoderV2.1 Register addresses
#define I2C_ENCODER_GCONF      0x00
#define I2C_ENCODER_GP1CONF    0x01
#define I2C_ENCODER_GP2CONF    0x02
#define I2C_ENCODER_GP3CONF    0x03
#define I2C_ENCODER_INTCONF    0x04
#define I2C_ENCODER_ESTATUS    0x05
#define I2C_ENCODER_I2STATUS   0x06
#define I2C_ENCODER_FSTATUS    0x07
#define I2C_ENCODER_CVAL       0x08
#define I2C_ENCODER_CMAX       0x0C
#define I2C_ENCODER_CMIN       0x10
#define I2C_ENCODER_ISTEP      0x14
#define I2C_ENCODER_RLED       0x18
#define I2C_ENCODER_GLED       0x19
#define I2C_ENCODER_BLED       0x1A
#define I2C_ENCODER_GP1REG     0x1B
#define I2C_ENCODER_GP2REG     0x1C
#define I2C_ENCODER_GP3REG     0x1D
#define I2C_ENCODER_ANTBOUNC   0x1E
#define I2C_ENCODER_DPPERIOD   0x1F
#define I2C_ENCODER_FADERGB    0x20
#define I2C_ENCODER_FADEGP     0x21

// Configuration bits
#define GCONF_FLOAT_DATA       0x01
#define GCONF_INT_DATA         0x00
#define GCONF_WRAP_ENABLE      0x02
#define GCONF_WRAP_DISABLE     0x00
#define GCONF_DIRE_LEFT        0x04
#define GCONF_DIRE_RIGHT       0x00
#define GCONF_IPUP_DISABLE     0x08
#define GCONF_IPUP_ENABLE      0x00
#define GCONF_RMOD_X2          0x10
#define GCONF_RMOD_X1          0x00
#define GCONF_RGB_ENCODER      0x20
#define GCONF_STD_ENCODER      0x00
#define GCONF_EEPROM_BANK1     0x40
#define GCONF_EEPROM_BANK0     0x00
#define GCONF_RESET            0x80

class RotaryEncoder : public Device {
public:
    RotaryEncoder(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex);
      bool begin() override;
    bool isConnected() override;
    void update() override;
    std::map<String, String> readData() override;
    
    // Encoder reading methods
    int32_t getPosition();
    void setPosition(int32_t pos);
    int32_t getPositionChange();
    
    // Button methods
    bool isButtonPressed();
    bool wasButtonPressed();
    bool isButtonHeld(unsigned long holdTime = 1000);
    
    // Configuration
    void setMinMax(int32_t minVal, int32_t maxVal);
    void setStepSize(int32_t step);
    void enableWrap(bool enable);
    void setDirection(bool clockwiseIncrement);
    
    // LED control (if RGB encoder)
    void setLED(uint8_t red, uint8_t green, uint8_t blue);
    void setLEDFade(uint8_t fade);

private:
    int32_t position;
    int32_t lastPosition;
    int32_t minValue, maxValue;
    int32_t stepValue;
    bool wrapEnabled;
    bool floatData;
    bool rgbEncoder;
    
    // Button state
    bool buttonPressed;
    bool lastButtonPressed;
    bool buttonHeld;
    unsigned long buttonPressTime;
    
    bool writeRegister(uint8_t reg, uint8_t value);
    bool writeRegister32(uint8_t reg, int32_t value);
    uint8_t readRegister(uint8_t reg);
    int32_t readRegister32(uint8_t reg);
    bool writeConfig();
};

#endif // ROTARY_ENCODER_H
