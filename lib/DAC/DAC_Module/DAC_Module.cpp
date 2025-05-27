#include "DAC_Module.h"

// DFR1073 DAC Constants
#define DAC_MAX_VALUE 32767  // 15-bit resolution
#define DAC_MAX_VOLTAGE 10.0 // 0-10V output range

DAC_Module::DAC_Module(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(i2cAddress, tcaChannel, deviceName, deviceIndex),
      channelAValue(0), channelBValue(0), vrefValue(32767),
      gain2xA(false), gain2xB(false) {
    type = "DAC_Module";
}

bool DAC_Module::begin() {
    selectTCAChannel(tcaChannel);
    
    if (!testI2CConnection()) {
        Serial.println("DFR1073 DAC Module not found!");
        return false;
    }
    
    // Initialize configuration
    if (!writeConfig()) {
        Serial.println("Failed to configure DAC");
        return false;
    }
    
    // Set both channels to 0V
    setBothChannels(0, 0);
    
    initialized = true;
    Serial.println("DFR1073 DAC Module initialized successfully");
    return true;
}

bool DAC_Module::isConnected() {
    return testI2CConnection();
}

void DAC_Module::update() {
    // Check if DAC is ready
    if (!isReady()) {
        Serial.println("DAC Module busy or not ready");
    }
}

bool DAC_Module::setChannelA(uint16_t value) {
    if (value > DAC_MAX_VALUE) {
        value = DAC_MAX_VALUE;
    }
    
    if (writeRegister(DAC_REG_CHANNEL_A, value)) {
        channelAValue = value;
        return true;
    }
    return false;
}

bool DAC_Module::setChannelB(uint16_t value) {
    if (value > DAC_MAX_VALUE) {
        value = DAC_MAX_VALUE;
    }
    
    if (writeRegister(DAC_REG_CHANNEL_B, value)) {
        channelBValue = value;
        return true;
    }
    return false;
}

bool DAC_Module::setChannelVoltage(uint8_t channel, float voltage) {
    if (voltage < 0.0 || voltage > DAC_MAX_VOLTAGE) {
        Serial.println("Voltage out of range (0-10V)!");
        return false;
    }
    
    uint16_t dacValue = voltageToDAC(voltage);
    
    if (channel == 0) {
        return setChannelA(dacValue);
    } else if (channel == 1) {
        return setChannelB(dacValue);
    }
    
    return false;
}

bool DAC_Module::setBothChannels(uint16_t valueA, uint16_t valueB) {
    bool successA = setChannelA(valueA);
    bool successB = setChannelB(valueB);
    return successA && successB;
}

bool DAC_Module::setTemperaturePower(float percentage) {
    if (percentage < 0.0) percentage = 0.0;
    if (percentage > 100.0) percentage = 100.0;
    
    float voltage = (percentage / 100.0) * DAC_MAX_VOLTAGE;
    return setChannelVoltage(0, voltage); // Channel A for temperature
}

bool DAC_Module::setHumidityPower(float percentage) {
    if (percentage < 0.0) percentage = 0.0;
    if (percentage > 100.0) percentage = 100.0;
    
    float voltage = (percentage / 100.0) * DAC_MAX_VOLTAGE;
    return setChannelVoltage(1, voltage); // Channel B for humidity
}

bool DAC_Module::setGain(uint8_t channel, bool gain2x) {
    if (channel == 0) {
        gain2xA = gain2x;
    } else if (channel == 1) {
        gain2xB = gain2x;
    } else {
        return false;
    }
    
    return writeConfig();
}

bool DAC_Module::setVRef(uint16_t vref) {
    if (writeRegister(DAC_REG_VREF, vref)) {
        vrefValue = vref;
        return true;
    }
    return false;
}

bool DAC_Module::isReady() {
    uint16_t config = readRegister(DAC_REG_CONFIG);
    return (config & DAC_CONFIG_READY) != 0;
}

bool DAC_Module::writeRegister(uint8_t reg, uint16_t value) {
    selectTCAChannel(tcaChannel);
    
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF); // MSB
    Wire.write(value & 0xFF);        // LSB
    return (Wire.endTransmission() == 0);
}

uint16_t DAC_Module::readRegister(uint8_t reg) {
    selectTCAChannel(tcaChannel);
    
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
        return 0;
    }
    
    Wire.requestFrom(i2cAddress, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint16_t value = Wire.read() << 8; // MSB
        value |= Wire.read();              // LSB
        return value;
    }
    
    return 0;
}

bool DAC_Module::writeConfig() {
    uint16_t config = DAC_CONFIG_READY;
    
    if (gain2xA) config |= (DAC_CONFIG_GAIN_2X << 0);
    if (gain2xB) config |= (DAC_CONFIG_GAIN_2X << 1);
    
    return writeRegister(DAC_REG_CONFIG, config);
}

uint16_t DAC_Module::voltageToDAC(float voltage) {
    if (voltage < 0.0) voltage = 0.0;
    if (voltage > DAC_MAX_VOLTAGE) voltage = DAC_MAX_VOLTAGE;
    
    return (uint16_t)((voltage / DAC_MAX_VOLTAGE) * DAC_MAX_VALUE);
}

float DAC_Module::dacToVoltage(uint16_t dacValue) {
    return ((float)dacValue / DAC_MAX_VALUE) * DAC_MAX_VOLTAGE;
    if (percent < 0.0 || percent > 100.0) {
        Serial.println("Percentage out of range!");
        return false;
    }
    
    float voltage = (percent / 100.0) * referenceVoltage;
    return setVoltage(voltage);
}

bool DAC_Module::setRawValue(uint16_t value) {
    if (value > 4095) {
        Serial.println("Raw value out of range (0-4095)!");
        return false;
    }
    
    if (writeDAC(value)) {
        currentRawValue = value;
        currentVoltage = (float)value * referenceVoltage / 4095.0;
        return true;
    }
    
    return false;
}

bool DAC_Module::setTemperaturePower(float powerPercent) {
    // Map power percentage to voltage output for temperature control
    // This assumes the temperature control module expects 0-100% power
    // mapped to 0V to reference voltage
    
    if (powerPercent < 0.0 || powerPercent > 100.0) {
        Serial.println("Power percentage out of range!");
        return false;
    }
    
    return setVoltagePercent(powerPercent);
}

bool DAC_Module::writeDAC(uint16_t value) {
    selectTCAChannel(tcaChannel);
    
    Wire.beginTransmission(i2cAddress);
    Wire.write(MCP4725_CMD_WRITEDAC);
    Wire.write(value >> 4);                 // Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
    Wire.write((value & 0xF) << 4);         // Lower data bits (D3.D2.D1.D0.x.x.x.x)
    
    return (Wire.endTransmission() == 0);
}

bool DAC_Module::writeEEPROM(uint16_t value) {
    selectTCAChannel(tcaChannel);
    
    Wire.beginTransmission(i2cAddress);
    Wire.write(MCP4725_CMD_WRITEDACEEPROM);
    Wire.write(value >> 4);                 // Upper data bits
    Wire.write((value & 0xF) << 4);         // Lower data bits
    
    return (Wire.endTransmission() == 0);
}

uint16_t DAC_Module::readDAC() {
    selectTCAChannel(tcaChannel);
    
    Wire.requestFrom(i2cAddress, (uint8_t)3);
    
    if (Wire.available() >= 3) {
        uint8_t byte1 = Wire.read();
        uint8_t byte2 = Wire.read();
        uint8_t byte3 = Wire.read();
        
        // Extract 12-bit DAC value
        uint16_t value = ((byte1 & 0x0F) << 8) | byte2;
        return value;
    }
    
    return 0;
}
