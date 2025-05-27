#include "GP8403dac.h"
#include "I2CHandler.h"

// GP8403 DAC Constants
#define DAC_MAX_VALUE 32767  // 15-bit resolution
#define DAC_MAX_VOLTAGE 10.0 // 0-10V output range

GP8403dac::GP8403dac(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(i2cAddress, tcaChannel, deviceName, deviceIndex),
      wire(&Wire), channelAValue(0), channelBValue(0), vrefValue(32767),
      gain2xA(false), gain2xB(false) {
    type = "GP8403dac";
}

GP8403dac::GP8403dac(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, i2cChannel, tcaPort, threshold, channels, deviceIndex),
      channelAValue(0), channelBValue(0), vrefValue(32767),
      gain2xA(false), gain2xB(false) {
    type = "GP8403dac";
}

bool GP8403dac::begin() {
    I2CHandler::selectTCA(getTCAChannel());
    
    wire->beginTransmission(getI2CAddress());
    bool connected = (wire->endTransmission() == 0);
    
    if (!connected) {
        Serial.println("GP8403 DAC Module not found!");
        return false;
    }
    
    // Initialize configuration
    if (!writeConfig()) {
        Serial.println("Failed to configure DAC");
        return false;
    }
    
    // Set both channels to 0V
    setBothChannels(0, 0);
    
    Serial.println("GP8403 DAC Module initialized successfully");
    return true;
}

bool GP8403dac::isConnected() {
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(getI2CAddress());
    return (wire->endTransmission() == 0);
}

void GP8403dac::update() {
    // Check if DAC is ready
    if (!isReady()) {
        Serial.println("DAC Module busy or not ready");
    }
}

std::map<String, String> GP8403dac::readData() {
    std::map<String, String> data;
    
    // Read current channel values
    data["channelA_raw"] = String(channelAValue);
    data["channelB_raw"] = String(channelBValue);
    data["channelA_voltage"] = String(dacToVoltage(channelAValue), 2);
    data["channelB_voltage"] = String(dacToVoltage(channelBValue), 2);
    data["vref_value"] = String(vrefValue);
    data["gain2x_A"] = gain2xA ? "true" : "false";
    data["gain2x_B"] = gain2xB ? "true" : "false";
    data["ready"] = isReady() ? "true" : "false";
    data["connected"] = isConnected() ? "true" : "false";
    
    return data;
}

bool GP8403dac::setChannelA(uint16_t value) {
    if (value > DAC_MAX_VALUE) {
        value = DAC_MAX_VALUE;
    }
    
    if (writeRegister(DAC_REG_CHANNEL_A, value)) {
        channelAValue = value;
        return true;
    }
    return false;
}

bool GP8403dac::setChannelB(uint16_t value) {
    if (value > DAC_MAX_VALUE) {
        value = DAC_MAX_VALUE;
    }
    
    if (writeRegister(DAC_REG_CHANNEL_B, value)) {
        channelBValue = value;
        return true;
    }
    return false;
}

bool GP8403dac::setChannelVoltage(uint8_t channel, float voltage) {
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

bool GP8403dac::setBothChannels(uint16_t valueA, uint16_t valueB) {
    bool successA = setChannelA(valueA);
    bool successB = setChannelB(valueB);
    return successA && successB;
}

bool GP8403dac::setTemperaturePower(float percentage) {
    if (percentage < 0.0) percentage = 0.0;
    if (percentage > 100.0) percentage = 100.0;
    
    float voltage = (percentage / 100.0) * DAC_MAX_VOLTAGE;
    return setChannelVoltage(0, voltage); // Channel A for temperature
}

bool GP8403dac::setHumidityPower(float percentage) {
    if (percentage < 0.0) percentage = 0.0;
    if (percentage > 100.0) percentage = 100.0;
    
    float voltage = (percentage / 100.0) * DAC_MAX_VOLTAGE;
    return setChannelVoltage(1, voltage); // Channel B for humidity
}

bool GP8403dac::setGain(uint8_t channel, bool gain2x) {
    if (channel == 0) {
        gain2xA = gain2x;
    } else if (channel == 1) {
        gain2xB = gain2x;
    } else {
        return false;
    }
    
    return writeConfig();
}

bool GP8403dac::setVRef(uint16_t vref) {
    if (writeRegister(DAC_REG_VREF, vref)) {
        vrefValue = vref;
        return true;
    }
    return false;
}

bool GP8403dac::isReady() {
    uint16_t config = readRegister(DAC_REG_CONFIG);
    return (config & DAC_CONFIG_READY) != 0;
}

bool GP8403dac::writeRegister(uint8_t reg, uint16_t value) {
    I2CHandler::selectTCA(getTCAChannel());
    
    wire->beginTransmission(getI2CAddress());
    wire->write(reg);
    wire->write((value >> 8) & 0xFF); // MSB
    wire->write(value & 0xFF);        // LSB
    return (wire->endTransmission() == 0);
}

uint16_t GP8403dac::readRegister(uint8_t reg) {
    I2CHandler::selectTCA(getTCAChannel());
    
    wire->beginTransmission(getI2CAddress());
    wire->write(reg);
    if (wire->endTransmission() != 0) {
        return 0;
    }
    
    wire->requestFrom(getI2CAddress(), (uint8_t)2);
    if (wire->available() >= 2) {
        uint16_t value = wire->read() << 8; // MSB
        value |= wire->read();              // LSB
        return value;
    }
    
    return 0;
}

bool GP8403dac::writeConfig() {
    uint16_t config = DAC_CONFIG_READY;
    
    if (gain2xA) config |= (DAC_CONFIG_GAIN_2X << 0);
    if (gain2xB) config |= (DAC_CONFIG_GAIN_2X << 1);
    
    return writeRegister(DAC_REG_CONFIG, config);
}

uint16_t GP8403dac::voltageToDAC(float voltage) {
    if (voltage < 0.0) voltage = 0.0;
    if (voltage > DAC_MAX_VOLTAGE) voltage = DAC_MAX_VOLTAGE;
    
    return (uint16_t)((voltage / DAC_MAX_VOLTAGE) * DAC_MAX_VALUE);
}

float GP8403dac::dacToVoltage(uint16_t dacValue) {
    return ((float)dacValue / DAC_MAX_VALUE) * DAC_MAX_VOLTAGE;
}
