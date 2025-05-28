#include "GP8403dac.h"
#include "I2CHandler.h"

// GP8403 DAC Constants
#define DAC_MAX_VALUE 32767  // 15-bit resolution (Official library uses this)
#define DAC_MAX_VOLTAGE 5.0  // 0-5V output range

// GP8403 Register addresses - from official DFRobot library
#define REG_DAC_A     0x01   // Write to DAC channel A  
#define REG_DAC_B     0x02   // Write to DAC channel B
#define REG_CONFIG    0x03   // Configuration register
#define REG_SYNC_ALL  0x04   // Synchronized update both channels

GP8403dac::GP8403dac(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(i2cAddress, tcaChannel, deviceName, deviceIndex),
      channelAValue(0), channelBValue(0), vrefValue(32767),
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
    
    Wire.beginTransmission(getI2CAddress());
    bool connected = (Wire.endTransmission() == 0);
    
    if (!connected) {
        Serial.println("GP8403 DAC Module not found!");
        return false;
    }
    
    // Initialize DAC - based on official library
    delay(10);
    
    // Set both channels to 0V
    setBothChannels(0, 0);
    
    // Set output range to 0-5V
    setGain(0, false); // Channel A, 1x gain
    setGain(1, false); // Channel B, 1x gain
    
    Serial.println("GP8403 DAC Module initialized successfully");
    initialized = true; // Set initialized flag to true
    return true;
}

bool GP8403dac::isConnected() {
    I2CHandler::selectTCA(getTCAChannel());
    Wire.beginTransmission(getI2CAddress());
    return (Wire.endTransmission() == 0);
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
      // Using register approach like in the official library
    I2CHandler::selectTCA(getTCAChannel());
    
    Wire.beginTransmission(getI2CAddress());
    Wire.write(REG_DAC_A);
    Wire.write((value >> 8) & 0xFF); // MSB
    Wire.write(value & 0xFF);        // LSB
    
    bool success = (Wire.endTransmission() == 0);
    
    if (success) {
        channelAValue = value;
    }
    return success;
}

bool GP8403dac::setChannelB(uint16_t value) {
    if (value > DAC_MAX_VALUE) {
        value = DAC_MAX_VALUE;
    }
      // Using register approach like in the official library
    I2CHandler::selectTCA(getTCAChannel());
    
    Wire.beginTransmission(getI2CAddress());
    Wire.write(REG_DAC_B);
    Wire.write((value >> 8) & 0xFF); // MSB
    Wire.write(value & 0xFF);        // LSB
    
    bool success = (Wire.endTransmission() == 0);
    
    if (success) {
        channelBValue = value;
    }
    return success;
}

bool GP8403dac::setBothChannels(uint16_t valueA, uint16_t valueB) {
    // Using register approach like in the official library
    I2CHandler::selectTCA(getTCAChannel());
    
    // Set channel A value
    if (!setChannelA(valueA)) {
        return false;
    }
    
    // Set channel B value 
    if (!setChannelB(valueB)) {
        return false;
    }
    
    // Optionally sync channels (not required for this use case)
    return true;
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
    
    Wire.beginTransmission(getI2CAddress());
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF); // MSB
    Wire.write(value & 0xFF);        // LSB
    return (Wire.endTransmission() == 0);
}

uint16_t GP8403dac::readRegister(uint8_t reg) {
    I2CHandler::selectTCA(getTCAChannel());
    
    Wire.beginTransmission(getI2CAddress());
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
        return 0;
    }
    
    Wire.requestFrom(getI2CAddress(), (uint8_t)2);
    if (Wire.available() >= 2) {
        uint16_t value = Wire.read() << 8; // MSB
        value |= Wire.read();              // LSB
        return value;
    }
    
    return 0;
}

bool GP8403dac::writeConfig() {
    uint8_t config = 0; // Start with 0
    
    // Set gain bits
    if (gain2xA) config |= 0x01;
    if (gain2xB) config |= 0x02;
      I2CHandler::selectTCA(getTCAChannel());
    
    Wire.beginTransmission(getI2CAddress());
    Wire.write(REG_CONFIG);
    Wire.write(config);
    return (Wire.endTransmission() == 0);
}

uint16_t GP8403dac::voltageToDAC(float voltage) {
    if (voltage < 0.0) voltage = 0.0;
    if (voltage > DAC_MAX_VOLTAGE) voltage = DAC_MAX_VOLTAGE;
    
    return (uint16_t)((voltage / DAC_MAX_VOLTAGE) * DAC_MAX_VALUE);
}

float GP8403dac::dacToVoltage(uint16_t dacValue) {
    return ((float)dacValue / DAC_MAX_VALUE) * DAC_MAX_VOLTAGE;
}

bool GP8403dac::setChannelVoltage(uint8_t channel, float voltage) {
    if (voltage < 0.0 || voltage > DAC_MAX_VOLTAGE) {
        Serial.print("Voltage out of range (0-");
        Serial.print(DAC_MAX_VOLTAGE);
        Serial.println("V)!");
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
