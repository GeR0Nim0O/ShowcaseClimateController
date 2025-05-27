#include "DAC_Module.h"

// MCP4725 Commands
#define MCP4725_CMD_WRITEDAC      0x40  // Writes data to the DAC
#define MCP4725_CMD_WRITEDACEEPROM 0x60  // Writes data to the DAC and the EEPROM

DAC_Module::DAC_Module(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(i2cAddress, tcaChannel, deviceName, deviceIndex),
      referenceVoltage(3.3), currentVoltage(0.0), currentRawValue(0) {
    type = "DAC_Module";
}

bool DAC_Module::begin() {
    selectTCAChannel(tcaChannel);
    
    if (!testI2CConnection()) {
        Serial.println("DAC Module not found!");
        return false;
    }
    
    // Initialize DAC to 0V
    setRawValue(0);
    
    initialized = true;
    Serial.println("DAC Module initialized successfully");
    return true;
}

bool DAC_Module::isConnected() {
    return testI2CConnection();
}

void DAC_Module::update() {
    // DAC doesn't need periodic updates unless reading back values
    // For now, we'll just verify connection
    if (!isConnected()) {
        Serial.println("DAC Module connection lost!");
    }
}

bool DAC_Module::setVoltage(float voltage) {
    if (voltage < 0.0 || voltage > referenceVoltage) {
        Serial.println("Voltage out of range!");
        return false;
    }
    
    uint16_t rawValue = (uint16_t)((voltage / referenceVoltage) * 4095.0);
    return setRawValue(rawValue);
}

bool DAC_Module::setVoltagePercent(float percent) {
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
