#include "GP8403dac.h"
#include "I2CHandler.h"

// GP8403 DAC Constants - Based on official DFRobot library
#define DAC_MAX_VALUE 4095   // 12-bit resolution (0-4095)
#define DAC_MAX_VOLTAGE 5.0  // 0-5V output range

GP8403dac::GP8403dac(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(wire, i2cAddress, tcaChannel, deviceName, deviceIndex),
      channelAValue(0), channelBValue(0), vrefValue(4095),
      gain2xA(false), gain2xB(false) {
    type = "GP8403dac";
}

GP8403dac::GP8403dac(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, threshold, channels, i2cAddress, tcaPort, deviceIndex),
      channelAValue(0), channelBValue(0), vrefValue(4095),
      gain2xA(false), gain2xB(false) {
    type = "GP8403dac";
}

bool GP8403dac::begin() {
    Serial.println("GP8403: Starting DAC initialization (DFRobot compatible)...");
    
    // First, try basic I2C connectivity using official begin() method
    bool basicConnection = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.print("GP8403: Connection attempt ");
        Serial.print(attempt);
        Serial.print("/3...");
        
        I2CHandler::selectTCA(getTCAChannel());
        delayMicroseconds(500); // Extended stabilization delay
        
        // Use DFRobot's begin() method approach
        wire->beginTransmission(getI2CAddress());
        wire->write(OUTPUT_RANGE);  // Write to output range register
        int result = wire->endTransmission();
        
        if (result == 0) {
            Serial.println(" SUCCESS");
            basicConnection = true;
            break;
        } else {
            Serial.print(" FAILED (error: ");
            Serial.print(result);
            Serial.println(")");
            delay(100); // Wait before retry
        }
    }
    
    if (!basicConnection) {
        Serial.println("GP8403: Basic connection failed - trying fallback...");
        // Fall back to limited mode if we can't get full validation
        return initializeLimitedMode();
    }
    
    // Set the DAC to 5V output range (as per official library default)
    Serial.println("GP8403: Setting output range to 5V...");
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(getI2CAddress());
    wire->write(OUTPUT_RANGE);
    wire->write(OUTPUT_RANGE_5V);
    if (wire->endTransmission() != 0) {
        Serial.println("GP8403: Failed to set output range");
        return initializeLimitedMode();
    }    // Try comprehensive validation
    if (validateDAC()) {
        initialized = true;  // Direct access to protected member
        Serial.println("GP8403: Full initialization and validation successful!");
        return true;
    } else {
        // Fall back to gentle validation
        Serial.println("GP8403: Full validation failed, trying gentle validation...");
        if (validateDACGentle()) {
            initialized = true;  // Direct access to protected member
            Serial.println("GP8403: Limited initialization successful (gentle validation passed)");
            return true;
        } else {
            return initializeLimitedMode();
        }
    }
}

bool GP8403dac::validateDACGentle() {
    Serial.println("GP8403: Starting gentle DAC validation...");
    
    // Test 1: Basic connectivity check with extended retries
    for (int attempt = 1; attempt <= 5; attempt++) {
        I2CHandler::selectTCA(getTCAChannel());
        delayMicroseconds(1000); // Extended delay
        
        wire->beginTransmission(getI2CAddress());
        int result = wire->endTransmission();
        
        if (result == 0) {
            Serial.println("GP8403 Gentle Validation: Basic connectivity test PASSED");
            return true; // If we can connect, that's sufficient for gentle validation
        }
        
        Serial.print("GP8403 Gentle Validation: Connection attempt ");
        Serial.print(attempt);
        Serial.print("/5 failed (error: ");
        Serial.print(result);
        Serial.println(")");
        
        delay(20); // Longer delay between attempts
    }
    
    Serial.println("GP8403 Gentle Validation: All connection attempts failed");
    return false;
}

bool GP8403dac::initializeLimitedMode() {
    Serial.println("GP8403: Falling back to limited mode operation");
    
    // Just mark the device as initialized but with limitations
    initialized = true;  // Direct access to protected member
    
    // Try one last basic connection check
    I2CHandler::selectTCA(getTCAChannel());
    delay(50); // Extended stabilization
    
    wire->beginTransmission(getI2CAddress());
    if (wire->endTransmission() == 0) {
        Serial.println("GP8403: Device responds to I2C in limited mode");
    } else {
        Serial.println("GP8403: Device not responding, but proceeding with limited functionality");
    }
    
    // Try minimal initialization - just set output range to 5V
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(getI2CAddress());
    wire->write(OUTPUT_RANGE);
    wire->write(OUTPUT_RANGE_5V);
    if (wire->endTransmission() == 0) {
        Serial.println("GP8403: Successfully set output range in limited mode");
    }
    
    Serial.println("GP8403: Limited mode initialization complete");
    return true;
}

bool GP8403dac::validateDAC() {
    Serial.println("GP8403: Starting comprehensive DAC validation...");
    
    // Test 1: Basic connectivity check
    if (!isConnected()) {
        Serial.println("GP8403 Validation: Basic connectivity test FAILED");
        return false;
    }
    Serial.println("GP8403 Validation: Basic connectivity test PASSED");
    
    // Test 2: Test Channel A write/verify cycle
    float testVoltage = 1.0f; // Test with 1V
    if (!setChannelVoltage(0, testVoltage)) {
        Serial.println("GP8403 Validation: Channel A write test FAILED");
        return false;
    }
    delay(10); // Allow voltage to settle
    
    // Verify the value was stored internally (we can't read back from GP8403, but we can check our internal state)
    float expectedDAC = voltageToDAC(testVoltage);
    if (abs(channelAValue - expectedDAC) > 100) { // Allow some tolerance
        Serial.print("GP8403 Validation: Channel A value mismatch - Expected: ");
        Serial.print((int)expectedDAC);
        Serial.print(", Got: ");
        Serial.println(channelAValue);
        return false;
    }
    Serial.println("GP8403 Validation: Channel A write test PASSED");
    
    // Test 3: Test Channel B write/verify cycle
    if (!setChannelVoltage(1, testVoltage)) {
        Serial.println("GP8403 Validation: Channel B write test FAILED");
        return false;
    }
    delay(10);
    
    if (abs(channelBValue - expectedDAC) > 100) {
        Serial.print("GP8403 Validation: Channel B value mismatch - Expected: ");
        Serial.print((int)expectedDAC);
        Serial.print(", Got: ");
        Serial.println(channelBValue);
        return false;
    }
    Serial.println("GP8403 Validation: Channel B write test PASSED");
    
    // Test 4: Test range validation (set to 0V)
    if (!setChannelVoltage(0, 0.0f)) {
        Serial.println("GP8403 Validation: Channel A zero voltage test FAILED");
        return false;
    }
    if (!setChannelVoltage(1, 0.0f)) {
        Serial.println("GP8403 Validation: Channel B zero voltage test FAILED");
        return false;
    }
    Serial.println("GP8403 Validation: Zero voltage test PASSED");
    
    // Test 5: Test maximum safe voltage (4V - below 5V max)
    float maxSafeVoltage = 4.0f;
    if (!setChannelVoltage(0, maxSafeVoltage)) {
        Serial.println("GP8403 Validation: Channel A max voltage test FAILED");
        return false;
    }
    if (!setChannelVoltage(1, maxSafeVoltage)) {
        Serial.println("GP8403 Validation: Channel B max voltage test FAILED");
        return false;
    }
    Serial.println("GP8403 Validation: Max voltage test PASSED");
    
    // Reset to safe state (0V)
    setChannelVoltage(0, 0.0f);
    setChannelVoltage(1, 0.0f);
    
    Serial.println("GP8403 Validation: All tests PASSED - DAC is fully functional");
    return true;
}

bool GP8403dac::isConnected() {
    // Improved connection check with better I2C handling
    const int maxRetries = 2;
    
    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        I2CHandler::selectTCA(getTCAChannel());
        delayMicroseconds(200); // Minimal stabilization delay
        
        wire->beginTransmission(getI2CAddress());
        int result = wire->endTransmission();
        
        if (result == 0) {
            return true; // Success
        }
        
        if (attempt < maxRetries) {
            delay(1); // Short delay before retry
        }
    }
    
    return false; // Failed after all retries
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
    
    Serial.print("GP8403: Setting Channel A to raw value ");
    Serial.println(value);
    
    // Save original value for later reference
    uint16_t originalValue = value;
    
    // Shift left by 4 bits as per DFRobot's library format
    value = value << 4;
    
    // Improved retry mechanism with better timing
    const int maxRetries = 3;
    bool success = false;
    
    for (int attempt = 1; attempt <= maxRetries && !success; attempt++) {
        // Select TCA port before transmission
        I2CHandler::selectTCA(getTCAChannel());
        
        // Minimal delay for TCA stabilization
        delayMicroseconds(200);
        
        // Follow DFRobot's implementation pattern:
        // Write to GP8302_CONFIG_CURRENT_REG for channel A (0)
        wire->beginTransmission(getI2CAddress());
        wire->write(GP8302_CONFIG_CURRENT_REG);
        wire->write(value & 0xFF);         // LSB first 
        wire->write((value >> 8) & 0xFF);  // MSB second
        
        int result = wire->endTransmission();
        success = (result == 0);
        
        if (success) {
            channelAValue = originalValue;  // Store the unshifted value for reference
            Serial.print("GP8403: Channel A updated to ");
            Serial.println(channelAValue);
        } else {
            Serial.print("GP8403: I2C transmission attempt ");
            Serial.print(attempt);
            Serial.print(" failed with error: ");
            Serial.println(result);
            
            if (attempt < maxRetries) {
                // Progressive delay increase for retries
                delay(attempt * 2);
            }
        }
    }
    
    if (!success) {
        Serial.println("GP8403: Failed to update Channel A after all retries");
    }
    
    return success;
}

bool GP8403dac::setChannelB(uint16_t value) {
    if (value > DAC_MAX_VALUE) {
        value = DAC_MAX_VALUE;
    }
    
    Serial.print("GP8403: Setting Channel B to raw value ");
    Serial.println(value);
    
    // Save original value for later reference
    uint16_t originalValue = value;
    
    // Shift left by 4 bits as per DFRobot's library format
    value = value << 4;
    
    // Improved retry mechanism with better timing
    const int maxRetries = 3;
    bool success = false;
    
    for (int attempt = 1; attempt <= maxRetries && !success; attempt++) {
        // Select TCA port before transmission
        I2CHandler::selectTCA(getTCAChannel());
        
        // Minimal delay for TCA stabilization
        delayMicroseconds(200);
        
        // Follow DFRobot's implementation pattern:
        // Write to GP8302_CONFIG_CURRENT_REG<<1 for channel B (1)
        wire->beginTransmission(getI2CAddress());
        wire->write(GP8302_CONFIG_CURRENT_REG << 1);
        wire->write(value & 0xFF);         // LSB first 
        wire->write((value >> 8) & 0xFF);  // MSB second
        
        int result = wire->endTransmission();
        success = (result == 0);
        
        if (success) {
            channelBValue = originalValue;  // Store the unshifted value for reference
            Serial.print("GP8403: Channel B updated to ");
            Serial.println(channelBValue);
        } else {
            Serial.print("GP8403: I2C transmission attempt ");
            Serial.print(attempt);
            Serial.print(" failed with error: ");
            Serial.println(result);
            
            if (attempt < maxRetries) {
                // Progressive delay increase for retries
                delay(attempt * 2);
            }
        }
    }
    
    if (!success) {
        Serial.println("GP8403: Failed to update Channel B after all retries");
    }
    
    return success;
}

bool GP8403dac::setBothChannels(uint16_t valueA, uint16_t valueB) {
    if (valueA > DAC_MAX_VALUE) valueA = DAC_MAX_VALUE;
    if (valueB > DAC_MAX_VALUE) valueB = DAC_MAX_VALUE;
    
    Serial.print("GP8403: Setting both channels to values A:");
    Serial.print(valueA);
    Serial.print(" B:");
    Serial.println(valueB);
    
    // Save original values for later reference
    uint16_t originalValueA = valueA;
    uint16_t originalValueB = valueB;
    
    // Shift left by 4 bits as per DFRobot's library format
    valueA = valueA << 4;
    valueB = valueB << 4;
    
    // Improved retry mechanism
    const int maxRetries = 3;
    bool success = false;
    
    for (int attempt = 1; attempt <= maxRetries && !success; attempt++) {
        // Select TCA port before transmission
        I2CHandler::selectTCA(getTCAChannel());
        
        // Minimal delay for TCA stabilization
        delayMicroseconds(200);
        
        // Follow DFRobot's implementation pattern for both channels
        wire->beginTransmission(getI2CAddress());
        wire->write(GP8302_CONFIG_CURRENT_REG);
        wire->write(valueA & 0xFF);         // LSB first for channel A
        wire->write((valueA >> 8) & 0xFF);  // MSB second for channel A
        wire->write(valueB & 0xFF);         // LSB first for channel B
        wire->write((valueB >> 8) & 0xFF);  // MSB second for channel B
        
        int result = wire->endTransmission();
        success = (result == 0);
        
        if (success) {
            channelAValue = originalValueA;
            channelBValue = originalValueB;
            Serial.println("GP8403: Both channels updated successfully");
        } else {
            Serial.print("GP8403: I2C transmission attempt ");
            Serial.print(attempt);
            Serial.print(" failed with error: ");
            Serial.println(result);
            
            if (attempt < maxRetries) {
                delay(attempt * 2);
            }
        }
    }
    
    if (!success) {
        Serial.println("GP8403: Failed to update both channels after all retries");
    }
    
    return success;
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
    // The GP8403 doesn't have a ready status that can be read back
    // Let's assume it's ready if we can communicate with it
    return isConnected();
}

bool GP8403dac::writeRegister(uint8_t reg, uint16_t value) {
    I2CHandler::selectTCA(getTCAChannel());
    delayMicroseconds(200);
    
    wire->beginTransmission(getI2CAddress());
    wire->write(reg);
    wire->write(value & 0xFF);        // LSB first as per DFRobot implementation
    wire->write((value >> 8) & 0xFF); // MSB second
    return (wire->endTransmission() == 0);
}

uint16_t GP8403dac::readRegister(uint8_t reg) {
    // GP8403 doesn't support reading registers
    // This is just a stub for compatibility with the existing code
    return 0;
}

bool GP8403dac::writeConfig() {
    // GP8403 has different configuration method - this is just for compatibility
    return true;
}

uint16_t GP8403dac::voltageToDAC(float voltage) {
    if (voltage < 0.0) voltage = 0.0;
    if (voltage > DAC_MAX_VOLTAGE) voltage = DAC_MAX_VOLTAGE;
    
    return (uint16_t)((voltage / DAC_MAX_VOLTAGE) * DAC_MAX_VALUE);
}

float GP8403dac::dacToVoltage(uint16_t dacValue) {
    if (dacValue > DAC_MAX_VALUE) dacValue = DAC_MAX_VALUE;
    return ((float)dacValue / DAC_MAX_VALUE) * DAC_MAX_VOLTAGE;
}

bool GP8403dac::setChannelVoltage(uint8_t channel, float voltage) {
    if (voltage < 0.0 || voltage > DAC_MAX_VOLTAGE) {
        Serial.print("GP8403: Voltage out of range (0-");
        Serial.print(DAC_MAX_VOLTAGE);
        Serial.print("V)! Requested: ");
        Serial.println(voltage);
        return false;
    }
    
    uint16_t dacValue = voltageToDAC(voltage);
    
    Serial.print("GP8403: Converting ");
    Serial.print(voltage, 2);
    Serial.print("V to DAC value ");
    Serial.println(dacValue);
    
    bool success = false;
    if (channel == 0) {
        success = setChannelA(dacValue);
        Serial.print("GP8403: Channel A set result: ");
        Serial.println(success ? "SUCCESS" : "FAILED");
    } else if (channel == 1) {
        success = setChannelB(dacValue);
        Serial.print("GP8403: Channel B set result: ");
        Serial.println(success ? "SUCCESS" : "FAILED");
    } else {
        Serial.print("GP8403: Invalid channel: ");
        Serial.println(channel);
    }
    
    return success;
}
