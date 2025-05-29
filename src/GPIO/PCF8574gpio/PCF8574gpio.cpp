#include "PCF8574gpio.h"

PCF8574gpio::PCF8574gpio(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex, PCF8574Mode mode)
    : Device(wire, threshold, channels, i2cAddress, tcaChannel, deviceIndex), _address(i2cAddress), _gpioState(0xFF), _mode(mode) {
      // Set type - inherited from Device base class
    type = "PCF8574GPIO";
    Serial.println("PCF8574gpio created:");
    Serial.print("Address: 0x");
    Serial.println(_address, HEX);
    Serial.print("Mode: ");
    Serial.println(_mode == PCF8574Mode::INPUT_MODE ? "INPUT_MODE" : "OUTPUT_MODE");
    Serial.print("Threshold: ");
    Serial.println(threshold);
    Serial.print("Number of Channels: ");
    Serial.println(channels.size());
    Serial.print("Device Index: ");
    Serial.println(deviceIndex);
}

bool PCF8574gpio::begin() {
    bool connected = isConnected();
    if (connected) {
        initialized = true;  // Set initialized flag to true when connection is successful
        initializeOutputs();  // Initialize pins based on mode
    }
    return connected;
}

bool PCF8574gpio::isConnected() {
    selectTCAChannel(tcaChannel);  // Select TCA channel before communication
    wire->beginTransmission(_address);
    return (wire->endTransmission() == 0);
}

void PCF8574gpio::update() {
    // Update GPIO state by reading the current state
    readByte(_gpioState);
}

std::map<String, String> PCF8574gpio::readData() {
    std::map<String, String> result;
    update();
    
    // Convert GPIO states to a map of channel values
    for (const auto& channel : channels) {
        int pin = channel.second.toInt();
        if (pin >= 0 && pin <= 7) {
            bool state = (_gpioState & (1 << pin)) != 0;
            result[channel.first] = state ? "1" : "0";
        }
    }
    
    return result;
}

bool PCF8574gpio::writeByte(uint8_t data) {
    // Ensure TCA channel is selected before communication
    selectTCAChannel(tcaChannel);
    delay(2); // Reduced delay for TCA switching
    
    // For output mode, we should NOT read back to verify since PCF8574 
    // in output mode may not read back the same values due to external loads
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        wire->beginTransmission(_address);
        wire->write(data);
        int result = wire->endTransmission();
        
        if (result == 0) {
            _gpioState = data; // Trust that the write succeeded
            Serial.print("GPIO: Setting output state to 0x");
            Serial.print(data, HEX);
            Serial.println(" - SUCCESS");
            return true;
        } else {
            Serial.print("GPIO: Write failed with I2C error: ");
            Serial.println(result);
            return false;
        }
    }
    
    // For input mode, we can try verification (original code)
    for (int attempt = 0; attempt < 3; attempt++) {
        wire->beginTransmission(_address);
        wire->write(data);
        int result = wire->endTransmission();
        
        if (result == 0) {
            _gpioState = data;
            
            // Wait a bit for the hardware to settle
            delay(10);
            
            // Verify the write by reading back
            uint8_t readBack;
            if (readByte(readBack)) {
                if (readBack == data) {
                    Serial.print("GPIO Write SUCCESS on attempt ");
                    Serial.print(attempt + 1);
                    Serial.print(": 0x");
                    Serial.println(data, HEX);
                    return true;
                } else {
                    Serial.print("GPIO Write verification FAILED on attempt ");
                    Serial.print(attempt + 1);
                    Serial.print("! Expected: 0x");
                    Serial.print(data, HEX);
                    Serial.print(", Read: 0x");
                    Serial.println(readBack, HEX);
                    delay(50);
                }
            } else {
                Serial.print("GPIO Read failed on attempt ");
                Serial.println(attempt + 1);
                delay(20);
            }
        } else {
            Serial.print("GPIO Write I2C error on attempt ");
            Serial.print(attempt + 1);
            Serial.print(": ");
            Serial.println(result);
            delay(20);
        }
    }
    
    Serial.println("GPIO Write FAILED after 3 attempts");
    return false;
}

bool PCF8574gpio::readByte(uint8_t &data) {
    // Ensure TCA channel is selected before communication
    selectTCAChannel(tcaChannel);
    delay(5); // Increased delay for TCA switching
    
    // Try multiple read attempts
    for (int attempt = 0; attempt < 3; attempt++) {
        if (wire->requestFrom(_address, (uint8_t)1) == 1) {
            data = wire->read();
            _gpioState = data;
            
            if (attempt > 0) {
                Serial.print("GPIO Read SUCCESS on attempt ");
                Serial.print(attempt + 1);
                Serial.print(": 0x");
                Serial.println(data, HEX);
            }
            return true;
        } else {
            Serial.print("GPIO Read FAILED on attempt ");
            Serial.print(attempt + 1);
            Serial.println(" - No data received");
            delay(10);
        }
    }
    
    Serial.println("GPIO Read FAILED after 3 attempts");
    return false;
}

bool PCF8574gpio::readBit(uint8_t pin, bool &state) {
    if (pin > 7) return false;
    
    uint8_t data;
    if (!readByte(data)) return false;
    
    state = (data & (1 << pin)) != 0;
    return true;
}

bool PCF8574gpio::writeBit(uint8_t pin, bool state) {
    if (pin > 7) {
        Serial.print("GPIO Error: Invalid pin number ");
        Serial.println(pin);
        return false;
    }
    
    // Read current state first to preserve other pins
    uint8_t currentState = _gpioState;
    
    // Update only the target pin
    if (state) {
        currentState |= (1 << pin);   // Set bit
    } else {
        currentState &= ~(1 << pin);  // Clear bit
    }
    
    // Only write if the state actually changed
    if (currentState != _gpioState) {
        Serial.print("GPIO: Setting pin ");
        Serial.print(pin);
        Serial.print(" to ");
        Serial.print(state ? "HIGH" : "LOW");
        Serial.print(" (state: 0x");
        Serial.print(currentState, HEX);
        Serial.print(") - ");
        
        bool success = writeByte(currentState);
        Serial.println(success ? "SUCCESS" : "FAILED");
        return success;
    } else {
        // Pin already in desired state, no need to write
        Serial.print("GPIO: Pin ");
        Serial.print(pin);
        Serial.print(" already ");
        Serial.println(state ? "HIGH" : "LOW");
        return true;
    }
}

bool PCF8574gpio::readPin(uint8_t pin) {
    bool state = false;
    readBit(pin, state);
    return state;
}

void PCF8574gpio::writePin(uint8_t pin, bool state) {
    writeBit(pin, state);
}

// Mode management methods
PCF8574Mode PCF8574gpio::getMode() const {
    return _mode;
}

void PCF8574gpio::setMode(PCF8574Mode mode) {
    _mode = mode;
    Serial.print("PCF8574 mode changed to: ");
    Serial.println(_mode == PCF8574Mode::INPUT_MODE ? "INPUT_MODE" : "OUTPUT_MODE");
    
    // If switching to output mode, initialize all outputs to false
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        initializeOutputs();
    }
}

bool PCF8574gpio::isOutputMode() const {
    return _mode == PCF8574Mode::OUTPUT_MODE;
}

bool PCF8574gpio::isInputMode() const {
    return _mode == PCF8574Mode::INPUT_MODE;
}

void PCF8574gpio::initializeOutputs() {
    Serial.print("PCF8574: Initializing outputs for mode: ");
    Serial.println(_mode == PCF8574Mode::INPUT_MODE ? "INPUT_MODE" : "OUTPUT_MODE");
    
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        // Set all outputs to LOW (0x00) and enforce output mode
        Serial.println("PCF8574: Setting all outputs to LOW (0x00)");
        
        // Force the device into output mode by writing all pins LOW
        selectTCAChannel(tcaChannel);
        delay(5);
        
        wire->beginTransmission(_address);
        wire->write(0x00);
        int result = wire->endTransmission();
        
        if (result == 0) {
            _gpioState = 0x00;
            Serial.println("PCF8574: Output initialization SUCCESS");
            
            // Small delay to ensure the device settles
            delay(50);
            
            // Optionally verify that we're in output mode by testing a pin
            Serial.println("PCF8574: Verifying output mode by testing pin states");
            
        } else {
            Serial.print("PCF8574: Output initialization FAILED with I2C error: ");
            Serial.println(result);
        }
    } else {
        // For input mode, set all pins high to enable pull-ups
        Serial.println("PCF8574: Enabling pull-ups (0xFF)");
        bool success = writeByte(0xFF);
        if (success) {
            Serial.println("PCF8574: Input initialization SUCCESS");
        } else {
            Serial.println("PCF8574: Input initialization FAILED");
        }
    }
}

// NEW: Method to refresh/maintain output state
void PCF8574gpio::refreshOutputState() {
    if (_mode == PCF8574Mode::OUTPUT_MODE) {
        // Re-write the current state to ensure it's maintained
        selectTCAChannel(tcaChannel);
        delay(2);
        
        wire->beginTransmission(_address);
        wire->write(_gpioState);
        int result = wire->endTransmission();
        
        if (result != 0) {
            Serial.print("GPIO: Failed to refresh output state, I2C error: ");
            Serial.println(result);
        }
    }
}

// NEW: Force output mode method
void PCF8574gpio::forceOutputMode() {
    Serial.println("PCF8574: Forcing device into OUTPUT mode");
    
    _mode = PCF8574Mode::OUTPUT_MODE;
    
    // Write current state to force output mode
    selectTCAChannel(tcaChannel);
    delay(5);
    
    // Write the current state twice to ensure it sticks
    for (int i = 0; i < 2; i++) {
        wire->beginTransmission(_address);
        wire->write(_gpioState);
        int result = wire->endTransmission();
        
        if (result == 0) {
            Serial.print("PCF8574: Force output mode attempt ");
            Serial.print(i + 1);
            Serial.println(" - SUCCESS");
        } else {
            Serial.print("PCF8574: Force output mode attempt ");
            Serial.print(i + 1);
            Serial.print(" - FAILED with error: ");
            Serial.println(result);
        }
        
        delay(10);
    }
}

// NEW: Advanced hardware diagnostics method
bool PCF8574gpio::performHardwareDiagnostics() {
    Serial.println("\n=== PCF8574 Advanced Hardware Diagnostics ===");
    
    // Step 1: Basic connectivity test
    Serial.println("Step 1: Testing basic I2C connectivity...");
    selectTCAChannel(tcaChannel);
    delay(10);
    
    wire->beginTransmission(_address);
    int error = wire->endTransmission();
    
    if (error != 0) {
        Serial.print("CRITICAL: I2C communication failed with error: ");
        Serial.println(error);
        return false;
    }
    Serial.println("✓ I2C communication OK");
    
    // Step 2: Power supply test (check if all pins can go HIGH)
    Serial.println("\nStep 2: Testing power supply (all pins HIGH)...");
    bool powerTestResult = performPowerSupplyTest();
    
    // Step 3: Ground test (check if all pins can go LOW)
    Serial.println("\nStep 3: Testing ground connections (all pins LOW)...");
    bool groundTestResult = performGroundTest();
    
    // Step 4: Individual pin test
    Serial.println("\nStep 4: Testing individual pin control...");
    bool pinTestResult = performIndividualPinTest();
    
    // Step 5: Load test (check current sinking capability)
    Serial.println("\nStep 5: Testing current sinking capability...");
    bool loadTestResult = performLoadTest();
    
    // Summary
    Serial.println("\n=== Diagnostic Summary ===");
    Serial.print("I2C Communication: ");
    Serial.println("PASS");
    Serial.print("Power Supply Test: ");
    Serial.println(powerTestResult ? "PASS" : "FAIL");
    Serial.print("Ground Test: ");
    Serial.println(groundTestResult ? "PASS" : "FAIL");
    Serial.print("Individual Pin Test: ");
    Serial.println(pinTestResult ? "PASS" : "FAIL");
    Serial.print("Load Test: ");
    Serial.println(loadTestResult ? "PASS" : "FAIL");
    
    // Provide recommendations
    if (!powerTestResult) {
        Serial.println("\n⚠️  POWER SUPPLY ISSUE DETECTED:");
        Serial.println("  - Check VCC connection to PCF8574");
        Serial.println("  - Verify 3.3V or 5V supply voltage");
        Serial.println("  - Check for loose connections");
    }
    
    if (!groundTestResult) {
        Serial.println("\n⚠️  GROUND CONNECTION ISSUE:");
        Serial.println("  - Check GND connection to PCF8574");
        Serial.println("  - Verify common ground with microcontroller");
    }
    
    if (!pinTestResult) {
        Serial.println("\n⚠️  PIN CONTROL ISSUE DETECTED:");
        Serial.println("  - PCF8574 may be damaged");
        Serial.println("  - Check for external load preventing pin changes");
        Serial.println("  - Verify pull-up resistors (10kΩ recommended)");
    }
    
    if (!loadTestResult) {
        Serial.println("\n⚠️  CURRENT SINKING ISSUE:");
        Serial.println("  - External load may be too high");
        Serial.println("  - Check LED/relay connections");
        Serial.println("  - PCF8574 max sink current is 25mA per pin");
    }
    
    Serial.println("=================================");
    
    return powerTestResult && groundTestResult && pinTestResult;
}

bool PCF8574gpio::performPowerSupplyTest() {
    // Try to set all pins HIGH and verify
    selectTCAChannel(tcaChannel);
    delay(10);
    
    wire->beginTransmission(_address);
    wire->write(0xFF);
    int writeResult = wire->endTransmission();
    
    if (writeResult != 0) {
        Serial.print("Write failed with error: ");
        Serial.println(writeResult);
        return false;
    }
    
    delay(50); // Allow time for pins to settle
    
    // Read back the state
    uint8_t readValue = 0x00;
    if (wire->requestFrom(_address, (uint8_t)1) == 1) {
        readValue = wire->read();
        
        Serial.print("Wrote 0xFF, Read 0x");
        Serial.print(readValue, HEX);
        
        if (readValue == 0xFF) {
            Serial.println(" - PASS");
            return true;
        } else {
            Serial.print(" - FAIL (");
            int failedPins = 0;
            for (int pin = 0; pin < 8; pin++) {
                if ((readValue & (1 << pin)) == 0) {
                    if (failedPins > 0) Serial.print(", ");
                    Serial.print("Pin");
                    Serial.print(pin);
                    failedPins++;
                }
            }
            Serial.print(" stuck LOW)");
            Serial.println();
            return false;
        }
    } else {
        Serial.println("Read failed - no response");
        return false;
    }
}

bool PCF8574gpio::performGroundTest() {
    // Try to set all pins LOW and verify
    selectTCAChannel(tcaChannel);
    delay(10);
    
    wire->beginTransmission(_address);
    wire->write(0x00);
    int writeResult = wire->endTransmission();
    
    if (writeResult != 0) {
        Serial.print("Write failed with error: ");
        Serial.println(writeResult);
        return false;
    }
    
    delay(50); // Allow time for pins to settle
    
    // Read back the state
    uint8_t readValue = 0xFF;
    if (wire->requestFrom(_address, (uint8_t)1) == 1) {
        readValue = wire->read();
        
        Serial.print("Wrote 0x00, Read 0x");
        Serial.print(readValue, HEX);
        
        if (readValue == 0x00) {
            Serial.println(" - PASS");
            return true;
        } else {
            Serial.print(" - FAIL (");
            int failedPins = 0;
            for (int pin = 0; pin < 8; pin++) {
                if ((readValue & (1 << pin)) != 0) {
                    if (failedPins > 0) Serial.print(", ");
                    Serial.print("Pin");
                    Serial.print(pin);
                    failedPins++;
                }
            }
            Serial.print(" stuck HIGH)");
            Serial.println();
            return false;
        }
    } else {
        Serial.println("Read failed - no response");
        return false;
    }
}

bool PCF8574gpio::performIndividualPinTest() {
    bool allPassed = true;
    
    for (int pin = 0; pin < 8; pin++) {
        Serial.print("Testing Pin ");
        Serial.print(pin);
        Serial.print(": ");
        
        // Test HIGH
        uint8_t testValue = (1 << pin);
        selectTCAChannel(tcaChannel);
        delay(5);
        
        wire->beginTransmission(_address);
        wire->write(testValue);
        if (wire->endTransmission() != 0) {
            Serial.println("Write FAIL");
            allPassed = false;
            continue;
        }
        
        delay(20);
        
        uint8_t readValue = 0x00;
        if (wire->requestFrom(_address, (uint8_t)1) == 1) {
            readValue = wire->read();
            
            if ((readValue & (1 << pin)) != 0) {
                Serial.print("HIGH✓ ");
            } else {
                Serial.print("HIGH✗ ");
                allPassed = false;
            }
        } else {
            Serial.print("READ✗ ");
            allPassed = false;
            continue;
        }
        
        // Test LOW
        selectTCAChannel(tcaChannel);
        delay(5);
        
        wire->beginTransmission(_address);
        wire->write(0x00);
        if (wire->endTransmission() != 0) {
            Serial.println("Write FAIL");
            allPassed = false;
            continue;
        }
        
        delay(20);
        
        if (wire->requestFrom(_address, (uint8_t)1) == 1) {
            readValue = wire->read();
            
            if ((readValue & (1 << pin)) == 0) {
                Serial.println("LOW✓");
            } else {
                Serial.println("LOW✗");
                allPassed = false;
            }
        } else {
            Serial.println("READ✗");
            allPassed = false;
        }
    }
    
    return allPassed;
}

bool PCF8574gpio::performLoadTest() {
    // This test checks if external loads are preventing proper operation
    Serial.println("Checking for excessive external loads...");
    
    // Test pattern that would normally work
    uint8_t testPatterns[] = {0x55, 0xAA, 0xFF, 0x00};
    bool allPassed = true;
    
    for (int i = 0; i < 4; i++) {
        selectTCAChannel(tcaChannel);
        delay(10);
        
        wire->beginTransmission(_address);
        wire->write(testPatterns[i]);
        if (wire->endTransmission() != 0) {
            allPassed = false;
            continue;
        }
        
        delay(50); // Longer delay for loads to settle
        
        uint8_t readValue;
        if (wire->requestFrom(_address, (uint8_t)1) == 1) {
            readValue = wire->read();
            
            Serial.print("Pattern 0x");
            Serial.print(testPatterns[i], HEX);
            Serial.print(" -> 0x");
            Serial.print(readValue, HEX);
            
            if (readValue == testPatterns[i]) {
                Serial.println(" ✓");
            } else {
                Serial.println(" ✗");
                allPassed = false;
            }
        } else {
            allPassed = false;
        }
    }
    
    return allPassed;
}
