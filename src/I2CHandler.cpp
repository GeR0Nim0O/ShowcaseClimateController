#include "I2CHandler.h"

#define TCAADDR 0x70

void I2CHandler::initializeI2C() {
    Serial.println("Initializing I2C bus...");
    
    // Initialize I2C with specific pins for ESP32-S3
    WIRE.begin(17, 16); // SDA=GPIO17, SCL=GPIO16
    WIRE.setClock(100000); // 100kHz for better reliability
    delay(100);
    
    // Test TCA multiplexer connection
    Serial.println("Testing TCA multiplexer connection...");
    WIRE.beginTransmission(TCAADDR);
    uint8_t tcaError = WIRE.endTransmission();
    
    if (tcaError == 0) {
        Serial.println("TCA multiplexer found and responding");
        
        // Reset TCA to known state (disable all channels)
        WIRE.beginTransmission(TCAADDR);
        WIRE.write(0x00);
        WIRE.endTransmission();
        Serial.println("TCA reset to default state");
    } else {
        Serial.print("TCA multiplexer NOT found! Error: ");
        Serial.println(tcaError);
        Serial.println("This will cause all device initializations to fail!");
    }
    
    Serial.println("I2C bus initialized.");
}

void I2CHandler::scanI2C() {
    Serial.println("\nI2C Scanner");
    for (uint8_t addr = 1; addr < 127; addr++) {
        WIRE.beginTransmission(addr);
        uint8_t error = WIRE.endTransmission();
        if (error == 0) {
            Serial.print("Found I2C device at address 0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    Serial.println("I2C scan done");
}

void I2CHandler::selectTCA(uint8_t i) {
    if (i > 7) {
        Serial.print("Invalid TCA port: ");
        Serial.println(i);
        return;
    }

    // Add mutex/locking mechanism for TCA selection to prevent conflicts
    static uint8_t lastSelectedPort = 255; // Invalid port number to force first selection
    static unsigned long lastSelectionTime = 0;
    unsigned long currentTime = millis();
    
    // If we're selecting the same port and it was recent, skip reselection
    if (lastSelectedPort == i && (currentTime - lastSelectionTime < 50)) {
        return; // Port already selected recently
    }
    
    // Retry TCA selection up to 3 times
    int retries = 3;
    bool success = false;
    
    for (int attempt = 0; attempt < retries && !success; attempt++) {
        WIRE.beginTransmission(TCAADDR);
        WIRE.write(1 << i);
        uint8_t error = WIRE.endTransmission();
        
        if (error == 0) {
            success = true;
            lastSelectedPort = i;
            lastSelectionTime = currentTime;
            
            // Remove readback verification as it interferes with device communication
            // Small delay to ensure TCA stabilizes
            delayMicroseconds(100);
        } else {
            Serial.print("Failed to select TCA Port ");
            Serial.print(i);
            Serial.print(" (attempt ");
            Serial.print(attempt + 1);
            Serial.print("): Error ");
            Serial.println(error);
            if (attempt < retries - 1) {
                delay(2); // Shorter delay before retry
            }
        }
    }
    
    if (!success) {
        Serial.print("TCA Port ");
        Serial.print(i);
        Serial.println(" selection failed after all retries");
        lastSelectedPort = 255; // Reset on failure
    }
}

void tcaSelect(uint8_t i) {
    I2CHandler::selectTCA(i);
}

std::map<uint8_t, std::vector<uint8_t>> I2CHandler::TCAScanner() {
    std::map<uint8_t, std::vector<uint8_t>> tcaScanResults;
    Serial.println("\nTCAScanner ready!");
    for (uint8_t t = 0; t < 8; t++) {
        I2CHandler::selectTCA(t);
        Serial.print("TCA Port #");
        Serial.println(t);

        for (uint8_t addr = 0; addr <= 127; addr++) {
            if (addr == TCAADDR)
                continue;

            WIRE.beginTransmission(addr);
            uint8_t error = WIRE.endTransmission();
            if (error == 0) {
                tcaScanResults[t].push_back(addr);
            }
        }
    }
    Serial.println("\nDone scanning TCA ports.");
    return tcaScanResults;
}

void I2CHandler::printTCAScanResults(const std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults) {
    Serial.println("TCA Scan Results:");
    for (const auto& portDevices : tcaScanResults) {
        uint8_t port = portDevices.first;
        const std::vector<uint8_t>& devices = portDevices.second;
        Serial.print("Port ");
        Serial.print(port);
        Serial.println(":");
        for (uint8_t address : devices) {
            Serial.print("  Address: 0x");
            Serial.println(address, HEX);
        }
    }
}