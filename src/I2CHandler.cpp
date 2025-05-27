#include "I2CHandler.h"

#define TCAADDR 0x70

void I2CHandler::initializeI2C() {
    Serial.println("Initializing I2C bus...");
    WIRE.begin(33, 13); // SDA, SCL
    WIRE.setClock(100000);
    delay(100);
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
    if (i > 7) return;

    WIRE.beginTransmission(TCAADDR);
    WIRE.write(1 << i);
    uint8_t error = WIRE.endTransmission();
    if (error != 0) {
        Serial.print("Failed to select TCA Port: ");
        Serial.println(i);
        Serial.print("Error: ");
        Serial.println(error);
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