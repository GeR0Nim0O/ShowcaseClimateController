#ifndef I2CHANDLER_H
#define I2CHANDLER_H

#include <Wire.h>
#include <Arduino.h>
#include <map>
#include <vector>

#define WIRE Wire

class I2CHandler {
public:
    static void initializeI2C();
    static void scanI2C();
    static void selectTCA(uint8_t i);
    static std::map<uint8_t, std::vector<uint8_t>> TCAScanner(); // Combined scan method
    static void printTCAScanResults(const std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults); // Print TCA scan results
};

void tcaSelect(uint8_t i); // Declare tcaSelect function

#endif // I2CHANDLER_H