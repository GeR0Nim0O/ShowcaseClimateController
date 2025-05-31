#ifndef EEPROM_CONFIG_H
#define EEPROM_CONFIG_H

#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

class EEPROMConfig {
public:
    // EEPROM configuration constants
    static const size_t EEPROM_SIZE = 4096;  // 4KB for configuration
    static const size_t CONFIG_START_ADDR = 0;
    static const size_t CONFIG_SIZE_ADDR = 0;  // First 4 bytes store config size
    static const size_t CONFIG_DATA_ADDR = 4;  // Config data starts at byte 4
    static const uint32_t CONFIG_MAGIC = 0x43464721;  // Magic number "CFG!"
    
    // Initialize EEPROM
    static bool initialize();
    
    // Write project config.json to EEPROM
    static bool writeProjectConfigToEEPROM();
    
    // Read configuration from EEPROM
    static bool readConfigFromEEPROM(JsonDocument& doc);
    
    // Check if valid configuration exists in EEPROM
    static bool hasValidConfig();
    
    // Clear EEPROM configuration
    static void clearConfig();
    
    // Get EEPROM usage statistics
    static void printEEPROMStatus();

private:
    // Read project config.json file and return as string
    static String readProjectConfigFile();
    
    // Validate JSON configuration
    static bool validateConfigJSON(const String& jsonString);
};

#endif // EEPROM_CONFIG_H
