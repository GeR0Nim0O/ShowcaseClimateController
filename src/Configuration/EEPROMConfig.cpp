#include "EEPROMConfig.h"
#include <FS.h>
#include <LittleFS.h>

bool EEPROMConfig::initialize() {
    Serial.println("Initializing EEPROM for configuration storage...");
    EEPROM.begin(EEPROM_SIZE);
    return true;
}

String EEPROMConfig::readProjectConfigFile() {
    // Try to read from LittleFS first (if config was uploaded)
    if (LittleFS.begin()) {
        File configFile = LittleFS.open("/config.json", "r");
        if (configFile) {
            String content = configFile.readString();
            configFile.close();
            Serial.println("Read config from LittleFS");
            return content;
        }
    }
    
    // If LittleFS fails, we have no project config available
    Serial.println("No project config.json file available");
    return "";
}

bool EEPROMConfig::validateConfigJSON(const String& jsonString) {
    if (jsonString.length() == 0) {
        return false;
    }
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    if (error) {
        Serial.print("Config validation failed: ");
        Serial.println(error.c_str());
        return false;
    }
    
    // Check for required fields
    if (!doc["project"]["number"] || !doc["wifi"]["ssid"]) {
        Serial.println("Config validation failed: missing required fields");
        return false;
    }
    
    return true;
}

bool EEPROMConfig::writeProjectConfigToEEPROM() {
    Serial.println("Attempting to write project config to EEPROM...");
    
    String configContent = readProjectConfigFile();
    if (configContent.length() == 0) {
        Serial.println("No project config content to write to EEPROM");
        return false;
    }
    
    if (!validateConfigJSON(configContent)) {
        Serial.println("Project config validation failed");
        return false;
    }
    
    size_t configSize = configContent.length();
    if (configSize > (EEPROM_SIZE - CONFIG_DATA_ADDR - 4)) {  // -4 for magic number
        Serial.println("Config too large for EEPROM");
        return false;
    }
    
    // Write magic number
    EEPROM.put(CONFIG_START_ADDR, CONFIG_MAGIC);
    
    // Write config size
    EEPROM.put(CONFIG_SIZE_ADDR + 4, (uint32_t)configSize);
    
    // Write config data
    for (size_t i = 0; i < configSize; i++) {
        EEPROM.write(CONFIG_DATA_ADDR + i, configContent[i]);
    }
    
    EEPROM.commit();
    
    Serial.print("Successfully wrote ");
    Serial.print(configSize);
    Serial.println(" bytes of config to EEPROM");
    
    return true;
}

bool EEPROMConfig::readConfigFromEEPROM(JsonDocument& doc) {
    Serial.println("Reading configuration from EEPROM...");
    
    // Check magic number
    uint32_t magic;
    EEPROM.get(CONFIG_START_ADDR, magic);
    if (magic != CONFIG_MAGIC) {
        Serial.println("Invalid magic number in EEPROM - no valid config");
        return false;
    }
    
    // Read config size
    uint32_t configSize;
    EEPROM.get(CONFIG_SIZE_ADDR + 4, configSize);
    
    if (configSize == 0 || configSize > (EEPROM_SIZE - CONFIG_DATA_ADDR)) {
        Serial.println("Invalid config size in EEPROM");
        return false;
    }
    
    // Read config data
    String configContent;
    configContent.reserve(configSize);
    
    for (size_t i = 0; i < configSize; i++) {
        configContent += (char)EEPROM.read(CONFIG_DATA_ADDR + i);
    }
    
    // Parse JSON
    DeserializationError error = deserializeJson(doc, configContent);
    if (error) {
        Serial.print("Failed to parse EEPROM config: ");
        Serial.println(error.c_str());
        return false;
    }
    
    Serial.print("Successfully loaded ");
    Serial.print(configSize);
    Serial.println(" bytes of config from EEPROM");
    
    return true;
}

bool EEPROMConfig::hasValidConfig() {
    uint32_t magic;
    EEPROM.get(CONFIG_START_ADDR, magic);
    return (magic == CONFIG_MAGIC);
}

void EEPROMConfig::clearConfig() {
    Serial.println("Clearing EEPROM configuration...");
    
    // Clear magic number
    EEPROM.put(CONFIG_START_ADDR, 0x00000000);
    EEPROM.commit();
    
    Serial.println("EEPROM configuration cleared");
}

void EEPROMConfig::printEEPROMStatus() {
    Serial.println("\n=== EEPROM Configuration Status ===");
    
    uint32_t magic;
    EEPROM.get(CONFIG_START_ADDR, magic);
    
    Serial.print("Magic Number: 0x");
    Serial.println(magic, HEX);
    Serial.print("Valid Config: ");
    Serial.println(hasValidConfig() ? "Yes" : "No");
    
    if (hasValidConfig()) {
        uint32_t configSize;
        EEPROM.get(CONFIG_SIZE_ADDR + 4, configSize);
        Serial.print("Config Size: ");
        Serial.print(configSize);
        Serial.println(" bytes");
        
        float usage = (float)(configSize + 8) / EEPROM_SIZE * 100.0;
        Serial.print("EEPROM Usage: ");
        Serial.print(usage, 1);
        Serial.println("%");
    }
    
    Serial.println("==================================\n");
}
