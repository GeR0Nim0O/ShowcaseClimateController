#include "ClimateConfig.h"
#include <Arduino.h>
#include <EEPROM.h>

// Singleton instance
ClimateConfig& ClimateConfig::getInstance() {
    static ClimateConfig instance;
    return instance;
}

bool ClimateConfig::begin() {
    // Initialize EEPROM if needed
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed to initialize EEPROM");
        return false;
    }
    
    // Try to load settings, if fails, load defaults
    if (!loadSettings()) {
        Serial.println("No valid settings found in EEPROM, loading defaults");
        loadDefaults();
        saveSettings(); // Save the defaults to EEPROM
    }
    
    return true;
}

void ClimateConfig::loadDefaults() {
    settings.temperatureSetpoint = 22.0;
    settings.humiditySetpoint = 50.0;
    settings.temperatureKp = 2.0;
    settings.temperatureKi = 0.5;
    settings.temperatureKd = 0.1;
    settings.humidityKp = 1.0;
    settings.humidityKi = 0.2;
    settings.humidityKd = 0.05;
    settings.fanInteriorEnabled = true;
    settings.fanExteriorEnabled = false;
    settings.updateInterval = 1000; // 1 second
    settings.maxTemperature = 35.0;
    settings.minTemperature = 10.0;
    settings.maxHumidity = 80.0;
    settings.minHumidity = 20.0;
}

bool ClimateConfig::saveSettings() {
    // Calculate checksum before saving
    settings.checksum = calculateChecksum();
    
    // Write settings to EEPROM
    EEPROM.put(SETTINGS_ADDRESS, settings);
    if (!EEPROM.commit()) {
        Serial.println("Failed to commit settings to EEPROM");
        return false;
    }
    
    Serial.println("Settings saved to EEPROM");
    return true;
}

bool ClimateConfig::loadSettings() {
    // Read settings from EEPROM
    EEPROM.get(SETTINGS_ADDRESS, settings);
    
    // Validate settings with checksum
    if (!isValidChecksum()) {
        Serial.println("Checksum validation failed");
        return false;
    }
    
    // Validate that values are within reasonable ranges
    if (!validateSettings()) {
        Serial.println("Settings validation failed");
        return false;
    }
    
    Serial.println("Settings loaded from EEPROM");
    return true;
}

uint32_t ClimateConfig::calculateChecksum() {
    // Simple checksum calculation
    uint32_t checksum = 0;
    uint8_t* ptr = (uint8_t*)&settings;
    
    // Skip the checksum field itself
    for (size_t i = 0; i < sizeof(ClimateSettings) - sizeof(uint32_t); i++) {
        checksum += ptr[i];
    }
    
    return checksum;
}

bool ClimateConfig::isValidChecksum() {
    uint32_t storedChecksum = settings.checksum;
    settings.checksum = 0;
    uint32_t calculatedChecksum = calculateChecksum();
    settings.checksum = storedChecksum;
    
    return storedChecksum == calculatedChecksum;
}

bool ClimateConfig::validateSettings() {
    // Check temperature setpoint range
    if (settings.temperatureSetpoint < 10.0 || settings.temperatureSetpoint > 35.0) {
        return false;
    }
    
    // Check humidity setpoint range
    if (settings.humiditySetpoint < 20.0 || settings.humiditySetpoint > 80.0) {
        return false;
    }
    
    // Check PID parameters are positive
    if (settings.temperatureKp <= 0 || settings.temperatureKi < 0 || settings.temperatureKd < 0) {
        return false;
    }
    
    if (settings.humidityKp <= 0 || settings.humidityKi < 0 || settings.humidityKd < 0) {
        return false;
    }
    
    // Check update interval is reasonable
    if (settings.updateInterval < 100 || settings.updateInterval > 10000) {
        return false;
    }
    
    return true;
}

void ClimateConfig::printSettings() {
    Serial.println("\n=== Climate Controller Settings ===");
    Serial.print("Temperature Setpoint: ");
    Serial.print(settings.temperatureSetpoint);
    Serial.println("°C");
    
    Serial.print("Humidity Setpoint: ");
    Serial.print(settings.humiditySetpoint);
    Serial.println("%");
    
    Serial.println("Temperature PID:");
    Serial.print("  Kp: ");
    Serial.println(settings.temperatureKp);
    Serial.print("  Ki: ");
    Serial.println(settings.temperatureKi);
    Serial.print("  Kd: ");
    Serial.println(settings.temperatureKd);
    
    Serial.println("Humidity PID:");
    Serial.print("  Kp: ");
    Serial.println(settings.humidityKp);
    Serial.print("  Ki: ");
    Serial.println(settings.humidityKi);
    Serial.print("  Kd: ");
    Serial.println(settings.humidityKd);
    
    Serial.print("Interior Fan: ");
    Serial.println(settings.fanInteriorEnabled ? "ON" : "OFF");
    Serial.print("Exterior Fan: ");
    Serial.println(settings.fanExteriorEnabled ? "ON" : "OFF");
    
    Serial.print("Update Interval: ");
    Serial.print(settings.updateInterval);
    Serial.println("ms");
    
    Serial.print("Temperature Range: ");
    Serial.print(settings.minTemperature);
    Serial.print("°C - ");
    Serial.print(settings.maxTemperature);
    Serial.println("°C");
    
    Serial.print("Humidity Range: ");
    Serial.print(settings.minHumidity);
    Serial.print("% - ");
    Serial.print(settings.maxHumidity);
    Serial.println("%");
    
    Serial.println("===============================");
}
