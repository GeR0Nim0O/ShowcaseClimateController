#include "ClimateConfig.h"

ClimateConfig& ClimateConfig::getInstance() {
    static ClimateConfig instance;
    return instance;
}

bool ClimateConfig::begin() {
    EEPROM.begin(EEPROM_SIZE);
    
    if (loadSettings()) {
        Serial.println("Climate configuration loaded from EEPROM");
        printSettings();
        return true;
    } else {
        Serial.println("Loading default climate configuration");
        loadDefaults();
        saveSettings();
        return true;
    }
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
    settings.fanExteriorEnabled = true;
    settings.updateInterval = 1000;
    settings.maxTemperature = 35.0;
    settings.minTemperature = 10.0;
    settings.maxHumidity = 80.0;
    settings.minHumidity = 20.0;
    settings.checksum = calculateChecksum();
}

bool ClimateConfig::saveSettings() {
    settings.checksum = calculateChecksum();
    
    EEPROM.put(SETTINGS_ADDRESS, settings);
    EEPROM.commit();
    
    Serial.println("Climate settings saved to EEPROM");
    return true;
}

bool ClimateConfig::loadSettings() {
    ClimateSettings tempSettings;
    EEPROM.get(SETTINGS_ADDRESS, tempSettings);
    
    // Temporarily store current settings
    ClimateSettings originalSettings = settings;
    settings = tempSettings;
    
    if (isValidChecksum() && validateSettings()) {
        Serial.println("Valid settings loaded from EEPROM");
        return true;
    } else {
        // Restore original settings if loaded settings are invalid
        settings = originalSettings;
        Serial.println("Invalid settings in EEPROM, using defaults");
        return false;
    }
}

bool ClimateConfig::validateSettings() {
    return (settings.temperatureSetpoint >= 0.0 && settings.temperatureSetpoint <= 50.0 &&
            settings.humiditySetpoint >= 0.0 && settings.humiditySetpoint <= 100.0 &&
            settings.temperatureKp >= 0.0 && settings.temperatureKp <= 100.0 &&
            settings.temperatureKi >= 0.0 && settings.temperatureKi <= 100.0 &&
            settings.temperatureKd >= 0.0 && settings.temperatureKd <= 100.0 &&
            settings.humidityKp >= 0.0 && settings.humidityKp <= 100.0 &&
            settings.humidityKi >= 0.0 && settings.humidityKi <= 100.0 &&
            settings.humidityKd >= 0.0 && settings.humidityKd <= 100.0 &&
            settings.updateInterval >= 100 && settings.updateInterval <= 10000 &&
            settings.maxTemperature > settings.minTemperature &&
            settings.maxHumidity > settings.minHumidity);
}

uint32_t ClimateConfig::calculateChecksum() {
    uint32_t checksum = 0;
    uint8_t* data = (uint8_t*)&settings;
    
    // Calculate checksum for all data except the checksum field itself
    for (int i = 0; i < sizeof(ClimateSettings) - sizeof(uint32_t); i++) {
        checksum += data[i];
    }
    
    return checksum;
}

bool ClimateConfig::isValidChecksum() {
    uint32_t calculatedChecksum = calculateChecksum();
    return (calculatedChecksum == settings.checksum);
}

void ClimateConfig::printSettings() {
    Serial.println("\n=== Climate Configuration ===");
    Serial.print("Temperature Setpoint: "); Serial.println(settings.temperatureSetpoint);
    Serial.print("Humidity Setpoint: "); Serial.println(settings.humiditySetpoint);
    Serial.print("Temperature PID (Kp, Ki, Kd): "); 
    Serial.print(settings.temperatureKp); Serial.print(", ");
    Serial.print(settings.temperatureKi); Serial.print(", ");
    Serial.println(settings.temperatureKd);
    Serial.print("Humidity PID (Kp, Ki, Kd): "); 
    Serial.print(settings.humidityKp); Serial.print(", ");
    Serial.print(settings.humidityKi); Serial.print(", ");
    Serial.println(settings.humidityKd);
    Serial.print("Fan Interior: "); Serial.println(settings.fanInteriorEnabled ? "Enabled" : "Disabled");
    Serial.print("Fan Exterior: "); Serial.println(settings.fanExteriorEnabled ? "Enabled" : "Disabled");
    Serial.print("Update Interval: "); Serial.print(settings.updateInterval); Serial.println(" ms");
    Serial.print("Temperature Range: "); Serial.print(settings.minTemperature); 
    Serial.print(" - "); Serial.println(settings.maxTemperature);
    Serial.print("Humidity Range: "); Serial.print(settings.minHumidity); 
    Serial.print(" - "); Serial.println(settings.maxHumidity);
    Serial.println("=============================\n");
}
