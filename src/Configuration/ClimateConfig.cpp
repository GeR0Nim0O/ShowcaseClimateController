#include "../../../lib/Config/ClimateConfig/ClimateConfig.h"  // Use relative path to find the header in lib directory
#include <Arduino.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

// Singleton instance
ClimateConfig& ClimateConfig::getInstance() {
    static ClimateConfig instance;
    return instance;
}

bool ClimateConfig::begin() {
    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to initialize SPIFFS");
        return false;
    }
    
    // Initialize EEPROM if needed
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed to initialize EEPROM");
        return false;
    }
    
    // Try to load settings from JSON file first, then from EEPROM
    if (!loadFromJsonFile()) {
        Serial.println("No valid JSON file found, trying EEPROM");
        if (!loadSettings()) {
            Serial.println("No valid settings found in EEPROM, loading defaults");
            loadDefaults();
            saveSettings(); // Save the defaults to EEPROM
            createDefaultJsonFile(); // Create default JSON file
        }
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
    // New settings
    settings.climateMode = "AUTO";
    settings.humidityMode = "AUTO";
    settings.autoFanControl = true;
    settings.temperatureHysteresis = 0.5;
    settings.humidityHysteresis = 2.0;
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
    
    // Check hysteresis values are reasonable
    if (settings.temperatureHysteresis < 0.1 || settings.temperatureHysteresis > 5.0) {
        return false;
    }
    
    if (settings.humidityHysteresis < 0.5 || settings.humidityHysteresis > 10.0) {
        return false;
    }
    
    // Check that climate and humidity modes are valid
    if (settings.climateMode != "AUTO" && settings.climateMode != "MANUAL" && 
        settings.climateMode != "HEAT" && settings.climateMode != "COOL") {
        return false;
    }
    
    if (settings.humidityMode != "AUTO" && settings.humidityMode != "MANUAL" && 
        settings.humidityMode != "HUMIDIFY" && settings.humidityMode != "DEHUMIDIFY") {
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
    
    Serial.print("Climate Mode: ");
    Serial.println(settings.climateMode);
    
    Serial.print("Humidity Mode: ");
    Serial.println(settings.humidityMode);
    
    Serial.print("Auto Fan Control: ");
    Serial.println(settings.autoFanControl ? "ON" : "OFF");
    
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
    
    Serial.print("Temperature Hysteresis: ");
    Serial.print(settings.temperatureHysteresis);
    Serial.println("°C");
    
    Serial.print("Humidity Hysteresis: ");
    Serial.print(settings.humidityHysteresis);
    Serial.println("%");
    
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

bool ClimateConfig::loadFromJsonFile(const String& filePath) {
    if (!SPIFFS.exists(filePath)) {
        Serial.println("Climate config JSON file does not exist: " + filePath);
        return false;
    }
    
    File file = SPIFFS.open(filePath, "r");
    if (!file) {
        Serial.println("Failed to open climate config file for reading");
        return false;
    }
    
    size_t size = file.size();
    if (size > 2048) {
        Serial.println("Climate config file too large");
        file.close();
        return false;
    }
    
    std::unique_ptr<char[]> buf(new char[size]);
    file.readBytes(buf.get(), size);
    file.close();
    
    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, buf.get());
    
    if (error) {
        Serial.print("Failed to parse climate config JSON: ");
        Serial.println(error.c_str());
        return false;
    }
    
    // Parse climate controller section
    JsonObject climate = doc["climate_controller"];
    if (!climate) {
        Serial.println("No climate_controller section found in JSON");
        return false;
    }
    
    // Load basic settings
    settings.temperatureSetpoint = climate["temperature_setpoint"] | 22.0;
    settings.humiditySetpoint = climate["humidity_setpoint"] | 50.0;
    settings.climateMode = climate["climate_mode"] | "AUTO";
    settings.humidityMode = climate["humidity_mode"] | "AUTO";
    settings.autoFanControl = climate["auto_fan_control"] | true;
    settings.updateInterval = climate["update_interval_ms"] | 1000;
    
    // Load safety limits
    JsonObject safety = climate["safety_limits"];
    if (safety) {
        settings.maxTemperature = safety["max_temperature"] | 35.0;
        settings.minTemperature = safety["min_temperature"] | 10.0;
        settings.maxHumidity = safety["max_humidity"] | 80.0;
        settings.minHumidity = safety["min_humidity"] | 20.0;
    }
    
    // Load PID parameters
    JsonObject tempPid = climate["pid_parameters"]["temperature"];
    if (tempPid) {
        settings.temperatureKp = tempPid["kp"] | 2.0;
        settings.temperatureKi = tempPid["ki"] | 0.5;
        settings.temperatureKd = tempPid["kd"] | 0.1;
    }
    
    JsonObject humPid = climate["pid_parameters"]["humidity"];
    if (humPid) {
        settings.humidityKp = humPid["kp"] | 1.0;
        settings.humidityKi = humPid["ki"] | 0.2;
        settings.humidityKd = humPid["kd"] | 0.05;
    }
    
    // Load control parameters
    JsonObject control = climate["control_parameters"];
    if (control) {
        settings.temperatureHysteresis = control["temperature_hysteresis"] | 0.5;
        settings.humidityHysteresis = control["humidity_hysteresis"] | 2.0;
    }
    
    // Load fan settings
    JsonObject fans = climate["fan_settings"];
    if (fans) {
        settings.fanInteriorEnabled = fans["interior_fan_enabled"] | true;
        settings.fanExteriorEnabled = fans["exterior_fan_enabled"] | false;
    }
    
    // Validate loaded settings
    if (!validateSettings()) {
        Serial.println("Loaded climate settings failed validation");
        return false;
    }
    
    Serial.println("Climate settings loaded from JSON file successfully");
    return true;
}

bool ClimateConfig::saveToJsonFile(const String& filePath) {
    DynamicJsonDocument doc(2048);
    
    // Create climate controller section
    JsonObject climate = doc.createNestedObject("climate_controller");
    climate["enabled"] = true;
    climate["temperature_setpoint"] = settings.temperatureSetpoint;
    climate["humidity_setpoint"] = settings.humiditySetpoint;
    climate["climate_mode"] = settings.climateMode;
    climate["humidity_mode"] = settings.humidityMode;
    climate["auto_fan_control"] = settings.autoFanControl;
    climate["update_interval_ms"] = settings.updateInterval;
    
    // Safety limits
    JsonObject safety = climate.createNestedObject("safety_limits");
    safety["max_temperature"] = settings.maxTemperature;
    safety["min_temperature"] = settings.minTemperature;
    safety["max_humidity"] = settings.maxHumidity;
    safety["min_humidity"] = settings.minHumidity;
    
    // PID parameters
    JsonObject pid = climate.createNestedObject("pid_parameters");
    JsonObject tempPid = pid.createNestedObject("temperature");
    tempPid["kp"] = settings.temperatureKp;
    tempPid["ki"] = settings.temperatureKi;
    tempPid["kd"] = settings.temperatureKd;
    
    JsonObject humPid = pid.createNestedObject("humidity");
    humPid["kp"] = settings.humidityKp;
    humPid["ki"] = settings.humidityKi;
    humPid["kd"] = settings.humidityKd;
    
    // Control parameters
    JsonObject control = climate.createNestedObject("control_parameters");
    control["temperature_hysteresis"] = settings.temperatureHysteresis;
    control["humidity_hysteresis"] = settings.humidityHysteresis;
    
    // Fan settings
    JsonObject fans = climate.createNestedObject("fan_settings");
    fans["interior_fan_enabled"] = settings.fanInteriorEnabled;
    fans["exterior_fan_enabled"] = settings.fanExteriorEnabled;
    
    // Metadata
    JsonObject metadata = doc.createNestedObject("metadata");
    metadata["version"] = "1.0";
    metadata["last_updated"] = "2025-05-31T00:00:00Z";
    metadata["created_by"] = "ClimateConfig";
    metadata["description"] = "Dynamic climate controller configuration";
    
    // Write to file
    File file = SPIFFS.open(filePath, "w");
    if (!file) {
        Serial.println("Failed to open climate config file for writing");
        return false;
    }
    
    if (serializeJsonPretty(doc, file) == 0) {
        Serial.println("Failed to write climate config to file");
        file.close();
        return false;
    }
    
    file.close();
    Serial.println("Climate settings saved to JSON file successfully");
    return true;
}

bool ClimateConfig::updateJsonFile(const String& filePath) {
    // Save current settings and also update EEPROM
    bool jsonSuccess = saveToJsonFile(filePath);
    bool eepromSuccess = saveSettings();
    
    return jsonSuccess && eepromSuccess;
}

bool ClimateConfig::createDefaultJsonFile(const String& filePath) {
    // Load defaults first
    loadDefaults();
    
    // Save defaults to JSON file
    return saveToJsonFile(filePath);
}
