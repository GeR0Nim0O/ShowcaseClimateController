#include "../../../lib/Config/ClimateConfig/ClimateConfig.h"  // Use relative path to find the header in lib directory
#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

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
    
    // Try to load settings from main config.json first, then dedicated ClimateConfig.json, then EEPROM
    Serial.println("DEBUG: ClimateConfig::begin() - Attempting to load from main config.json");
    if (loadFromJsonFile("/data/config.json")) {
        Serial.println("DEBUG: ClimateConfig - Successfully loaded from main config.json");
        saveSettings(); // Save to EEPROM as backup
        return true;
    }
    
    Serial.println("DEBUG: ClimateConfig::begin() - Main config.json failed, trying dedicated ClimateConfig.json");
    if (loadFromJsonFile()) {
        Serial.println("DEBUG: ClimateConfig - Successfully loaded from ClimateConfig.json");
        saveSettings(); // Save to EEPROM as backup
        return true;
    }
      Serial.println("No valid JSON file found, trying EEPROM");
    if (!loadSettings()) {
        Serial.println("No valid settings found in EEPROM, loading defaults");
        loadDefaults();
        Serial.print("DEBUG: ClimateConfig - Default updateInterval set to: ");
        Serial.println(settings.updateInterval);
        saveSettings(); // Save the defaults to EEPROM
        createDefaultJsonFile(); // Create default JSON file
    } else {
        Serial.print("DEBUG: ClimateConfig - EEPROM updateInterval loaded as: ");
        Serial.println(settings.updateInterval);
        
        // Check if the loaded value looks corrupted (outside reasonable range)
        if (settings.updateInterval < 100 || settings.updateInterval > 10000) {
            Serial.println("DEBUG: ClimateConfig - EEPROM data appears corrupted, clearing and reloading");
            clearEEPROM();
        }
    }
    
    return true;
}

void ClimateConfig::loadDefaults() {
    Serial.println("DEBUG: ClimateConfig::loadDefaults() called");
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
    settings.updateInterval = 500; // Fix: Use 500ms like config.json
    settings.maxTemperature = 35.0;
    settings.minTemperature = 10.0;
    settings.maxHumidity = 80.0;
    settings.minHumidity = 20.0;    // New settings
    strncpy(settings.climateMode, "AUTO", 15);
    settings.climateMode[15] = '\0';
    strncpy(settings.humidityMode, "AUTO", 15);
    settings.humidityMode[15] = '\0';    settings.autoFanControl = true;
    settings.temperatureHysteresis = 0.5;
    settings.humidityHysteresis = 2.0;
    
    // Initialize AutoTune results as empty
    settings.hasAutoTuneResults = false;
    settings.autoTuneKp = 0.0;
    settings.autoTuneKi = 0.0;
    settings.autoTuneKd = 0.0;
    
    // Initialize AutoTune configuration
    settings.autoTuneOutputStep = 50.0;  // Default to 50% output step for moderate power
    
    Serial.print("DEBUG: ClimateConfig::loadDefaults() - updateInterval set to: ");
    Serial.println(settings.updateInterval);
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
        Serial.print("DEBUG: ClimateConfig - updateInterval value during validation failure: ");
        Serial.println(settings.updateInterval);
        return false;
    }
    
    Serial.println("Settings loaded from EEPROM");
    Serial.print("DEBUG: ClimateConfig - Final updateInterval after EEPROM load: ");
    Serial.println(settings.updateInterval);
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
    Serial.print("DEBUG: ClimateConfig::validateSettings() - updateInterval value: ");
    Serial.println(settings.updateInterval);
    if (settings.updateInterval < 100 || settings.updateInterval > 10000) {
        Serial.println("DEBUG: ClimateConfig::validateSettings() - updateInterval validation failed!");
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
    String climateMode = String(settings.climateMode);
    String humidityMode = String(settings.humidityMode);
    
    if (climateMode != "AUTO" && climateMode != "MANUAL" && 
        climateMode != "HEAT" && climateMode != "COOL") {
        return false;
    }
    
    if (humidityMode != "AUTO" && humidityMode != "MANUAL" && 
        humidityMode != "HUMIDIFY" && humidityMode != "DEHUMIDIFY") {
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
    }    std::unique_ptr<char[]> buf(new char[size]);
    file.readBytes(buf.get(), size);
    file.close();
    
    JsonDocument doc;
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
    }    // Load basic settings
    settings.temperatureSetpoint = climate["temperature_setpoint"].as<double>();
    if (!climate["temperature_setpoint"]) settings.temperatureSetpoint = 22.0;
    
    settings.humiditySetpoint = climate["humidity_setpoint"].as<double>();
    if (!climate["humidity_setpoint"]) settings.humiditySetpoint = 50.0;
    
    String tempClimateMode = climate["climate_mode"].as<String>();
    if (!climate["climate_mode"]) tempClimateMode = "AUTO";
    String tempHumidityMode = climate["humidity_mode"].as<String>();
    if (!climate["humidity_mode"]) tempHumidityMode = "AUTO";
    strncpy(settings.climateMode, tempClimateMode.c_str(), 15);
    settings.climateMode[15] = '\0';
    strncpy(settings.humidityMode, tempHumidityMode.c_str(), 15);
    settings.humidityMode[15] = '\0';
    
    settings.autoFanControl = climate["auto_fan_control"].as<bool>();
    if (!climate["auto_fan_control"]) settings.autoFanControl = true;
      settings.updateInterval = climate["update_interval_ms"].as<int>();
    if (!climate["update_interval_ms"]) settings.updateInterval = 1000;
    
    Serial.print("DEBUG: ClimateConfig - JSON updateInterval parsed as: ");
    Serial.println(settings.updateInterval);
      // Load safety limits
    JsonObject safety = climate["safety_limits"];
    if (safety) {
        settings.maxTemperature = safety["max_temperature"].as<double>();
        if (!safety["max_temperature"]) settings.maxTemperature = 35.0;
        
        settings.minTemperature = safety["min_temperature"].as<double>();
        if (!safety["min_temperature"]) settings.minTemperature = 10.0;
        
        settings.maxHumidity = safety["max_humidity"].as<double>();
        if (!safety["max_humidity"]) settings.maxHumidity = 80.0;
        
        settings.minHumidity = safety["min_humidity"].as<double>();
        if (!safety["min_humidity"]) settings.minHumidity = 20.0;
    }
      // Load PID parameters
    JsonObject tempPid = climate["pid_parameters"]["temperature"];
    if (tempPid) {
        settings.temperatureKp = tempPid["kp"].as<double>();
        if (!tempPid["kp"]) settings.temperatureKp = 2.0;
        
        settings.temperatureKi = tempPid["ki"].as<double>();
        if (!tempPid["ki"]) settings.temperatureKi = 0.5;
        
        settings.temperatureKd = tempPid["kd"].as<double>();
        if (!tempPid["kd"]) settings.temperatureKd = 0.1;
    }
    
    JsonObject humPid = climate["pid_parameters"]["humidity"];
    if (humPid) {
        settings.humidityKp = humPid["kp"].as<double>();
        if (!humPid["kp"]) settings.humidityKp = 1.0;
        
        settings.humidityKi = humPid["ki"].as<double>();
        if (!humPid["ki"]) settings.humidityKi = 0.2;
        
        settings.humidityKd = humPid["kd"].as<double>();
        if (!humPid["kd"]) settings.humidityKd = 0.05;
    }
    
    // Load control parameters
    JsonObject control = climate["control_parameters"];
    if (control) {
        settings.temperatureHysteresis = control["temperature_hysteresis"].as<double>();
        if (!control["temperature_hysteresis"]) settings.temperatureHysteresis = 0.5;
        
        settings.humidityHysteresis = control["humidity_hysteresis"].as<double>();
        if (!control["humidity_hysteresis"]) settings.humidityHysteresis = 2.0;
    }
      // Load fan settings
    JsonObject fans = climate["fan_settings"];
    if (fans) {
        settings.fanInteriorEnabled = fans["interior_fan_enabled"].as<bool>();
        if (!fans["interior_fan_enabled"]) settings.fanInteriorEnabled = true;
        
        settings.fanExteriorEnabled = fans["exterior_fan_enabled"].as<bool>();
        if (!fans["exterior_fan_enabled"]) settings.fanExteriorEnabled = false;
    }
    
    // Load AutoTune results
    JsonObject autoTune = climate["autotune_results"];
    if (autoTune) {
        settings.hasAutoTuneResults = autoTune["has_results"].as<bool>();
        if (settings.hasAutoTuneResults) {
            settings.autoTuneKp = autoTune["kp"].as<double>();
            settings.autoTuneKi = autoTune["ki"].as<double>();
            settings.autoTuneKd = autoTune["kd"].as<double>();
            
            Serial.println("AutoTune results loaded from JSON:");
            Serial.print("  Kp: "); Serial.println(settings.autoTuneKp, 4);
            Serial.print("  Ki: "); Serial.println(settings.autoTuneKi, 4);
            Serial.print("  Kd: "); Serial.println(settings.autoTuneKd, 4);
        }
    } else {
        // No AutoTune results in JSON
        settings.hasAutoTuneResults = false;
        settings.autoTuneKp = 0.0;
        settings.autoTuneKi = 0.0;
        settings.autoTuneKd = 0.0;
    }
    
    // Load AutoTune configuration
    JsonObject autoTuneConfig = climate["autotune_config"];
    if (autoTuneConfig) {
        settings.autoTuneOutputStep = autoTuneConfig["output_step"].as<double>();
        if (!autoTuneConfig["output_step"]) settings.autoTuneOutputStep = 50.0;
        
        Serial.print("AutoTune output step loaded from JSON: ");
        Serial.print(settings.autoTuneOutputStep);
        Serial.println("%");
    } else {
        // Default AutoTune configuration
        settings.autoTuneOutputStep = 50.0;
        Serial.println("Using default AutoTune output step: 50%");
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
    JsonDocument doc;
    
    // Create climate controller section
    JsonObject climate = doc["climate_controller"].to<JsonObject>();
    climate["enabled"] = true;climate["temperature_setpoint"] = settings.temperatureSetpoint;
    climate["humidity_setpoint"] = settings.humiditySetpoint;
    climate["climate_mode"] = String(settings.climateMode);
    climate["humidity_mode"] = String(settings.humidityMode);
    climate["auto_fan_control"] = settings.autoFanControl;
    climate["update_interval_ms"] = settings.updateInterval;
      // Safety limits
    JsonObject safety = climate["safety_limits"].to<JsonObject>();
    safety["max_temperature"] = settings.maxTemperature;
    safety["min_temperature"] = settings.minTemperature;
    safety["max_humidity"] = settings.maxHumidity;
    safety["min_humidity"] = settings.minHumidity;
      // PID parameters
    JsonObject pid = climate["pid_parameters"].to<JsonObject>();
    JsonObject tempPid = pid["temperature"].to<JsonObject>();
    tempPid["kp"] = settings.temperatureKp;
    tempPid["ki"] = settings.temperatureKi;
    tempPid["kd"] = settings.temperatureKd;
    
    JsonObject humPid = pid["humidity"].to<JsonObject>();
    humPid["kp"] = settings.humidityKp;
    humPid["ki"] = settings.humidityKi;
    humPid["kd"] = settings.humidityKd;
      // Control parameters
    JsonObject control = climate["control_parameters"].to<JsonObject>();
    control["temperature_hysteresis"] = settings.temperatureHysteresis;
    control["humidity_hysteresis"] = settings.humidityHysteresis;    // Fan settings
    JsonObject fans = climate["fan_settings"].to<JsonObject>();
    fans["interior_fan_enabled"] = settings.fanInteriorEnabled;
    fans["exterior_fan_enabled"] = settings.fanExteriorEnabled;
    
    // AutoTune results
    JsonObject autoTune = climate["autotune_results"].to<JsonObject>();
    autoTune["has_results"] = settings.hasAutoTuneResults;
    if (settings.hasAutoTuneResults) {
        autoTune["kp"] = settings.autoTuneKp;
        autoTune["ki"] = settings.autoTuneKi;
        autoTune["kd"] = settings.autoTuneKd;
    } else {
        autoTune["kp"] = 0.0;
        autoTune["ki"] = 0.0;
        autoTune["kd"] = 0.0;
    }
      // Metadata
    JsonObject metadata = doc["metadata"].to<JsonObject>();
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

void ClimateConfig::clearEEPROM() {
    Serial.println("DEBUG: ClimateConfig::clearEEPROM() - Clearing EEPROM");
    
    // Clear EEPROM by writing zeros
    for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
    
    Serial.println("DEBUG: ClimateConfig::clearEEPROM() - EEPROM cleared, forcing reload from JSON");
    
    // Force reload from JSON files
    if (loadFromJsonFile("/data/config.json")) {
        Serial.println("DEBUG: ClimateConfig - Successfully reloaded from main config.json after EEPROM clear");
        saveSettings(); // Save to EEPROM
    } else if (loadFromJsonFile()) {
        Serial.println("DEBUG: ClimateConfig - Successfully reloaded from ClimateConfig.json after EEPROM clear");
        saveSettings(); // Save to EEPROM
    } else {
        Serial.println("DEBUG: ClimateConfig - No JSON files available, loading defaults after EEPROM clear");
        loadDefaults();
        saveSettings();
    }
}
