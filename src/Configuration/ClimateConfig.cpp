#include "../../../lib/Config/ClimateConfig/ClimateConfig.h"  // Use relative path to find the header in lib directory
#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

// Debug print helper
#define DEBUG_PRINTLN(x) Serial.println(x)

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
    
    // NO MORE EEPROM - JSON FILES ONLY
    DEBUG_PRINTLN("ClimateConfig::begin() - Attempting to load from SPIFFS ClimateConfig.json");
    
    // First try to load from SPIFFS ClimateConfig.json
    if (loadFromJsonFile()) {
        DEBUG_PRINTLN("ClimateConfig - Successfully loaded from SPIFFS ClimateConfig.json");
        return true;
    }
    
    DEBUG_PRINTLN("ClimateConfig::begin() - SPIFFS ClimateConfig.json failed, trying main config.json");
    // If that fails, try main config.json
    if (loadFromJsonFile("/data/config.json")) {
        return true;
    }
    
    // If both JSON files fail, load defaults and save them to JSON
    Serial.println("No valid JSON file found, loading defaults");
    loadDefaults();
    saveToJsonFile("/data/ClimateConfig.json");
    return true;}

void ClimateConfig::loadDefaults() {
    Serial.println("DEBUG: ClimateConfig::loadDefaults() called");
    settings.temperatureSetpoint = 0.0; // Will be set from config file
    settings.humiditySetpoint = 0.0;    // Will be set from config file
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
    settings.autoTuneKd = 0.0;    // Initialize AutoTune configuration
    settings.autoTuneOutputStep = 100.0;      // Default to 100% output step for normal AutoTune
    settings.fastAutoTuneOutputStep = 100.0;  // Default to 100% output step for fast AutoTune
      Serial.print("DEBUG: ClimateConfig::loadDefaults() - updateInterval set to: ");
    Serial.println(settings.updateInterval);
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

void ClimateConfig::printSettings() {    Serial.println("\n=== Climate Controller Settings ===");
    Serial.print("Temperature Setpoint: ");
    Serial.print(settings.temperatureSetpoint, 2);
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
    if (!climate["temperature_setpoint"]) settings.temperatureSetpoint = 20.0;  // Updated from hardcoded 22.0
    
    settings.humiditySetpoint = climate["humidity_setpoint"].as<double>();
    if (!climate["humidity_setpoint"]) settings.humiditySetpoint = 40.0;  // Updated from hardcoded 50.0
    
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
    }    // Load PID parameters
    JsonObject tempPid = climate["pid_parameters"]["temperature"];
    if (tempPid) {
        settings.temperatureKp = tempPid["kp"].as<double>();
        if (!tempPid["kp"]) settings.temperatureKp = 2.0;
        
        settings.temperatureKi = tempPid["ki"].as<double>();
        if (!tempPid["ki"]) settings.temperatureKi = 0.5;
        
        settings.temperatureKd = tempPid["kd"].as<double>();
        if (!tempPid["kd"]) settings.temperatureKd = 0.1;        // Load normal autotune configuration
        JsonObject normalAutoTune = tempPid["normal_autotune"];
        if (normalAutoTune) {
            if (normalAutoTune["output_step_percent"].is<double>() || normalAutoTune["output_step_percent"].is<int>()) {
                settings.autoTuneOutputStep = normalAutoTune["output_step_percent"].as<double>();
            } else {
                settings.autoTuneOutputStep = 50.0;
            }
            
            Serial.print("Normal AutoTune output step loaded from JSON: ");
            Serial.print(settings.autoTuneOutputStep);
            Serial.println("%");
        } else {
            settings.autoTuneOutputStep = 50.0;
            Serial.println("Using default Normal AutoTune output step: 50%");
        }
          // Load fast autotune configuration
        JsonObject fastAutoTune = tempPid["fast_autotune"];
        if (fastAutoTune) {
            if (fastAutoTune["output_step_percent"].is<double>() || fastAutoTune["output_step_percent"].is<int>()) {
                settings.fastAutoTuneOutputStep = fastAutoTune["output_step_percent"].as<double>();
            } else {
                settings.fastAutoTuneOutputStep = 75.0;
            }
            
            Serial.print("Fast AutoTune output step loaded from JSON: ");
            Serial.print(settings.fastAutoTuneOutputStep);
            Serial.println("%");
        } else {
            settings.fastAutoTuneOutputStep = 75.0;
            Serial.println("Using default Fast AutoTune output step: 75%");
        }
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
    }      // Load AutoTune configuration (fallback for legacy config files)
    JsonObject autoTuneConfig = climate["autotune_config"];
    if (autoTuneConfig) {
        // Only use this if we didn't already load from PID parameters section
        JsonObject tempPid = climate["pid_parameters"]["temperature"];
        JsonObject normalAutoTune = tempPid["normal_autotune"];
        JsonObject fastAutoTune = tempPid["fast_autotune"];
        
        // Only apply legacy config if new format doesn't exist
        if (!tempPid || (!normalAutoTune && !fastAutoTune)) {
            settings.autoTuneOutputStep = autoTuneConfig["output_step"].as<double>();
            if (!autoTuneConfig["output_step"]) settings.autoTuneOutputStep = 50.0;
            
            Serial.print("Legacy AutoTune output step loaded from JSON: ");
            Serial.print(settings.autoTuneOutputStep);
            Serial.println("% (applied to both normal and fast)");
            
            // For legacy configs, use same value for fast autotune
            settings.fastAutoTuneOutputStep = settings.autoTuneOutputStep;
        } else {
            Serial.println("Skipping legacy AutoTune config - new format already loaded");
        }
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
    safety["min_humidity"] = settings.minHumidity;    // PID parameters
    JsonObject pid = climate["pid_parameters"].to<JsonObject>();
    JsonObject tempPid = pid["temperature"].to<JsonObject>();
    tempPid["kp"] = settings.temperatureKp;
    tempPid["ki"] = settings.temperatureKi;
    tempPid["kd"] = settings.temperatureKd;
    tempPid["auto_tuned"] = settings.hasAutoTuneResults;
    tempPid["tuned_timestamp"] = "";
    
    // Normal AutoTune configuration
    JsonObject normalAutoTune = tempPid["normal_autotune"].to<JsonObject>();
    normalAutoTune["enabled"] = false;
    normalAutoTune["target_oscillation_amplitude"] = 1.0;
    normalAutoTune["test_duration_minutes"] = 30;
    normalAutoTune["output_step_percent"] = settings.autoTuneOutputStep;
    normalAutoTune["noise_band"] = 0.05;
    normalAutoTune["lookback_seconds"] = 20;
    
    // Fast AutoTune configuration
    JsonObject fastAutoTune = tempPid["fast_autotune"].to<JsonObject>();
    fastAutoTune["output_step_percent"] = settings.fastAutoTuneOutputStep;
    fastAutoTune["description"] = "Higher values create stronger oscillations for faster tuning";
    
    // AutoTune results
    JsonObject autoTuneResults = tempPid["autotune_results"].to<JsonObject>();
    autoTuneResults["has_results"] = settings.hasAutoTuneResults;
    if (settings.hasAutoTuneResults) {
        autoTuneResults["kp"] = settings.autoTuneKp;
        autoTuneResults["ki"] = settings.autoTuneKi;
        autoTuneResults["kd"] = settings.autoTuneKd;
    } else {
        autoTuneResults["kp"] = 0.0;
        autoTuneResults["ki"] = 0.0;
        autoTuneResults["kd"] = 0.0;
    }    
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
    
    file.close();    Serial.println("Climate settings saved to JSON file successfully");
    return true;
}

bool ClimateConfig::createDefaultJsonFile(const String& filePath) {
    // Load defaults first
    loadDefaults();
      // Save defaults to JSON file
    return saveToJsonFile(filePath);
}
