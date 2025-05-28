#ifndef CLIMATE_CONFIG_H
#define CLIMATE_CONFIG_H

#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

struct ClimateSettings {
    float temperatureSetpoint;
    float humiditySetpoint;
    float temperatureKp;
    float temperatureKi;
    float temperatureKd;
    float humidityKp;
    float humidityKi;
    float humidityKd;
    bool fanInteriorEnabled;
    bool fanExteriorEnabled;
    uint16_t updateInterval;
    float maxTemperature;
    float minTemperature;
    float maxHumidity;
    float minHumidity;
    uint32_t checksum;
};

class ClimateConfig {
public:
    static ClimateConfig& getInstance();
    
    bool begin();
    void loadDefaults();
    bool saveSettings();
    bool loadSettings();
    
    // Getters
    float getTemperatureSetpoint() const { return settings.temperatureSetpoint; }
    float getHumiditySetpoint() const { return settings.humiditySetpoint; }
    float getTemperatureKp() const { return settings.temperatureKp; }
    float getTemperatureKi() const { return settings.temperatureKi; }
    float getTemperatureKd() const { return settings.temperatureKd; }
    float getHumidityKp() const { return settings.humidityKp; }
    float getHumidityKi() const { return settings.humidityKi; }
    float getHumidityKd() const { return settings.humidityKd; }
    bool getFanInteriorEnabled() const { return settings.fanInteriorEnabled; }
    bool getFanExteriorEnabled() const { return settings.fanExteriorEnabled; }
    uint16_t getUpdateInterval() const { return settings.updateInterval; }
    float getMaxTemperature() const { return settings.maxTemperature; }
    float getMinTemperature() const { return settings.minTemperature; }
    float getMaxHumidity() const { return settings.maxHumidity; }
    float getMinHumidity() const { return settings.minHumidity; }
    
    // Setters
    void setTemperatureSetpoint(float value) { settings.temperatureSetpoint = value; }
    void setHumiditySetpoint(float value) { settings.humiditySetpoint = value; }
    void setTemperaturePID(float kp, float ki, float kd) { 
        settings.temperatureKp = kp; 
        settings.temperatureKi = ki; 
        settings.temperatureKd = kd; 
    }
    void setHumidityPID(float kp, float ki, float kd) { 
        settings.humidityKp = kp; 
        settings.humidityKi = ki; 
        settings.humidityKd = kd; 
    }
    void setFanInteriorEnabled(bool enabled) { settings.fanInteriorEnabled = enabled; }
    void setFanExteriorEnabled(bool enabled) { settings.fanExteriorEnabled = enabled; }
    void setUpdateInterval(uint16_t interval) { settings.updateInterval = interval; }
    
    // Utility
    void printSettings();
    bool validateSettings();

private:
    ClimateConfig() = default;
    ~ClimateConfig() = default;
    ClimateConfig(const ClimateConfig&) = delete;
    ClimateConfig& operator=(const ClimateConfig&) = delete;
    
    ClimateSettings settings;
    static const int EEPROM_SIZE = 512;
    static const int SETTINGS_ADDRESS = 0;
    
    uint32_t calculateChecksum();
    bool isValidChecksum();
};

#endif // CLIMATE_CONFIG_H
