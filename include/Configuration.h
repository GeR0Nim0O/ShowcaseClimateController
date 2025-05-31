#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include "Device.h" 
#include "DS3231rtc.h"

// Forward declaration of DS3231rtc class
class DS3231rtc;

class Configuration {
public:    // Static member variables
    static std::map<String, String> wifiConfig;
    static std::map<String, String> mqttConfig;
    static std::map<String, String> projectConfig;
    static std::map<String, String> customWifiConfig;
    static std::map<String, String> customMqttConfig;
    static std::map<String, String> mqttThrottlingConfig;
    static std::map<String, String> climateControllerConfig;
    static std::map<String, String> displayConfig;
    static std::map<String, String> systemConfig;
    static JsonDocument devicesConfigDoc;  // Changed to JsonDocument to persist the data
    static JsonObject devicesConfig;
      // Config loading functions
    static bool loadConfigFromSD(const char* filename);
    static bool loadConfigFromCodebase();
    static bool loadConfig(const JsonObject& config);
    
    // Setters for WiFi configuration
    static void setWiFiSSID(const String& ssid);
    static void setWiFiPassword(const String& password);
    
    // Getters for WiFi configuration
    static String getWiFiSSID();
    static String getWiFiPassword();
    
    // Setters for MQTT configuration
    static void setMqttsServer(const String& server);
    static void setMqttsPort(int port);
    static void setFlespiToken(const String& token);
    
    // Getters for MQTT configuration
    static String getMqttsServer();
    static int getMqttsPort();
    static String getFlespiToken();
    
    // Setters for project information
    static void setProjectNumber(const String& number);
    static void setShowcaseId(const String& id);
    static void setDeviceName(const String& name);
    static void setTimezone(const String& tz);
    static void setLogFileSize(uint32_t size);
    
    // Getters for project information
    static String getProjectNumber();
    static String getShowcaseId();
    static String getDeviceName();
    static String getTimezone();
    static uint32_t getLogFileSize();
      // Device configuration
    static JsonObject getDevicesConfig();
    static void initializeEachDevice(std::vector<Device*>& devices);
    static bool validateDeviceObject(Device* device);    // Initialize devices from JSON configuration using positional indexing
    static std::vector<Device*> initializeDevices(std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, DS3231rtc*& rtc);
    
    // Print configuration values
    static void printConfigValues();

private:
    // Private helper methods
    static void parseWiFiConfig(const JsonObject& config);
    static void parseMQTTConfig(const JsonObject& config);
    static void parseProjectConfig(const JsonObject& config);
};

#endif // CONFIGURATION_H