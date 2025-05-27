#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include "Device.h"
#include "DS3231rtc.h"
#include "I2CHandler.h"

class Configuration {
public:
    static bool loadConfig(const char* path);
    static bool loadConfigFromSD(const char* path);
    static String getWiFiSSID();
    static void setWiFiSSID(const String& ssid);
    static String getWiFiPassword();
    static void setWiFiPassword(const String& password);
    static String getMqttServer();
    static void setMqttServer(const String& server);
    static int getMqttPort();
    static void setMqttPort(int port);
    static String getMqttUsername();
    static void setMqttUsername(const String& username);
    static String getMqttPassword();
    static void setMqttPassword(const String& password);
    static String getMqttsServer();
    static void setMqttsServer(const String& server);
    static int getMqttsPort();
    static void setMqttsPort(int port);
    static String getMqttsUsername();
    static void setMqttsUsername(const String& username);
    static String getMqttsPassword();
    static void setMqttsPassword(const String& password);
    static String getProjectNumber();
    static void setProjectNumber(const String& number);
    static String getShowcaseId();
    static void setShowcaseId(const String& id);
    static String getDeviceName();
    static void setDeviceName(const String& name);
    static String getTimezone();
    static void setTimezone(const String& timezone);
    static size_t getSdLogFileSize();
    static void printConfigValues();
    static JsonObject getDevicesConfig();

    // Add method declarations for reading sensor properties
    static String getSensorType(const JsonObject& sensorConfig);
    static String getSensorTypeNumber(const JsonObject& sensorConfig);
    static uint8_t getSensorAddress(const JsonObject& sensorConfig);
    static float getSensorThreshold(const JsonObject& sensorConfig, const String& channelKey = "");
    static JsonObject getSensorChannels(const JsonObject& sensorConfig);

    // Modified to work with new channel structure
    static std::map<String, String> getChannelNames(const JsonObject& sensorConfig);
    static std::map<String, float> getChannelThresholds(const JsonObject& sensorConfig);

    // New method to check if config uses the new format
    static bool isNewConfigFormat(const JsonObject& sensorConfig);

    // New functions for device initialization and printing
    static std::vector<Device*> initializeDevices(const std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, DS3231rtc*& rtc);
    static void initializeEachDevice(const std::vector<Device*>& devices);

    // New function to convert JsonObject to map
    static std::map<String, String> jsonToMap(JsonObject jsonObject);

    // Methods to handle flespi token
    static String getFlespiToken();
    static void setFlespiToken(const String& token);

private:
    static JsonDocument configDoc;
    static std::vector<std::pair<uint8_t, uint8_t>> i2cAddressList;
};

#endif // CONFIGURATION_H