#include "Configuration.h"
#include "DeviceRegistry/DeviceRegistry.h"
#include "GP8403dac.h"
#include "PCF8574gpio.h"
#include "SHTsensor.h"
#include "DS3231rtc.h"
#include "BH1705sensor.h"
#include "SCALESsensor.h"

std::map<String, String> Configuration::wifiConfig;
std::map<String, String> Configuration::mqttConfig;
std::map<String, String> Configuration::projectConfig;
JsonObject Configuration::devicesConfig;

bool Configuration::loadConfigFromSD(const char* filename) {
    // ...existing code...
}

void Configuration::printConfigValues() {
    // ...existing code...
}

std::vector<Device*> Configuration::initializeDevices(std::map<uint8_t, std::vector<uint8_t>> &tcaScanResults, DS3231rtc* &rtc) {
    // ...existing code...
    
    // Process each device in the configuration
    for (JsonPair device : devicesConfig) {
        const String deviceKey = device.key().c_str();
        JsonObject deviceObj = device.value();
        
        // Extract common device parameters
        String deviceType = deviceObj["Type"].as<const char*>();
        String deviceTypeNumber = deviceObj["TypeNumber"].as<const char*>();
        uint8_t deviceAddress = (uint8_t)strtol(deviceObj["Address"].as<const char*>(), NULL, 16);
        uint8_t deviceTcaPort = deviceObj["TCA_Port"] | 0; // Default to 0 if not specified
        int deviceIndex = deviceObj["DeviceIndex"] | 0;
        
        // Log device information
        Serial.print("Configuring device: ");
        Serial.print(deviceType);
        Serial.print(" (");
        Serial.print(deviceTypeNumber);
        Serial.print(") at address 0x");
        Serial.print(deviceAddress, HEX);
        Serial.print(" on TCA port ");
        Serial.print(deviceTcaPort);
        Serial.print(", index ");
        Serial.println(deviceIndex);
        
        // Initialize channel parameters
        std::map<String, float> channelThresholds;
        std::map<String, String> channelNames;
        
        // Process channels for this device
        if (deviceObj["Channels"].is<JsonObject>()) {
            for (JsonPair channel : deviceObj["Channels"].as<JsonObject>()) {
                String channelKey = channel.key().c_str();
                JsonObject channelObj = channel.value();
                
                // Extract channel parameters
                float threshold = channelObj["Threshold"] | 0.0;
                String channelName = channelObj["Name"].as<const char*>();
                
                // Log channel information
                Serial.print("  Channel: ");
                Serial.print(channelKey);
                Serial.print(", Name: ");
                Serial.print(channelName);
                Serial.print(", Threshold: ");
                Serial.println(threshold);
                
                // Store channel parameters
                channelThresholds[channelKey] = threshold;
                channelNames[channelKey] = channelName;
            }
        }
        
        // Create the device using DeviceRegistry
        Device* createdDevice = DeviceRegistry::createDeviceWithThresholds(
            deviceType, 
            deviceTypeNumber, 
            &Wire,  // Use default Wire instance
            deviceAddress, 
            deviceTcaPort, 
            channelThresholds, 
            channelNames, 
            deviceIndex
        );
        
        // Special log for DAC device after creation attempt
        if (deviceType.equalsIgnoreCase("DAC")) {
            if (createdDevice != nullptr) {
                Serial.print("DAC device created successfully with index ");
                Serial.println(deviceIndex);
            } else {
                Serial.println("WARNING: DAC device creation failed!");
            }
        }
        
        // ...existing code...
    }
    
    // ...existing code...
}

void Configuration::setWiFiSSID(const String& ssid) {
    wifiConfig["ssid"] = ssid;
}

void Configuration::setWiFiPassword(const String& password) {
    wifiConfig["password"] = password;
}

void Configuration::setMqttsServer(const String& server) {
    mqttConfig["server"] = server;
}

void Configuration::setMqttsPort(int port) {
    mqttConfig["port"] = String(port);
}

void Configuration::setFlespiToken(const String& token) {
    mqttConfig["token"] = token;
}

void Configuration::setProjectNumber(const String& number) {
    projectConfig["number"] = number;
}

void Configuration::setShowcaseId(const String& id) {
    projectConfig["showcase_id"] = id;
}

void Configuration::setDeviceName(const String& name) {
    projectConfig["device"] = name;
}

void Configuration::setTimezone(const String& tz) {
    projectConfig["timezone"] = tz;
}

void Configuration::setLogFileSize(uint32_t size) {
    projectConfig["sd_logfile_size"] = String(size);
}