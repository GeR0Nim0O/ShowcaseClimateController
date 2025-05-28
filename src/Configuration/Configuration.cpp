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
    std::vector<Device*> devices;
    JsonObject devicesConfig = getDevicesConfig();
    
    // Debug TCA scan results
    Serial.println("\nTCA scan results:");
    for (const auto& result : tcaScanResults) {
        Serial.print("TCA Port ");
        Serial.print(result.first);
        Serial.print(": ");
        for (const auto& address : result.second) {
            Serial.print("0x");
            Serial.print(address, HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    // Process each device in the configuration
    for (JsonPair device : devicesConfig) {
        const String deviceKey = device.key().c_str();
        JsonObject deviceObj = device.value();
        
        // Get device type, type number, and I2C address
        String deviceType = deviceObj["Type"].as<String>();
        String deviceTypeNumber = deviceObj["TypeNumber"].as<String>();
        uint8_t deviceAddress = strtol(deviceObj["Address"].as<String>().c_str(), NULL, 16);
        String deviceMode = deviceObj["Mode"] | "";
        
        // Log device details for debugging
        Serial.print("Config Device: ");
        Serial.print(deviceKey);
        Serial.print(", Type: ");
        Serial.print(deviceType);
        Serial.print(", TypeNumber: ");
        Serial.print(deviceTypeNumber);
        Serial.print(", Address: 0x");
        Serial.println(deviceAddress, HEX);

        // For DAC devices, log extra debug info
        if (deviceType.equalsIgnoreCase("DAC")) {
            Serial.println("DAC device found in config:");
            Serial.print("  Key: ");
            Serial.println(deviceKey);
            Serial.print("  TypeNumber: ");
            Serial.println(deviceTypeNumber);
            Serial.print("  Address: 0x");
            Serial.println(deviceAddress, HEX);
            
            Serial.println("  Channel config:");
            for (JsonPair channel : deviceObj["Channels"].as<JsonObject>()) {
                String channelKey = channel.key().c_str();
                JsonObject channelObj = channel.value();
                Serial.print("    ");
                Serial.print(channelKey);
                Serial.print(": ");
                Serial.println(channelObj["Name"].as<const char*>());
            }
        }

        // Find TCA port where this device exists
        int tcaPort = -1;
        for (const auto& result : tcaScanResults) {
            for (const auto& address : result.second) {
                if (address == deviceAddress) {
                    tcaPort = result.first;
                    break;
                }
            }
            if (tcaPort != -1) {
                break;
            }
        }

        // Skip device if it's not found on the I2C bus
        if (tcaPort == -1) {
            Serial.print("Device with address 0x");
            Serial.print(deviceAddress, HEX);
            Serial.println(" not found on any TCA port, skipping...");
            continue;
        }

        // Process channels for this device
        std::map<String, String> channelNames;
        std::map<String, float> channelThresholds;
        
        for (JsonPair channel : deviceObj["Channels"].as<JsonObject>()) {
            String channelKey = channel.key().c_str();
            JsonObject channelObj = channel.value();
            
            if (channelObj["Name"].is<const char*>()) {
                channelNames[channelKey] = channelObj["Name"].as<const char*>();
                
                // Get threshold if available
                if (channelObj["Threshold"].is<float>()) {
                    channelThresholds[channelKey] = channelObj["Threshold"].as<float>();
                } else {
                    // Use default threshold
                    channelThresholds[channelKey] = 1.0f;
                }
            }
        }

        // Special handling for DAC devices - they may not have thresholds
        if (deviceType.equalsIgnoreCase("DAC")) {
            Serial.println("Processing DAC device...");
            
            if (channelNames.empty()) {
                Serial.println("WARNING: DAC device has no defined channels in config!");
            }
            
            for (JsonPair channel : deviceObj["Channels"].as<JsonObject>()) {
                String channelKey = channel.key().c_str();
                JsonObject channelObj = channel.value();
                
                // For DAC channels, the name is sufficient
                if (channelObj["Name"].is<const char*>()) {
                    channelNames[channelKey] = channelObj["Name"].as<const char*>();
                    if (!channelObj["Threshold"].is<float>()) {
                        // No threshold needed for DAC, but set a default for API consistency
                        channelThresholds[channelKey] = 0.1f;
                    }
                }
            }
            
            Serial.print("DAC channels after processing: ");
            Serial.println(channelNames.size());
        }
        
        // Create the device using DeviceRegistry
        static int deviceIndex = 0;
        Device* createdDevice = DeviceRegistry::getInstance().createDeviceWithThresholds(
            deviceType,
            deviceTypeNumber,
            &Wire,
            deviceAddress,
            tcaPort,
            channelThresholds,
            channelNames,
            deviceIndex,
            deviceMode
        );
        
        deviceIndex++;
        
        if (createdDevice != nullptr) {
            devices.push_back(createdDevice);
            
            // Special processing for RTC device
            if (deviceType.equalsIgnoreCase("RTC")) {
                rtc = static_cast<DS3231rtc*>(createdDevice);
                Serial.println("RTC device initialized and assigned");
            }
            
            // Special logging for DAC device
            if (deviceType.equalsIgnoreCase("DAC")) {
                Serial.println("DAC device added to devices vector");
            }
        } else {
            Serial.print("Failed to create device: ");
            Serial.println(deviceKey);
        }
    }

    Serial.println("Device initialization from config complete");
    return devices;
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