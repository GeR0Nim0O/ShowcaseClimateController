#include "Configuration.h"
#include "SDHandler.h"
#include "DeviceRegistry/DeviceRegistry.h"
#include <SD_MMC.h>
#include <vector>

JsonDocument Configuration::configDoc; 
std::vector<std::pair<uint8_t, uint8_t>> Configuration::i2cAddressList;

bool Configuration::loadConfig(const char* path) {
    Serial.print("Loading config from: ");
    Serial.println(path);

    File configFile = SD_MMC.open(path);
    if (!configFile) {
        Serial.println("Failed to open config file for reading");
        return false;
    }

    String fileContent;
    while (configFile.available()) {
        fileContent += String((char)configFile.read());
    }
    configFile.close();

    Serial.println("Config file content:");
    Serial.println(fileContent);

    DeserializationError error = deserializeJson(configDoc, fileContent);
    if (error) {
        Serial.print("Failed to parse config file: ");
        Serial.println(error.f_str());
        return false;
    }

    Serial.println("Config loaded successfully");
    return true;
}

bool Configuration::loadConfigFromSD(const char* path) {
    File file = SD_MMC.open(path);
    if (!file) {
        Serial.println("Failed to open config file");
        return false;
    }

    size_t size = file.size();
    if (size > 8192) {
        Serial.println("Config file size is too large");
        file.close();
        return false;
    }

    std::unique_ptr<char[]> buf(new char[size]);
    file.readBytes(buf.get(), size);
    file.close();

    auto error = deserializeJson(configDoc, buf.get());
    if (error) {
        Serial.println("Failed to parse config file");
        return false;
    }

    printConfigValues();
    return true;
}

void Configuration::printConfigValues() {
    Serial.println("Configuration loaded:");
    Serial.print("WiFi SSID: ");
    Serial.println(getWiFiSSID());
    Serial.print("WiFi Password: ");
    Serial.println(getWiFiPassword());
    Serial.print("MQTT Server: ");
    Serial.println(getMqttsServer());
    Serial.print("MQTT Port: ");
    Serial.println(getMqttsPort());
    Serial.print("Project Number: ");
    Serial.println(getProjectNumber());
    Serial.print("Showcase ID: ");
    Serial.println(getShowcaseId());
    Serial.print("Timezone: ");
    Serial.println(getTimezone());
    Serial.print("Device Name: ");
    Serial.println(getDeviceName());
    Serial.print("Flespi Token: ");
    // Print first few characters of token for security
    String token = getFlespiToken();
    if (token.length() > 8) {
        Serial.print(token.substring(0, 4) + "..." + token.substring(token.length() - 4));
    } else {
        Serial.print("Not set");
    }
    Serial.println();
}

String Configuration::getWiFiSSID() {
    return configDoc["wifi"]["ssid"].as<String>();
}

void Configuration::setWiFiSSID(const String& ssid) {
    configDoc["wifi"]["ssid"] = ssid;
}

String Configuration::getWiFiPassword() {
    return configDoc["wifi"]["password"].as<String>();
}

void Configuration::setWiFiPassword(const String& password) {
    configDoc["wifi"]["password"] = password;
}

String Configuration::getMqttServer() {
    return configDoc["mqtt"]["server"].as<String>();
}

void Configuration::setMqttServer(const String& server) {
    configDoc["mqtt"]["server"] = server;
}

int Configuration::getMqttPort() {
    return configDoc["mqtt"]["port"].as<int>();
}

void Configuration::setMqttPort(int port) {
    configDoc["mqtt"]["port"] = port;
}

String Configuration::getMqttUsername() {
    return configDoc["mqtt"]["username"].as<String>();
}

void Configuration::setMqttUsername(const String& username) {
    configDoc["mqtt"]["username"] = username;
}

String Configuration::getMqttPassword() {
    return configDoc["mqtt"]["password"].as<String>();
}

void Configuration::setMqttPassword(const String& password) {
    configDoc["mqtt"]["password"] = password;
}

String Configuration::getDeviceName() {
    return configDoc["project"]["device"].as<String>();
}

void Configuration::setDeviceName(const String& name) {
    configDoc["project"]["device"] = name;
}

String Configuration::getProjectNumber() {
    return configDoc["project"]["number"].as<String>();
}

void Configuration::setProjectNumber(const String& number) {
    configDoc["project"]["number"] = number;
}

String Configuration::getShowcaseId() {
    return configDoc["project"]["showcase_id"].as<String>();
}

void Configuration::setShowcaseId(const String& id) {
    configDoc["project"]["showcase_id"] = id;
}

String Configuration::getTimezone() {
    return configDoc["timezone"].as<String>();
}

void Configuration::setTimezone(const String& timezone) {
    configDoc["timezone"] = timezone;
}

size_t Configuration::getSdLogFileSize() {
    return configDoc["sd_logfile_size"].as<size_t>();
}

String Configuration::getMqttsServer() {
    return configDoc["mqtts"]["server"].as<String>();
}

void Configuration::setMqttsServer(const String& server) {
    configDoc["mqtts"]["server"] = server;
}

int Configuration::getMqttsPort() {
    return configDoc["mqtts"]["port"].as<int>();
}

void Configuration::setMqttsPort(int port) {
    configDoc["mqtts"]["port"] = port;
}

String Configuration::getMqttsUsername() {
    return configDoc["mqtts"]["username"].as<String>();
}

void Configuration::setMqttsUsername(const String& username) {
    configDoc["mqtts"]["username"] = username;
}

String Configuration::getMqttsPassword() {
    return configDoc["mqtts"]["password"].as<String>();
}

void Configuration::setMqttsPassword(const String& password) {
    configDoc["mqtts"]["password"] = password;
}

String Configuration::getFlespiToken() {
    return configDoc["mqtts"]["flespi_token"].as<String>();
}

void Configuration::setFlespiToken(const String& token) {
    configDoc["mqtts"]["flespi_token"] = token;
}

JsonObject Configuration::getDevicesConfig() {
    return configDoc["Devices"].as<JsonObject>();
}

String Configuration::getSensorType(const JsonObject& sensorConfig) {
    return sensorConfig["Type"].as<String>();
}

String Configuration::getSensorTypeNumber(const JsonObject& sensorConfig) {
    return sensorConfig["TypeNumber"].as<String>();
}

uint8_t Configuration::getSensorAddress(const JsonObject& sensorConfig) {
    return strtol(sensorConfig["Address"].as<const char*>(), nullptr, 16);
}

float Configuration::getSensorThreshold(const JsonObject& sensorConfig, const String& channelKey) {
    // If using new format and a specific channel is requested, return channel-specific threshold
    if (isNewConfigFormat(sensorConfig) && channelKey.length() > 0) {
        if (sensorConfig["Channels"].containsKey(channelKey) && 
            sensorConfig["Channels"][channelKey].containsKey("Threshold")) {
            return sensorConfig["Channels"][channelKey]["Threshold"].as<float>();
        }
    }
    
    // Fall back to sensor-level threshold for backward compatibility
    if (sensorConfig.containsKey("Threshold")) {
        return sensorConfig["Threshold"].as<float>();
    }
    
    // Default threshold if none specified
    return 1.0f;
}

JsonObject Configuration::getSensorChannels(const JsonObject& sensorConfig) {
    return sensorConfig["Channels"].as<JsonObject>();
}

bool Configuration::isNewConfigFormat(const JsonObject& sensorConfig) {
    // Check if any channel has a "Threshold" property
    if (sensorConfig.containsKey("Channels")) {
        JsonObject channels = sensorConfig["Channels"].as<JsonObject>();
        for (JsonPair channel : channels) {
            // If the channel is an object with a Threshold field, we're using the new format
            if (channel.value().is<JsonObject>() && channel.value().as<JsonObject>().containsKey("Threshold")) {
                return true;
            }
        }
    }
    return false;
}

std::map<String, String> Configuration::getChannelNames(const JsonObject& sensorConfig) {
    std::map<String, String> channelMap;
    
    if (sensorConfig.containsKey("Channels")) {
        JsonObject channels = sensorConfig["Channels"].as<JsonObject>();
        
        if (isNewConfigFormat(sensorConfig)) {
            // New format: "Channels": { "T": { "Name": "Temperature", "Threshold": 0.3 } }
            for (JsonPair channel : channels) {
                String key = String(channel.key().c_str());
                String name = channel.value()["Name"].as<String>();
                channelMap[key] = name;
            }
        } else {
            // Old format: "Channels": { "Temperature": "T" }
            for (JsonPair channel : channels) {
                String name = String(channel.key().c_str());
                String key = channel.value().as<String>();
                channelMap[key] = name;
            }
        }
    }
    
    return channelMap;
}

std::map<String, float> Configuration::getChannelThresholds(const JsonObject& sensorConfig) {
    std::map<String, float> thresholdMap;
    float defaultThreshold = getSensorThreshold(sensorConfig);
    
    if (sensorConfig.containsKey("Channels")) {
        JsonObject channels = sensorConfig["Channels"].as<JsonObject>();
        
        if (isNewConfigFormat(sensorConfig)) {
            // New format with per-channel thresholds
            for (JsonPair channel : channels) {
                String key = String(channel.key().c_str());
                float threshold = channel.value()["Threshold"].as<float>();
                thresholdMap[key] = threshold;
            }
        } else {
            // Old format - use the same threshold for all channels
            for (JsonPair channel : channels) {
                String key = channel.value().as<String>();
                thresholdMap[key] = defaultThreshold;
            }
        }
    }
    
    return thresholdMap;
}

std::vector<Device*> Configuration::initializeDevices(const std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, DS3231rtc*& rtc) {
    std::vector<Device*> devices;
    std::map<uint8_t, int> deviceIndexMap;
    JsonObject devicesConfig = getDevicesConfig();

    for (const auto& portDevices : tcaScanResults) {
        uint8_t tcaPort = portDevices.first;
        for (uint8_t address : portDevices.second) {
            i2cAddressList.push_back(std::make_pair(tcaPort, address));
        }
    }

    for (const auto& i2cAddress : i2cAddressList) {
        uint8_t tcaPort = i2cAddress.first;
        uint8_t address = i2cAddress.second;

        for (JsonPair deviceConfigPair : devicesConfig) {
            JsonObject deviceConfig = deviceConfigPair.value().as<JsonObject>();
            if (getSensorAddress(deviceConfig) == address) {
                String type = getSensorType(deviceConfig);
                String typeNumber = getSensorTypeNumber(deviceConfig);
                float threshold = getSensorThreshold(deviceConfig);
                std::map<String, String> channels = jsonToMap(deviceConfig["Channels"].as<JsonObject>());

                int deviceIndex = deviceIndexMap[address]++;

                Serial.print("Creating device with type: ");
                Serial.print(type);
                Serial.print(", type number: ");
                Serial.print(typeNumber);
                Serial.print(", address: 0x");
                Serial.print(address, HEX);
                Serial.print(", TCA port: ");
                Serial.println(tcaPort);

                // Get channel names and thresholds
                std::map<String, String> channelNames = getChannelNames(deviceConfig);
                std::map<String, float> channelThresholds = getChannelThresholds(deviceConfig);

                // Print threshold debug information
                Serial.print("Channel thresholds for ");
                Serial.print(type);
                Serial.println(":");
                for (const auto& threshold : channelThresholds) {
                    Serial.print("  ");
                    Serial.print(threshold.first);
                    Serial.print(": ");
                    Serial.println(threshold.second);
                }

                Device* device = DeviceRegistry::createDeviceWithThresholds(
                    type, typeNumber, &Wire, address, tcaPort, 
                    channelThresholds, channelNames, deviceIndex);
                
                if (device && type.equalsIgnoreCase("RTC") && typeNumber.equalsIgnoreCase("DS3231")) {
                    rtc = static_cast<DS3231rtc*>(device);
                }
                if (device) {
                    devices.push_back(device);
                    Serial.println(type + " created with index " + String(deviceIndex));
                } else {
                    Serial.println("Unknown device type or address: " + type + " " + typeNumber + " 0x" + String(address, HEX));
                }
            }
        }
    }

    return devices;
}

void Configuration::initializeEachDevice(const std::vector<Device*>& devices) {
    for (Device* device : devices) {
        I2CHandler::selectTCA(device->getTcaPort());
        if (!device->begin()) {
            Serial.print("Failed to initialize device: ");
            Serial.print(device->getType());
            Serial.print(" ");
            Serial.print(device->getTypeNumber());
            Serial.print(" at address 0x");
            Serial.println(device->getAddress(), HEX);
        } else {
            Serial.println("Device initialized successfully: " + device->getType() + " " + device->getTypeNumber());
            Serial.print("Address: 0x");
            Serial.println(device->getAddress(), HEX);
            Serial.print("Threshold: ");
            Serial.println(device->getThreshold());
            Serial.print("Number of Channels: ");
            Serial.println(device->getNumChannels());
        }
    }
}

std::map<String, String> Configuration::jsonToMap(JsonObject jsonObject) {
    std::map<String, String> map;
    for (JsonPair kv : jsonObject) {
        map[String(kv.key().c_str())] = kv.value().as<String>();
    }
    return map;
}

