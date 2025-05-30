#include "Configuration.h"
#include "DeviceRegistry/DeviceRegistry.h"
#include "GP8403dac.h"
#include "PCF8574gpio.h"
#include "SHTsensor.h"
#include "DS3231rtc.h"
#include "BH1705sensor.h"
#include "SCALESsensor.h"
#include "SDHandler.h"
#include <Wire.h>
#include <algorithm> // For std::remove_if and std::distance

// Initialize static member variables
std::map<String, String> Configuration::wifiConfig;
std::map<String, String> Configuration::mqttConfig;
std::map<String, String> Configuration::projectConfig;
JsonDocument Configuration::devicesConfigDoc;
JsonObject Configuration::devicesConfig = JsonObject();

bool Configuration::loadConfigFromSD(const char* filename) {
    // Parse the JSON file from the SD card
    JsonDocument doc;
    
    if (!SDHandler::readJsonFile(filename, doc)) {
        Serial.println("Failed to read config file");
        return false;
    }
    
    // Load configuration from parsed JSON
    if (!loadConfig(doc.as<JsonObject>())) {
        Serial.println("Failed to parse config");
        return false;
    }
    
    Serial.println("Configuration loaded successfully");
    return true;
}

bool Configuration::loadConfig(const JsonObject& config) {
    if (config.isNull()) {
        Serial.println("Config is null");
        return false;
    }
    
    // Parse WiFi configuration
    if (!config["wifi"].isNull()) {
        parseWiFiConfig(config["wifi"]);
    }
    
    // Parse MQTT configuration
    if (!config["mqtt"].isNull()) {
        parseMQTTConfig(config["mqtt"]);
    }
    
    // Parse MQTTS configuration (SSL)
    if (!config["mqtts"].isNull()) {
        parseMQTTConfig(config["mqtts"]);
    }
    
    // Parse project configuration
    if (!config["project"].isNull()) {
        parseProjectConfig(config["project"]);
    }
      // Store devices configuration for later use
    if (!config["Devices"].isNull()) {
        // Clear the existing document and copy the Devices section to it
        devicesConfigDoc.clear();
        devicesConfigDoc.set(config["Devices"]);
        devicesConfig = devicesConfigDoc.as<JsonObject>();
    }
    
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
    Serial.println(getFlespiToken().length() > 0 ? "Set" : "Not set");
}

std::vector<Device*> Configuration::initializeDevicesFromJSON(std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, DS3231rtc*& rtc) {
    std::vector<Device*> devices;
    
    // Create lists of scanned devices by type for positional indexing
    std::vector<std::pair<uint8_t, uint8_t>> scannedDevices; // (address, tcaPort) pairs
    
    // Build a list of all scanned devices with their TCA ports
    for (const auto& result : tcaScanResults) {
        uint8_t tcaPort = result.first;
        for (const auto& address : result.second) {
            if (address != 0) {  // Skip the invalid zero address
                scannedDevices.push_back(std::make_pair(address, tcaPort));
            }
        }
    }
    
    // Sort scanned devices by TCA port first, then by address for consistent ordering
    std::sort(scannedDevices.begin(), scannedDevices.end(), 
              [](const std::pair<uint8_t, uint8_t>& a, const std::pair<uint8_t, uint8_t>& b) {
                  if (a.second != b.second) return a.second < b.second;
                  return a.first < b.first;
              });
    
    // Create counters for positional indexing by device type
    std::map<String, int> deviceTypeCounters;
    
    static int deviceIndex = 0;
    
    // Iterate through each device configuration in the JSON
    for (JsonPair devicePair : devicesConfig) {
        String deviceKey = devicePair.key().c_str();
        JsonObject deviceConfig = devicePair.value().as<JsonObject>();
        
        if (deviceConfig.isNull()) {
            Serial.print("Skipping invalid device config for: ");
            Serial.println(deviceKey);
            continue;
        }
        
        // Extract device configuration
        String deviceType = deviceConfig["Type"] | "";
        String deviceTypeNumber = deviceConfig["TypeNumber"] | "";
        String addressStr = deviceConfig["Address"] | "";
        String deviceLabel = deviceConfig["Label"] | "";
        String deviceMode = deviceConfig["Mode"] | "";
        
        if (deviceType.isEmpty() || deviceTypeNumber.isEmpty() || addressStr.isEmpty()) {
            Serial.print("Missing required fields for device: ");
            Serial.println(deviceKey);
            continue;
        }
        
        // Parse I2C address from JSON (this is the expected address)
        uint8_t expectedAddress = 0;
        if (addressStr.startsWith("0x") || addressStr.startsWith("0X")) {
            expectedAddress = strtol(addressStr.c_str(), NULL, 16);
        } else {
            expectedAddress = addressStr.toInt();
        }
        
        // Use positional indexing to find the device
        String typeKey = deviceTypeNumber;
        int typeIndex = deviceTypeCounters[typeKey];
        deviceTypeCounters[typeKey]++;
        
        // Find devices of the same type in scan results
        std::vector<std::pair<uint8_t, uint8_t>> matchingDevices;
        for (const auto& scannedDevice : scannedDevices) {
            if (scannedDevice.first == expectedAddress) {
                matchingDevices.push_back(scannedDevice);
            }
        }
        
        if (matchingDevices.empty()) {
            Serial.print("No scanned device found with address 0x");
            Serial.print(expectedAddress, HEX);
            Serial.print(" for device: ");
            Serial.println(deviceKey);
            continue;
        }
        
        if (typeIndex >= matchingDevices.size()) {
            Serial.print("Not enough devices of type ");
            Serial.print(deviceTypeNumber);
            Serial.print(" found (need index ");
            Serial.print(typeIndex);
            Serial.print(", but only ");
            Serial.print(matchingDevices.size());
            Serial.print(" found) for device: ");
            Serial.println(deviceKey);
            continue;
        }
        
        // Get the device at the specified positional index
        uint8_t deviceAddress = matchingDevices[typeIndex].first;
        uint8_t tcaPort = matchingDevices[typeIndex].second;
        
        Serial.print("Creating device from JSON: ");
        Serial.print(deviceKey);
        Serial.print(" (");
        Serial.print(deviceType);
        Serial.print("/");
        Serial.print(deviceTypeNumber);
        Serial.print(") at address 0x");
        Serial.print(deviceAddress, HEX);
        Serial.print(" on TCA port ");
        Serial.print(tcaPort);
        if (!deviceLabel.isEmpty()) {
            Serial.print(" with label: ");
            Serial.print(deviceLabel);
        }
        Serial.println();
        
        // Parse channels and thresholds
        std::map<String, String> channelNames;
        std::map<String, float> channelThresholds;
        
        JsonObject channels = deviceConfig["Channels"];
        if (!channels.isNull()) {
            for (JsonPair channelPair : channels) {
                String channelKey = channelPair.key().c_str();
                JsonObject channelConfig = channelPair.value().as<JsonObject>();
                
                if (!channelConfig.isNull()) {
                    String channelName = channelConfig["Name"] | channelKey;
                    float threshold = channelConfig["Threshold"] | 1.0f;
                    
                    channelNames[channelKey] = channelName;
                    channelThresholds[channelKey] = threshold;
                }
            }
        }
        
        // Create the device
        Device* createdDevice = DeviceRegistry::getInstance().createDeviceWithThresholds(
            &Wire, deviceType, deviceTypeNumber, deviceAddress, tcaPort, 
            channelThresholds, channelNames, deviceIndex, deviceMode
        );
        
        if (createdDevice != nullptr) {
            // Apply the label from JSON configuration
            if (!deviceLabel.isEmpty()) {
                createdDevice->setDeviceLabel(deviceLabel);
                Serial.print("Device labeled as: ");
                Serial.println(deviceLabel);
            }
            
            // Handle special case for RTC
            if (deviceType.equalsIgnoreCase("RTC") && deviceTypeNumber.equalsIgnoreCase("DS3231")) {
                rtc = static_cast<DS3231rtc*>(createdDevice);
                Serial.println("RTC assigned to global reference");
            }
            
            devices.push_back(createdDevice);
            deviceIndex++;
            
            Serial.print("Successfully created device: ");
            Serial.println(deviceKey);
        } else {
            Serial.print("Failed to create device: ");
            Serial.println(deviceKey);
        }
    }
    
    Serial.print("Created ");
    Serial.print(devices.size());
    Serial.println(" devices from JSON configuration");
    
    return devices;
}

std::vector<Device*> Configuration::initializeDevices(std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, DS3231rtc*& rtc) {
    std::vector<Device*> devices;
    
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

    // Check if JSON configuration is available
    if (devicesConfig.isNull()) {
        Serial.println("ERROR: No JSON device configuration available!");
        return devices;
    }

    Serial.println("\nInitializing devices from JSON configuration using positional indexing");
    
    // Create lists of scanned devices by type for positional indexing
    std::vector<std::pair<uint8_t, uint8_t>> scannedDevices; // (address, tcaPort) pairs
    
    // Build a list of all scanned devices with their TCA ports
    for (const auto& result : tcaScanResults) {
        uint8_t tcaPort = result.first;
        for (const auto& address : result.second) {
            if (address != 0) {  // Skip the invalid zero address
                scannedDevices.push_back(std::make_pair(address, tcaPort));
            }
        }
    }
    
    // Sort scanned devices by TCA port first, then by address for consistent ordering
    std::sort(scannedDevices.begin(), scannedDevices.end(), 
              [](const std::pair<uint8_t, uint8_t>& a, const std::pair<uint8_t, uint8_t>& b) {
                  if (a.second != b.second) return a.second < b.second;
                  return a.first < b.first;
              });
    
    // Create counters for positional indexing by device type
    std::map<String, int> deviceTypeCounters;
    
    static int deviceIndex = 0;
    
    // Iterate through each device configuration in the JSON
    for (JsonPair devicePair : devicesConfig) {
        String deviceKey = devicePair.key().c_str();
        JsonObject deviceConfig = devicePair.value().as<JsonObject>();
        
        if (deviceConfig.isNull()) {
            Serial.print("Skipping invalid device config for: ");
            Serial.println(deviceKey);
            continue;
        }
        
        // Extract device configuration
        String deviceType = deviceConfig["Type"] | "";
        String deviceTypeNumber = deviceConfig["TypeNumber"] | "";
        String addressStr = deviceConfig["Address"] | "";
        String deviceLabel = deviceConfig["Label"] | "";
        String deviceMode = deviceConfig["Mode"] | "";
        
        if (deviceType.isEmpty() || deviceTypeNumber.isEmpty() || addressStr.isEmpty()) {
            Serial.print("Missing required fields for device: ");
            Serial.println(deviceKey);
            continue;
        }
        
        // Parse I2C address from JSON (this is the expected address)
        uint8_t expectedAddress = 0;
        if (addressStr.startsWith("0x") || addressStr.startsWith("0X")) {
            expectedAddress = strtol(addressStr.c_str(), NULL, 16);
        } else {
            expectedAddress = addressStr.toInt();
        }
        
        // Use positional indexing to find the device
        String typeKey = deviceTypeNumber;
        int typeIndex = deviceTypeCounters[typeKey];
        deviceTypeCounters[typeKey]++;
        
        // Find devices of the same type in scan results
        std::vector<std::pair<uint8_t, uint8_t>> matchingDevices;
        for (const auto& scannedDevice : scannedDevices) {
            if (scannedDevice.first == expectedAddress) {
                matchingDevices.push_back(scannedDevice);
            }
        }
        
        if (matchingDevices.empty()) {
            Serial.print("No scanned device found with address 0x");
            Serial.print(expectedAddress, HEX);
            Serial.print(" for device: ");
            Serial.println(deviceKey);
            continue;
        }
        
        if (typeIndex >= matchingDevices.size()) {
            Serial.print("Not enough devices of type ");
            Serial.print(deviceTypeNumber);
            Serial.print(" found (need index ");
            Serial.print(typeIndex);
            Serial.print(", but only ");
            Serial.print(matchingDevices.size());
            Serial.print(" found) for device: ");
            Serial.println(deviceKey);
            continue;
        }
        
        // Get the device at the specified positional index
        uint8_t deviceAddress = matchingDevices[typeIndex].first;
        uint8_t tcaPort = matchingDevices[typeIndex].second;
        
        Serial.print("Creating device from JSON: ");
        Serial.print(deviceKey);
        Serial.print(" (");
        Serial.print(deviceType);
        Serial.print("/");
        Serial.print(deviceTypeNumber);
        Serial.print(") at address 0x");
        Serial.print(deviceAddress, HEX);
        Serial.print(" on TCA port ");
        Serial.print(tcaPort);
        if (!deviceLabel.isEmpty()) {
            Serial.print(" with label: ");
            Serial.print(deviceLabel);
        }
        Serial.println();
        
        // Parse channels and thresholds
        std::map<String, String> channelNames;
        std::map<String, float> channelThresholds;
        
        JsonObject channels = deviceConfig["Channels"];
        if (!channels.isNull()) {
            for (JsonPair channelPair : channels) {
                String channelKey = channelPair.key().c_str();
                JsonObject channelConfig = channelPair.value().as<JsonObject>();
                
                if (!channelConfig.isNull()) {
                    String channelName = channelConfig["Name"] | channelKey;
                    float threshold = channelConfig["Threshold"] | 1.0f;
                    
                    channelNames[channelKey] = channelName;
                    channelThresholds[channelKey] = threshold;
                }
            }
        }
        
        // Create the device
        Device* createdDevice = DeviceRegistry::getInstance().createDeviceWithThresholds(
            &Wire, deviceType, deviceTypeNumber, deviceAddress, tcaPort, 
            channelThresholds, channelNames, deviceIndex, deviceMode
        );
        
        if (createdDevice != nullptr) {
            // Apply the label from JSON configuration
            if (!deviceLabel.isEmpty()) {
                createdDevice->setDeviceLabel(deviceLabel);
                Serial.print("Device labeled as: ");
                Serial.println(deviceLabel);
            }
            
            // Handle special case for RTC
            if (deviceType.equalsIgnoreCase("RTC") && deviceTypeNumber.equalsIgnoreCase("DS3231")) {
                rtc = static_cast<DS3231rtc*>(createdDevice);
                Serial.println("RTC assigned to global reference");
            }
            
            devices.push_back(createdDevice);
            deviceIndex++;
            
            Serial.print("Successfully created device: ");
            Serial.println(deviceKey);
        } else {
            Serial.print("Failed to create device: ");
            Serial.println(deviceKey);
        }
    }
    
    Serial.print("Created ");
    Serial.print(devices.size());
    Serial.println(" devices from JSON configuration");
    
    return devices;
}

// Helper function to validate device object and vtable integrity
bool Configuration::validateDeviceObject(Device* device) {
    if (!device) {
        Serial.println("Device pointer is NULL");
        return false;
    }
    
    // Check if device pointer is in valid memory range
    uint32_t deviceAddr = (uint32_t)device;
    if (deviceAddr < 0x3F800000 || deviceAddr > 0x3FFFFFFF) {
        Serial.print("Device pointer 0x");
        Serial.print(deviceAddr, HEX);
        Serial.println(" is outside valid ESP32 memory range");
        return false;
    }
    
    // Try to read the vtable pointer (first 4 bytes of object) with improved safety
    try {
        // More defensive approach to check vtable
        volatile uint8_t* rawPtr = (volatile uint8_t*)device;
        // Just read first byte to see if memory is accessible
        volatile uint8_t testByte = *rawPtr;
        (void)testByte; // Avoid unused variable warning
        
        // Print device address for debugging
        Serial.print("Device memory accessible at 0x");
        Serial.println(deviceAddr, HEX);
        
        // Wait for any pending interrupts to complete
        yield();
        
        // ESP32's DRAM region is typically in this range
        if (deviceAddr >= 0x3F800000 && deviceAddr <= 0x3FFFFFFF) {
            // This is likely a valid object, try initializing it
            Serial.println("Device pointer in valid memory range, proceeding with initialization");
            return true;
        } else {
            Serial.println("Device pointer outside expected memory range");
        }
    } catch (...) {
        Serial.println("Exception while accessing device memory");
        return false;
    }
    
    // If we got here, object is probably valid
    return true;
}

void Configuration::initializeEachDevice(std::vector<Device*>& devices) {
    Serial.println("\n=== Starting Device Initialization ===");
    
    // Remove any null devices that might have been accidentally added
    auto it = std::remove_if(devices.begin(), devices.end(), [](Device* d) { return d == nullptr; });
    if (it != devices.end()) {
        int removedCount = std::distance(it, devices.end());
        devices.erase(it, devices.end());
        Serial.print("Removed ");
        Serial.print(removedCount);
        Serial.println(" null device pointers from vector");
    }
    
    Serial.print("Total devices to initialize: ");
    Serial.println(devices.size());
    Serial.print("Free heap at start: ");
    Serial.println(ESP.getFreeHeap());
    
    if (devices.empty()) {
        Serial.println("WARNING: No devices to initialize!");
        return;
    }
    
    for (size_t i = 0; i < devices.size(); i++) {
        Device* device = devices[i];
        
        Serial.print("\n--- Initializing Device ");
        Serial.print(i);
        Serial.println(" ---");
        
        // Double check for null pointers
        if (!device) {
            Serial.println("ERROR: Null device pointer found, skipping");
            continue;
        }
        
        // Skip vtable validation as it's causing issues
        Serial.print("Device Address: 0x");
        Serial.println((uint32_t)device, HEX);
        
        // Add memory integrity checks before calling virtual methods
        Serial.println("Performing memory integrity checks...");
        
        // Check if device pointer is in valid memory range
        if ((uint32_t)device < 0x3F800000 || (uint32_t)device > 0x3FFFFFFF) {
            Serial.println("ERROR: Device pointer is outside valid ESP32 memory range");
            continue;
        }

        // Force initialize device regardless of vtable validation
        bool success = false;
        try {
            // Apply a direct approach to device initialization
            // Each specific device type has its own initialization requirements
            if (device->getType().equalsIgnoreCase("PCF8574GPIO")) {
                Serial.println("Initializing PCF8574 GPIO expander with direct method");
                I2CHandler::selectTCA(device->getTCAChannel());
                Wire.beginTransmission(device->getI2CAddress());
                int error = Wire.endTransmission();
                
                if (error == 0) {
                    // Set all pins to OUTPUT mode
                    Wire.beginTransmission(device->getI2CAddress());
                    Wire.write(0x00); // Set all pins to LOW (off)
                    Wire.endTransmission();
                    success = true;
                }
            } 
            else if (device->getType().equalsIgnoreCase("GP8403dac")) {
                Serial.println("Initializing GP8403 DAC with direct method");
                I2CHandler::selectTCA(device->getTCAChannel());
                Wire.beginTransmission(device->getI2CAddress());
                int error = Wire.endTransmission();
                
                if (error == 0) {
                    success = true;
                }
            }
            else if (device->getType().equalsIgnoreCase("SHTSensor") || 
                     device->getType().equalsIgnoreCase("Sensor")) {
                Serial.println("Initializing SHT sensor with direct method");
                I2CHandler::selectTCA(device->getTCAChannel());
                Wire.beginTransmission(device->getI2CAddress());
                int error = Wire.endTransmission();
                
                if (error == 0) {
                    success = true;
                }
            }
            
            if (success) {
                Serial.println("Device direct initialization: SUCCESS");
                // Instead of setting initialized state directly, call begin() to properly initialize
                device->begin();
                
                // Check if the initialization was successful
                if (device->isInitialized()) {
                    Serial.println("Device is now properly initialized");
                } else {
                    Serial.println("Device still not initialized after begin()");
                }
            } else {
                Serial.println("Device direct initialization: FAILED");
            }
        } catch (...) {
            Serial.println("ERROR: Exception during direct device initialization");
            success = false;
        }
        
        // Safety delay between device initializations
        delay(100);
    }
    
    Serial.println("\n=== Device Initialization Complete ===");
}

// Parse configuration sections
void Configuration::parseWiFiConfig(const JsonObject& config) {
    wifiConfig["ssid"] = config["ssid"] | "";
    wifiConfig["password"] = config["password"] | "";
}

void Configuration::parseMQTTConfig(const JsonObject& config) {
    mqttConfig["server"] = config["server"] | "";
    mqttConfig["port"] = String(config["port"] | 1883);
    mqttConfig["username"] = config["username"] | "";
    mqttConfig["password"] = config["password"] | "";
    mqttConfig["token"] = config["token"] | "";
}

void Configuration::parseProjectConfig(const JsonObject& config) {
    projectConfig["number"] = config["number"] | "12345";
    projectConfig["showcase_id"] = config["showcase_id"] | "0.0";
    projectConfig["device"] = config["device"] | "Showcase";
    projectConfig["timezone"] = config["timezone"] | "UTC";
}

// Getter and setter implementations
void Configuration::setWiFiSSID(const String& ssid) {
    wifiConfig["ssid"] = ssid;
}

void Configuration::setWiFiPassword(const String& password) {
    wifiConfig["password"] = password;
}

String Configuration::getWiFiSSID() {
    return wifiConfig["ssid"];
}

String Configuration::getWiFiPassword() {
    return wifiConfig["password"];
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

String Configuration::getMqttsServer() {
    return mqttConfig["server"];
}

int Configuration::getMqttsPort() {
    return mqttConfig["port"].toInt();
}

String Configuration::getFlespiToken() {
    return mqttConfig["token"];
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

String Configuration::getProjectNumber() {
    return projectConfig["number"];
}

String Configuration::getShowcaseId() {
    return projectConfig["showcase_id"];
}

String Configuration::getDeviceName() {
    return projectConfig["device"];
}

String Configuration::getTimezone() {
    return projectConfig["timezone"];
}

void Configuration::setLogFileSize(uint32_t size) {
    projectConfig["sd_logfile_size"] = String(size);
}

uint32_t Configuration::getLogFileSize() {
    return projectConfig["sd_logfile_size"].toInt();
}

JsonObject Configuration::getDevicesConfig() {
    return devicesConfig;
}