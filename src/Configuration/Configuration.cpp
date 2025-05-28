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

// Initialize static member variables
std::map<String, String> Configuration::wifiConfig;
std::map<String, String> Configuration::mqttConfig;
std::map<String, String> Configuration::projectConfig;
JsonObject Configuration::devicesConfig = JsonObject();

bool Configuration::loadConfigFromSD(const char* filename) {
    // Parse the JSON file from the SD card
    JsonDocument doc();
    
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
    if (config.containsKey("wifi")) {
        parseWiFiConfig(config["wifi"]);
    }
    
    // Parse MQTT configuration
    if (config.containsKey("mqtt")) {
        parseMQTTConfig(config["mqtt"]);
    }
    
    // Parse MQTTS configuration (SSL)
    if (config.containsKey("mqtts")) {
        parseMQTTConfig(config["mqtts"]);
    }
    
    // Parse project configuration
    if (config.containsKey("project")) {
        parseProjectConfig(config["project"]);
    }
    
    // Store devices configuration for later use
    if (config.containsKey("Devices")) {
        devicesConfig = config["Devices"].as<JsonObject>();
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
        static int deviceIndex = 0;        Device* createdDevice = DeviceRegistry::getInstance().createDeviceWithThresholds(
            &Wire,
            deviceType,
            deviceTypeNumber,
            deviceAddress,
            tcaPort,
            channelThresholds,
            channelNames,
            deviceIndex,
            deviceMode
        );
          deviceIndex++;
        
        // Check available heap memory after device creation attempt
        Serial.print("Free heap after device creation attempt: ");
        Serial.println(ESP.getFreeHeap());
        
        if (createdDevice != nullptr) {
            // Additional safety checks on the created device before adding to vector
            Serial.println("Validating created device...");
            
            // Check if device pointer is in valid memory range
            if ((uint32_t)createdDevice < 0x3F800000 || (uint32_t)createdDevice > 0x3FFFFFFF) {
                Serial.println("ERROR: Created device pointer is outside valid ESP32 memory range");
                delete createdDevice;
                continue;
            }
            
            // Test basic device access without virtual method calls
            Serial.print("Device created at address: 0x");
            Serial.println((uint32_t)createdDevice, HEX);
            
            // Add a small delay to let system stabilize
            delay(50);
            
            // Try to safely access a basic property
            bool deviceValid = true;
            try {
                // Minimal test - just access the object without virtual calls yet
                volatile uint8_t* testPtr = (volatile uint8_t*)createdDevice;
                volatile uint8_t testValue = *testPtr; // Try to read first byte
                (void)testValue; // Prevent unused variable warning
                delay(10);
            } catch (...) {
                Serial.println("ERROR: Device memory test failed");
                deviceValid = false;
            }
            
            if (deviceValid) {
                devices.push_back(createdDevice);
                Serial.print("Device successfully added to vector. Vector size: ");
                Serial.println(devices.size());
                
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
                Serial.println("Device failed validation, deleting...");
                delete createdDevice;
            }
        } else {
            Serial.print("Failed to create device: ");
            Serial.println(deviceKey);
        }
    }    Serial.println("Device initialization from config complete");
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
    
    // Try to read the vtable pointer (first 4 bytes of object)
    volatile uint32_t* vtablePtr = (volatile uint32_t*)device;
    volatile uint32_t vtableAddr = 0;
    
    try {
        vtableAddr = *vtablePtr;
        
        // Check if vtable address is reasonable
        if (vtableAddr < 0x400D0000 || vtableAddr > 0x40400000) {
            Serial.print("Vtable address 0x");
            Serial.print(vtableAddr, HEX);
            Serial.println(" appears invalid");
            return false;
        }
        
        Serial.print("Device vtable at: 0x");
        Serial.println(vtableAddr, HEX);
        
    } catch (...) {
        Serial.println("Exception while reading vtable pointer");
        return false;
    }
    
    return true;
}

void Configuration::initializeEachDevice(std::vector<Device*>& devices) {
    Serial.println("\n=== Starting Device Initialization ===");
    
    for (size_t i = 0; i < devices.size(); i++) {
        Device* device = devices[i];
        if (!device) {
            Serial.print("Device at index ");
            Serial.print(i);
            Serial.println(" is NULL, skipping...");
            continue;
        }
        
        Serial.print("\n--- Initializing Device ");
        Serial.print(i);
        Serial.println(" ---");
        Serial.print("Device Address: 0x");
        Serial.println((uint32_t)device, HEX);
        
        // Add memory integrity checks before calling virtual methods
        Serial.println("Performing memory integrity checks...");
        
        // Check if device pointer is in valid memory range
        if ((uint32_t)device < 0x3F800000 || (uint32_t)device > 0x3FFFFFFF) {
            Serial.println("ERROR: Device pointer is outside valid ESP32 memory range");
            continue;
        }
        
        // Add a small delay and yield to prevent watchdog issues
        delay(10);
        yield();
        
        // Try to safely access device properties with error handling
        String deviceType = "UNKNOWN";
        uint8_t deviceAddress = 0;
        uint8_t tcaChannel = 0;
        
        Serial.println("Attempting to read device type...");
        try {
            deviceType = device->getType();
            Serial.print("Type: ");
            Serial.println(deviceType);
        } catch (...) {
            Serial.println("ERROR: Exception while reading device type");
            continue;
        }
        
        Serial.println("Attempting to read device address...");
        try {
            deviceAddress = device->getI2CAddress();
            Serial.print("Address: 0x");
            Serial.println(deviceAddress, HEX);
        } catch (...) {
            Serial.println("ERROR: Exception while reading device address");
            continue;
        }
        
        Serial.println("Attempting to read TCA channel...");
        try {
            tcaChannel = device->getTCAChannel();
            Serial.print("TCA Channel: ");
            Serial.println(tcaChannel);
        } catch (...) {
            Serial.println("ERROR: Exception while reading TCA channel");
            continue;
        }        
        // Add safety delay
        delay(100);
        
        Serial.println("Selecting TCA channel " + String(tcaChannel) + "...");
        
        // Safe TCA selection with error handling
        try {
            I2CHandler::selectTCA(tcaChannel);
            delay(50); // Give TCA time to switch
        } catch (...) {
            Serial.println("ERROR: Exception during TCA selection");
            continue;
        }
        
        // Test I2C connectivity
        Serial.println("Testing I2C connectivity...");
        Wire.beginTransmission(deviceAddress);
        int error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.println("I2C connection test PASSED");
        } else {
            Serial.print("I2C connection test FAILED with error code ");
            Serial.println(error);
            continue; // Skip device initialization if I2C test fails
        }
        
        Serial.println("Attempting device initialization...");
        Serial.print("DEBUG: Device address being initialized: 0x");
        Serial.println((uint32_t)device, HEX);
        
        // Add extra safety checks before calling begin()
        if (deviceAddress == 0x27 && tcaChannel == 4) {
            Serial.println("SPECIAL HANDLING: This is the problematic device (0x27 on TCA Port 4)");
            Serial.println("Adding extra delays and checks...");
            delay(200);
        }
        
        bool success = false;
        try {
            // Extra safety delay before begin()
            delay(50);
            
            // Check if device is still valid before calling begin()
            if ((uint32_t)device < 0x3F800000 || (uint32_t)device > 0x3FFFFFFF) {
                Serial.println("ERROR: Device pointer became invalid before begin()");
                continue;
            }
            
            success = device->begin();
            delay(50); // Safety delay after begin()
        } catch (...) {
            Serial.println("ERROR: Exception caught during device->begin()");
            success = false;
        }
        
        Serial.print("DEBUG: device->begin() returned: ");
        Serial.println(success);
        
        Serial.print("DEBUG: Device address after begin(): 0x");
        Serial.println((uint32_t)device, HEX);
        
        // Safely check initialization status
        bool isDeviceInitialized = false;
        try {
            isDeviceInitialized = device->isInitialized();
        } catch (...) {
            Serial.println("ERROR: Exception while checking initialization status");
            isDeviceInitialized = false;
        }
        
        Serial.print("DEBUG: device->isInitialized() after begin(): ");
        Serial.println(isDeviceInitialized);
        
        if (success) {
            Serial.println("Device initialization: SUCCESS");
        } else {
            Serial.println("Device initialization: FAILED");
        }
        
        Serial.print("Final device state - Initialized: ");
        Serial.println(isDeviceInitialized ? "YES" : "NO");
        
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