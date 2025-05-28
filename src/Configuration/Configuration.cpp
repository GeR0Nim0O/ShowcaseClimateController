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

    // Safest approach - rebuild our known devices from the scanned addresses
    Serial.println("\nUsing direct I2C scan results for device initialization");
    
    // Known device addresses we might encounter
    const uint8_t I2C_ADDR_RTC = 0x68;       // DS3231 RTC
    const uint8_t I2C_ADDR_SHT = 0x44;       // SHT temperature/humidity sensor
    const uint8_t I2C_ADDR_BH1705 = 0x23;    // BH1705 light sensor
    const uint8_t I2C_ADDR_PCF8574 = 0x20;   // PCF8574 GPIO expander
    const uint8_t I2C_ADDR_GP8403 = 0x5F;    // GP8403 DAC

    // Create a map to find TCA ports for each I2C address
    std::map<uint8_t, uint8_t> addressToTcaPort;
    
    // Build a map of address to TCA port from scan results
    for (const auto& result : tcaScanResults) {
        uint8_t tcaPort = result.first;
        for (const auto& address : result.second) {
            if (address != 0) {  // Skip the invalid zero address
                addressToTcaPort[address] = tcaPort;
                Serial.print("Found device at address 0x");
                Serial.print(address, HEX);
                Serial.print(" on TCA port ");
                Serial.println(tcaPort);
            }
        }
    }
    
    // Now create devices based on the detected addresses
    static int deviceIndex = 0;
    Device* createdDevice = nullptr;
    
    // Check for PCF8574 GPIO expander
    if (addressToTcaPort.find(I2C_ADDR_PCF8574) != addressToTcaPort.end()) {
        uint8_t tcaPort = addressToTcaPort[I2C_ADDR_PCF8574];
        Serial.print("Creating PCF8574 device at address 0x");
        Serial.print(I2C_ADDR_PCF8574, HEX);
        Serial.print(" on TCA port ");
        Serial.println(tcaPort);
        
        std::map<String, String> channelNames;
        std::map<String, float> channelThresholds;
        
        // Set up standard GPIO channel names
        channelNames["IO0"] = "FanExterior";
        channelNames["IO1"] = "FanInterior";
        channelNames["IO2"] = "Humidify";
        channelNames["IO3"] = "Dehumidify";
        channelNames["IO4"] = "TemperatureEnable";
        channelNames["IO5"] = "TemperatureCool";
        channelNames["IO6"] = "TemperatureHeat";
        channelNames["IO7"] = "IO7";
        
        // Default thresholds
        for (const auto& pair : channelNames) {
            channelThresholds[pair.first] = 1.0f;
        }
        
        // Create the device
        createdDevice = DeviceRegistry::getInstance().createDeviceWithThresholds(
            &Wire, "GPIO", "PCF8574", I2C_ADDR_PCF8574, tcaPort, 
            channelThresholds, channelNames, deviceIndex, "OUTPUT"
        );
        
        deviceIndex++;
        
        if (createdDevice != nullptr) {
            Serial.println("PCF8574 device created successfully");
            devices.push_back(createdDevice);
        } else {
            Serial.println("Failed to create PCF8574 device");
        }
    }
    
    // Check for GP8403 DAC
    if (addressToTcaPort.find(I2C_ADDR_GP8403) != addressToTcaPort.end()) {
        uint8_t tcaPort = addressToTcaPort[I2C_ADDR_GP8403];
        Serial.print("Creating GP8403 device at address 0x");
        Serial.print(I2C_ADDR_GP8403, HEX);
        Serial.print(" on TCA port ");
        Serial.println(tcaPort);
        
        std::map<String, String> channelNames;
        std::map<String, float> channelThresholds;
        
        // Set up DAC channel names
        channelNames["DAC0"] = "TemperaturePower";
        channelThresholds["DAC0"] = 0.1f;
        
        // Create the device
        createdDevice = DeviceRegistry::getInstance().createDeviceWithThresholds(
            &Wire, "DAC", "GP8403", I2C_ADDR_GP8403, tcaPort, 
            channelThresholds, channelNames, deviceIndex, ""
        );
        
        deviceIndex++;
        
        if (createdDevice != nullptr) {
            Serial.println("GP8403 DAC device created successfully");
            devices.push_back(createdDevice);
        } else {
            Serial.println("Failed to create GP8403 DAC device");
        }
    }
    
    // Check for SHT temperature/humidity sensor
    if (addressToTcaPort.find(I2C_ADDR_SHT) != addressToTcaPort.end()) {
        uint8_t tcaPort = addressToTcaPort[I2C_ADDR_SHT];
        Serial.print("Creating SHT sensor at address 0x");
        Serial.print(I2C_ADDR_SHT, HEX);
        Serial.print(" on TCA port ");
        Serial.println(tcaPort);
        
        std::map<String, String> channelNames;
        std::map<String, float> channelThresholds;
        
        channelNames["T"] = "Temperature";
        channelNames["H"] = "Humidity";
        channelThresholds["T"] = 0.3f;
        channelThresholds["H"] = 1.0f;
        
        createdDevice = DeviceRegistry::getInstance().createDeviceWithThresholds(
            &Wire, "Sensor", "SHT", I2C_ADDR_SHT, tcaPort, 
            channelThresholds, channelNames, deviceIndex, ""
        );
        
        deviceIndex++;
        
        if (createdDevice != nullptr) {
            Serial.println("SHT sensor created successfully");
            devices.push_back(createdDevice);
        } else {
            Serial.println("Failed to create SHT sensor");
        }
    }
    
    // Check for RTC
    if (addressToTcaPort.find(I2C_ADDR_RTC) != addressToTcaPort.end()) {
        uint8_t tcaPort = addressToTcaPort[I2C_ADDR_RTC];
        Serial.print("Creating RTC at address 0x");
        Serial.print(I2C_ADDR_RTC, HEX);
        Serial.print(" on TCA port ");
        Serial.println(tcaPort);
        
        std::map<String, String> channelNames;
        std::map<String, float> channelThresholds;
        
        channelNames["Time"] = "Time";
        channelThresholds["Time"] = 0.0f;
        
        createdDevice = DeviceRegistry::getInstance().createDeviceWithThresholds(
            &Wire, "RTC", "DS3231", I2C_ADDR_RTC, tcaPort, 
            channelThresholds, channelNames, deviceIndex, ""
        );
        
        deviceIndex++;
        
        if (createdDevice != nullptr) {
            Serial.println("RTC created successfully");
            rtc = static_cast<DS3231rtc*>(createdDevice);
            Serial.println("RTC assigned to global reference");
            devices.push_back(createdDevice);
        } else {
            Serial.println("Failed to create RTC");
        }
    }
    
    // Check for BH1705 light sensor
    if (addressToTcaPort.find(I2C_ADDR_BH1705) != addressToTcaPort.end()) {
        uint8_t tcaPort = addressToTcaPort[I2C_ADDR_BH1705];
        Serial.print("Creating BH1705 sensor at address 0x");
        Serial.print(I2C_ADDR_BH1705, HEX);
        Serial.print(" on TCA port ");
        Serial.println(tcaPort);
        
        std::map<String, String> channelNames;
        std::map<String, float> channelThresholds;
        
        channelNames["L"] = "Lux";
        channelThresholds["L"] = 1.0f;
        
        createdDevice = DeviceRegistry::getInstance().createDeviceWithThresholds(
            &Wire, "Sensor", "BH1705", I2C_ADDR_BH1705, tcaPort, 
            channelThresholds, channelNames, deviceIndex, ""
        );
        
        deviceIndex++;
        
        if (createdDevice != nullptr) {
            Serial.println("BH1705 sensor created successfully");
            devices.push_back(createdDevice);
        } else {
            Serial.println("Failed to create BH1705 sensor");
        }
    }
    
    Serial.print("Created ");
    Serial.print(devices.size());
    Serial.println(" devices based on I2C scan");
    
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
        
        // First, validate the device object thoroughly
        if (!validateDeviceObject(device)) {
            Serial.println("Device validation failed, skipping initialization");
            continue;
        }
        
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