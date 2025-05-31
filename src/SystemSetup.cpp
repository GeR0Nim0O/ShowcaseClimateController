#include "SystemSetup.h"
#include "Configuration.h"
#include "I2CHandler.h"
#include "SDHandler.h"
#include "TimeHandler.h"
#include "WifiMqttHandler.h"
#include "DeviceRegistry/DeviceRegistry.h"
#include "CACert.h"
#include <map>

extern std::map<std::string, float> lastSensorValues; // From main.cpp

bool SystemSetup::initializeSystem() {
    Serial.begin(115200);
    delay(5000);
    
    Serial.println("========================================");
    Serial.println("      SYSTEM INITIALIZATION START");
    Serial.println("========================================");
    
    // Initialize I2C bus
    I2CHandler::initializeI2C();
    
    // Perform I2C scan before connecting to any devices
    I2CHandler::scanI2C();
    
    if (!setupConfiguration()) {
        Serial.println("WARNING: Configuration setup failed, continuing with defaults");
    }
    
    Serial.println("System initialization completed successfully");
    return true;
}

bool SystemSetup::setupConfiguration() {
    Serial.println("Setting up configuration...");
    
    // Initialize SD card and configuration
    if (!SDHandler::initializeSDCardAndConfig()) {
        Serial.println("WARNING: Failed to initialize SD card. Continuing with fallback configuration.");
        // Continue initialization even without SD card
    }
    
    // Load device configurations from config.json
    if (!Configuration::loadConfigFromSD("/config.json")) {
        Serial.println("Failed to load config from SD card. Using default values.");
          // Set default values
        Configuration::setWiFiSSID("Ron&Rowie_Gast");
        Configuration::setWiFiPassword("Gast@Ron&Rowie");
        Configuration::setMqttsServer("mqtt.flespi.io");
        Configuration::setMqttsPort(8883);
        Configuration::setFlespiToken("ONz40m0iGTFbiFMcp14lLnt1Eb31qnPulPkg5DkJUuGGY6OhJhN1iPqImaRT0qbp");
        Configuration::setProjectNumber("12345");
        Configuration::setShowcaseId("67");
        Configuration::setDeviceName("TEST");
        Configuration::setTimezone("UTC");
        
        return false;
    } else {
        Serial.println("Config loaded successfully from SD card.");
        
        // Load custom settings from config if enabled
        if (Configuration::isCustomWifiEnabled()) {
            Configuration::setWiFiSSID(Configuration::getCustomWifiSSID());
            Configuration::setWiFiPassword(Configuration::getCustomWifiPassword());
        }
        
        if (Configuration::isCustomMqttEnabled()) {
            Configuration::setMqttsServer(Configuration::getCustomMqttServer());
            Configuration::setMqttsPort(Configuration::getCustomMqttPort());
            Configuration::setFlespiToken(Configuration::getCustomMqttToken());
        }
    }
    
    // Print configuration
    Configuration::printConfigValues();
    return true;
}

bool SystemSetup::initializeHardware(std::vector<Device*>& devices, 
                                    std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, 
                                    DS3231rtc*& rtc) {
    Serial.println("Initializing hardware...");
    
    // Get connected I2C devices with their addresses and TCA ports
    tcaScanResults = I2CHandler::TCAScanner();
    
    // Print TCA scan results
    I2CHandler::printTCAScanResults(tcaScanResults);
    
    return setupDevices(devices, tcaScanResults, rtc);
}

bool SystemSetup::setupDevices(std::vector<Device*>& devices, 
                              std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, 
                              DS3231rtc*& rtc) {
    Serial.println("Setting up devices...");
    
    // Initialize devices based on configuration
    devices = Configuration::initializeDevices(tcaScanResults, rtc);
    
    // Initialize each device
    Configuration::initializeEachDevice(devices);
    
    // Debug: Print device count before cleanup
    Serial.print("Devices vector size before cleanup: ");
    Serial.println(devices.size());
    
    // Remove nullptr entries from devices vector
    devices.erase(std::remove(devices.begin(), devices.end(), nullptr), devices.end());
    
    // Debug: Print device count after cleanup
    Serial.print("Devices vector size after cleanup: ");
    Serial.println(devices.size());
    
    // Validate devices
    validateDevices(devices);
    
    return !devices.empty();
}

bool SystemSetup::setupClimateController(ClimateController*& climateController) {
    if (!Configuration::isClimateControllerEnabled()) {
        Serial.println("Climate controller disabled in configuration");
        return false;
    }
    
    Serial.println("Setting up climate controller...");
    climateController = ClimateController::createFromDeviceRegistry();
    
    if (climateController != nullptr) {
        // Get configuration values
        float temperatureSetpoint = Configuration::getClimateTemperatureSetpoint();
        float humiditySetpoint = Configuration::getClimateHumiditySetpoint();
        String climateMode = Configuration::getClimateMode();
        String humidityMode = Configuration::getHumidityMode();
        
        // Convert string modes to enums
        ClimateMode climateEnum = ClimateMode::AUTO;
        if (climateMode == "HEATING") {
            climateEnum = ClimateMode::HEATING;
        } else if (climateMode == "COOLING") {
            climateEnum = ClimateMode::COOLING;
        }
        
        HumidityMode humidityEnum = HumidityMode::AUTO;
        if (humidityMode == "HUMIDIFY") {
            humidityEnum = HumidityMode::HUMIDIFYING;
        } else if (humidityMode == "DEHUMIDIFY") {
            humidityEnum = HumidityMode::DEHUMIDIFYING;
        }
        
        // Configure all parameters at once
        climateController->configure(temperatureSetpoint, humiditySetpoint, climateEnum, humidityEnum);
        Serial.println("Climate controller initialized successfully");
        return true;
    } else {
        Serial.println("Failed to initialize climate controller from DeviceRegistry");
        return false;
    }
}

bool SystemSetup::setupDisplay(Display*& displayDevice) {
    Serial.println("Setting up display device...");
    
    // Get Display device from DeviceRegistry
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    displayDevice = (Display*)registry.getDeviceByType("Display", 0);
    
    if (displayDevice != nullptr && displayDevice->isInitialized()) {
        Serial.println("Display device found and initialized successfully");
        
        // Show initial startup message
        displayDevice->clear();
        displayDevice->setCursor(0, 0);
        displayDevice->print("Climate Control");
        displayDevice->setCursor(0, 1);
        displayDevice->print("Initializing...");
        delay(2000);
        return true;
    } else {
        Serial.println("Display device not found or not initialized");
        displayDevice = nullptr;
        return false;
    }
}

bool SystemSetup::setupNetworking(WiFiClientSecure& espClient, PubSubClient& client, 
                                 String& clientId, String& topic) {
    Serial.println("Setting up networking...");
    
    clientId = Configuration::getProjectNumber() + "_" + Configuration::getShowcaseId();
    topic = Configuration::getDeviceName() + "/" + Configuration::getProjectNumber() + "/" + Configuration::getShowcaseId();
    
    // Print networking information
    printSystemInfo();
    
    // Try to connect to WiFi (don't stop if it fails)
    Serial.println("Attempting WiFi connection...");
    bool wifiConnected = WifiMqttHandler::connectToWiFiWithCheck(Configuration::getWiFiSSID(), Configuration::getWiFiPassword());
    
    if (wifiConnected) {
        Serial.println("WiFi connected successfully in setup.");
        
        // Try to connect to MQTT broker only if WiFi is connected
        Serial.println("Attempting MQTT connection...");
        if (!WifiMqttHandler::connectToMqttBrokerWithCheck(client, espClient, 
            Configuration::getMqttsServer(), rootCACertificate, 
            Configuration::getMqttsPort(), clientId, topic,
            Configuration::getFlespiToken())) {
            Serial.println("Failed to connect to MQTT broker - will retry in main loop");
            return false;
        } else {
            Serial.println("MQTT connected successfully in setup.");
            return true;
        }
    } else {
        Serial.println("WiFi connection failed in setup - program will continue offline");
        Serial.println("WiFi and MQTT connections will be retried in main loop");
        return false;
    }
}

void SystemSetup::printInitialSensorData(const std::vector<Device*>& devices) {
    Serial.println("\n=== Initial Sensor Readings ===");
    
    for (size_t i = 0; i < devices.size(); i++) {
        Device* device = devices[i];
        if (device == nullptr || !device->isInitialized()) {
            continue;
        }
        
        String deviceName = device->getType() + "_" + String(device->getDeviceIndex());
        Serial.print("\n--- Initial readings for ");
        Serial.print(deviceName);
        Serial.println(" ---");
        
        I2CHandler::selectTCA(device->getTCAChannel());
        auto data = device->readData();
        
        for (const auto& channel : device->getChannels()) {
            String channelKey = channel.first;
            String deviceSpecificKey = deviceName + "_" + channelKey; // Make key device-specific
            std::string key = std::string(deviceSpecificKey.c_str());
            float value = data[channelKey].toFloat();
            
            // Store initial value in lastSensorValues to establish baseline
            lastSensorValues[key] = value;
            
            Serial.print(channelKey);
            Serial.print(": ");
            Serial.print(value);
            Serial.print(" (");
            Serial.print(channel.second);
            Serial.println(")");
        }
    }
    Serial.println("===========================\n");
}

void SystemSetup::printSystemInfo() {
    Serial.println("\n=== SYSTEM INFORMATION ===");
    Serial.print("SSID: ");
    Serial.println(Configuration::getWiFiSSID());
    Serial.print("MQTT Server: ");
    Serial.println(Configuration::getMqttsServer());
    Serial.print("MQTT Port: ");
    Serial.println(Configuration::getMqttsPort());
    Serial.print("Project Number: ");
    Serial.println(Configuration::getProjectNumber());
    Serial.print("Showcase ID: ");
    Serial.println(Configuration::getShowcaseId());
    Serial.print("Device Name: ");
    Serial.println(Configuration::getDeviceName());
    Serial.println("==========================\n");
}

void SystemSetup::validateDevices(const std::vector<Device*>& devices) {
    Serial.println("=== DEVICE VALIDATION ===");
    for (size_t i = 0; i < devices.size(); i++) {
        Device* device = devices[i];
        Serial.print("Device ");
        Serial.print(i);
        Serial.print(": ");
        if (device == nullptr) {
            Serial.println("NULL POINTER");
        } else {
            Serial.print("Type: ");
            Serial.print(device->getType());
            Serial.print(", Label: ");
            Serial.print(device->getDeviceLabel().length() > 0 ? device->getDeviceLabel() : "None");
            Serial.print(", Initialized: ");
            Serial.println(device->isInitialized() ? "Yes" : "No");
        }
    }
    Serial.println("========================\n");
}
