#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include "Device.h"
#include "BH1705sensor.h"
#include "SHTsensor.h"
#include "SCALESsensor.h"
#include "DS3231rtc.h"
#include "PCF8574gpio.h"
#include "GP8403dac.h"
#include "ClimateController.h"
#include "DeviceRegistry/DeviceRegistry.h"
#include "Display.h"  // Add Display include
#include "../lib/Config/ClimateConfig/ClimateConfig.h"  // Add ClimateConfig include

#include <WiFiClientSecure.h> 
#include <string>
#include <vector> 
#include <map> // Include map header
#include <set> // Include set header
#include <algorithm> // Include algorithm header for std::remove

#include "CACert.h"
#include "TimeHandler.h"
#include "WifiMqttHandler.h"
#include "JsonHandler.h"
#include "SDHandler.h"
#include "Configuration.h"
#include "I2CHandler.h"


WiFiClientSecure espClient; // Use WiFiClientSecure instead of WiFiClient
PubSubClient client(espClient);

// MQTT throttling - using Configuration class directly
unsigned long lastMqttSendTime = 0; // Last time data was sent via MQTT

// Global status timer settings - now loaded from config
unsigned long lastStatusUpdateTime = 0; // Last time status was printed

// Structure to track changed sensor data
struct SensorData {
    String deviceName;
    String projectNr;
    String showcaseId;
    String sensorType;
    float sensorValue;
    String currentTime;
    int deviceIndex;
    bool changed;
};

// Map to store changed sensor data
std::map<std::string, SensorData> changedSensorData;

unsigned long lastTimeFetch = 0;

bool setupComplete = false; // Flag to indicate setup completion
bool offlineMode = false; // Flag to indicate if running in offline mode

std::vector<Device*> devices; // Vector to hold device pointers
std::map<uint8_t, std::vector<uint8_t>> tcaScanResults; // Map to store TCA scan results
DS3231rtc* rtc = nullptr; // Pointer to hold RTC device - initialize to nullptr

struct StringComparator {
    bool operator()(const String& a, const String& b) const {
        return a.compareTo(b) < 0;
    }
};

std::map<std::string, float> lastSensorValues; // Changed from String to std::string

String clientId;
String topic;

int apiRetryCount = 0;
int ntpRetryCount = 0;

// Global variables for climate controller
// Climate controller instance
ClimateController* climateController = nullptr;

// Global variables for display
Display* displayDevice = nullptr;
unsigned long lastDisplayUpdate = 0;

void readAndPrintInitialSensorData(); // Add this function prototype
void readAndSendDataFromDevices();
void printDebugInfo();
void printCreatedSensors(); // Declare the function here
void logDataToSD(const String& deviceName, const String& currentTime, float value, const String& sensorType); // Update function declaration
void initializeClimateController(); // Function to initialize climate controller
void updateClimateController(); // Function to update climate controller
void showTemperatureAndHumidity(); // Function to show current temperature and humidity
void testPSRAM(); // Function to test PSRAM

// Display functions
void initializeDisplayDevice(); // Function to initialize display device
void updateDisplayWithClimateStatus(); // Function to update display with climate status

// Global status system functions
void printAllSystemStatus(); // Print all system status
void updateGlobalStatusSystem(); // Update global status system

void setup()
{
  Serial.begin(115200);
  delay(5000);
  
  // Test PSRAM first
  testPSRAM();
  
  // Initialize I2C bus
  I2CHandler::initializeI2C();  // Setup configuration
  if (!SDHandler::initializeSDCardAndConfig()) {
    Serial.println("WARNING: Failed to initialize SD card. Continuing with fallback configuration.");
  }
  
  // Try to load configuration from SD card first, then fallback to project config.json
  if (!Configuration::loadConfigFromSD("/config.json")) {
    Serial.println("SD card config failed, trying project config.json fallback...");
    if (!Configuration::loadConfigFromCodebase()) {
      Serial.println("ERROR: Both SD card and project config.json failed!");
      Serial.println("System cannot continue without configuration.");
      while(1) { delay(1000); } // Halt system
    }
  } else {
    // SD card config loaded successfully - check if it differs from project config
    String sdWifiSSID = Configuration::getWiFiSSID();
    
    // Temporarily load project config to compare
    JsonDocument tempProjectConfig;
    if (Configuration::loadJsonFromCodebase(tempProjectConfig)) {
      String projectWifiSSID = tempProjectConfig["wifi"]["ssid"].as<String>();
      
      if (sdWifiSSID != projectWifiSSID) {
        Serial.println();
        Serial.println("=== CONFIG MISMATCH DETECTED ===");
        Serial.print("SD Card WiFi SSID: '");
        Serial.print(sdWifiSSID);
        Serial.println("'");
        Serial.print("Project WiFi SSID: '");
        Serial.print(projectWifiSSID);
        Serial.println("'");
        Serial.println();
        Serial.println("The SD card configuration differs from your project config.json");
        Serial.println("Would you like to update the SD card with the project configuration?");
        Serial.println();
        Serial.println("Send 'Y' or 'y' within 15 seconds to update SD card");
        Serial.println("Send any other key or wait to keep current SD card config");
        Serial.print("Waiting for input: ");
        
        // Wait for user input for 15 seconds
        unsigned long startWait = millis();
        String userInput = "";
        
        while (millis() - startWait < 15000) { // 15 second timeout
          if (Serial.available()) {
            userInput = Serial.readString();
            userInput.trim();
            Serial.println(userInput);
            break;
          }
          delay(100);
        }
        
        if (userInput.length() == 0) {
          Serial.println("(timeout)");
          Serial.println("→ Keeping current SD card configuration");
        } else if (userInput.equalsIgnoreCase("y")) {
          Serial.println("→ Updating SD card with project configuration...");
          if (SDHandler::forceUpdateSDConfig()) {
            Serial.println("✓ SD card updated successfully!");
            Serial.println("→ Restarting ESP32 to use new configuration...");
            delay(2000);
            ESP.restart();
          } else {
            Serial.println("✗ Failed to update SD card");
            Serial.println("→ Continuing with current SD card configuration");
          }
        } else {
          Serial.print("→ User declined ('");
          Serial.print(userInput);
          Serial.println("') - keeping SD card configuration");
        }
        Serial.println("================================");
        Serial.println();
      }
    }
  }
  Configuration::printConfigValues();

  // Get connected I2C devices with their addresses and TCA ports
  tcaScanResults = I2CHandler::TCAScanner();

  // Print TCA scan results
  I2CHandler::printTCAScanResults(tcaScanResults);
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
  
  // Print created sensors for debugging (moved after initialization)
  printCreatedSensors();
    // Initialize climate controller
  initializeClimateController();
  
  
  // Initialize display device
  initializeDisplayDevice();
  
  // NEW: Read and print initial sensor values after initialization
  Serial.println("\n=== Initial Sensor Readings ===");
  readAndPrintInitialSensorData();
  // Additional debug: Check each device individually
  Serial.println("Device validation:");
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
      Serial.print(", Initialized: ");
      Serial.println(device->isInitialized() ? "Yes" : "No");
    }
  }
  Serial.println("I2C scan done");

  clientId = Configuration::getProjectNumber() + "_" + Configuration::getShowcaseId();
  topic = Configuration::getDeviceName() + "/" + Configuration::getProjectNumber() + "/" + Configuration::getShowcaseId();
  // Print debugging information
  printDebugInfo();
  // Try to connect to WiFi (don't stop if it fails)
  Serial.println("Attempting WiFi connection...");
  bool wifiConnected = WifiMqttHandler::connectToWiFiWithCheck(Configuration::getWiFiSSID(), Configuration::getWiFiPassword());
    if (!wifiConnected) {
    Serial.println();
    Serial.println("==== WiFi Connection Failed - Troubleshooting Help ====");
    Serial.println("WiFi connection failed. Here are some troubleshooting steps:");
    Serial.println();
    Serial.println("1. DETAILED NETWORK ANALYSIS:");
    WifiMqttHandler::scanAndAnalyzeNetworks();
    Serial.println();
    Serial.println("2. CONFIGURATION CHECK:");
    Serial.print("   Current SSID: '");
    Serial.print(Configuration::getWiFiSSID());
    Serial.println("'");
    Serial.print("   Password length: ");
    Serial.println(Configuration::getWiFiPassword().length());
    Serial.println();
    Serial.println("3. COMMON SOLUTIONS:");
    Serial.println("   - Verify network name is exactly correct (case-sensitive)");
    Serial.println("   - Check WiFi password");
    Serial.println("   - Ensure router is powered on and broadcasting");
    Serial.println("   - Move ESP32 closer to router");
    Serial.println("   - Restart router if needed");
    Serial.println("   - Check for MAC address filtering on router");
    Serial.println();    Serial.println("4. TO UPDATE WIFI SETTINGS:");
    Serial.println("   - Edit data/config.json file");
    Serial.println("   - Upload to ESP32 via PlatformIO");
    Serial.println("   - Or use SD card with updated config.json");
    Serial.println();    Serial.println("5. SD CARD CONFIG UPDATE OPTION:");
    Serial.println("   The SD card contains different WiFi settings than the project config.json");
    Serial.print("   SD Card WiFi: '");
    Serial.print(Configuration::getWiFiSSID());
    Serial.println("'");
    Serial.println("   Project config.json WiFi: 'Ron-Rowie' (stronger signal)");
    Serial.println();
    Serial.println("   Would you like to update the SD card with project config.json settings?");
    Serial.println("   This will overwrite the SD card config.json with data/config.json");
    Serial.println();
    Serial.println("   Send 'Y' or 'y' within 10 seconds to update SD card config");
    Serial.println("   Send any other key or wait 10 seconds to keep current SD card config");
    Serial.print("   Waiting for input: ");
    
    // Wait for user input for 10 seconds
    unsigned long startWait = millis();
    bool updateRequested = false;
    String userInput = "";
    
    while (millis() - startWait < 10000) { // 10 second timeout
        if (Serial.available()) {
            userInput = Serial.readString();
            userInput.trim();
            Serial.println(userInput);
            break;
        }
        delay(100);
    }
    
    if (userInput.length() == 0) {
        Serial.println("(timeout)");
        Serial.println("   → No input received - keeping current SD card configuration");
    } else if (userInput.equalsIgnoreCase("y")) {
        Serial.println("   → User confirmed - updating SD card configuration...");
        if (SDHandler::forceUpdateSDConfig()) {
            Serial.println("   ✓ SD card config updated successfully!");
            Serial.println("   → Please restart the ESP32 to use the updated configuration");
            Serial.println("   → The system will now use 'Ron-Rowie' network instead of 'Ron&Rowie_Gast'");
            Serial.println("   → Restart recommended: Press EN button or power cycle");
        } else {
            Serial.println("   ✗ Failed to update SD card configuration");
            Serial.println("   → Please manually update the SD card config.json file");
        }
    } else {
        Serial.print("   → User declined ('");
        Serial.print(userInput);
        Serial.println("') - keeping current SD card configuration");
    }
    Serial.println();
    Serial.println("Continuing in offline mode...");
    Serial.println("======================================================");
    Serial.println();
  }
  
  if (wifiConnected) {
    Serial.println("WiFi connected successfully in setup.");
    offlineMode = false;
    
    // Connect to TimeAPI and NTP only if WiFi is connected
    // Only fetch time from RTC if RTC is connected and initialized
    if (rtc && rtc->isInitialized()) {
      TimeHandler::fetchTime(*rtc);
    } else {
      Serial.println("RTC not connected or not initialized. Skipping RTC time fetch.");
    }

    // Try to connect to MQTT broker only if WiFi is connected
    Serial.println("Attempting MQTT connection...");
    if (!WifiMqttHandler::connectToMqttBrokerWithCheck(client, espClient, 
        Configuration::getMqttsServer(), rootCACertificate, 
        Configuration::getMqttsPort(), clientId, topic,
        Configuration::getFlespiToken())) {
      Serial.println("Failed to connect to MQTT broker - will retry in main loop");
    } else {
      Serial.println("MQTT connected successfully in setup.");
    }
  } else {
    Serial.println("WiFi connection failed in setup - program will continue offline");
    Serial.println("WiFi and MQTT connections will be retried in main loop");
    offlineMode = true;
  }
  delay(500);
  setupComplete = true; // Indicate that setup is complete
  Serial.println("Setup complete: " + String(setupComplete));
  // Configure MQTT throttling from configuration
  Serial.print("MQTT throttling configuration loaded: ");
  Serial.print(Configuration::isMqttThrottlingEnabled() ? "enabled" : "disabled");
  if (Configuration::isMqttThrottlingEnabled()) {
    Serial.print(", interval: ");
    Serial.print(Configuration::getMqttThrottlingInterval() / 1000);
    Serial.println(" seconds");
  } else {
    Serial.println();
  }
}

void loop() {
  if (!setupComplete) {
    return; // Exit loop if setup is not complete
  }

  // Use existing WifiMqttHandler::keepAlive for connection management
  WifiMqttHandler::keepAlive(client, espClient, 
                            Configuration::getWiFiSSID().c_str(), 
                            Configuration::getWiFiPassword().c_str(),
                            Configuration::getMqttsServer().c_str(), 
                            rootCACertificate, 
                            Configuration::getMqttsPort(), 
                            clientId.c_str(), 
                            topic.c_str());

  // Update offline mode status
  offlineMode = (WiFi.status() != WL_CONNECTED);

  // Periodic time synchronization
  TimeHandler::fetchCurrentTimePeriodically(rtc, lastTimeFetch, Configuration::getTimeFetchInterval());

  // Update global status system (aligned with MQTT timer)
  updateGlobalStatusSystem();
  
  // Read and send data from each device
  readAndSendDataFromDevices();
  
  // Update climate controller
  if (Configuration::isClimateControllerEnabled() && climateController != nullptr) {
    updateClimateController();
  }
    // Update display with climate status periodically
  unsigned long displayUpdateInterval = Configuration::getDisplayUpdateInterval();
  if (millis() - lastDisplayUpdate >= displayUpdateInterval) {
    updateDisplayWithClimateStatus();
    lastDisplayUpdate = millis();
  }
}

// New function to read and print initial sensor values
void readAndPrintInitialSensorData() {
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

void sendSensorDataOverMQTT(const SensorData& data) {
    if (WiFi.status() != WL_CONNECTED || !client.connected()) {
        return;
    }
    
    JsonHandler::sendJsonOverMqtt(
        client, 
        data.deviceName.c_str(), 
        data.projectNr.c_str(), 
        data.showcaseId.c_str(), 
        data.sensorType.c_str(), 
        data.sensorValue, 
        data.currentTime.c_str(), 
        data.deviceIndex
    );
    
    Serial.print("Sent to MQTT: ");
    Serial.print(data.deviceName);
    Serial.print(" ");
    Serial.print(data.sensorType);
    Serial.print(": ");
    Serial.println(data.sensorValue);
}

void readAndSendDataFromDevices() {
    // Flag to determine if we should print data this cycle (only true every 60 seconds)
    bool shouldPrintData = Configuration::isMqttThrottlingEnabled() && (millis() - lastMqttSendTime >= Configuration::getMqttThrottlingInterval());
      // Track timing for MQTT throttling
    unsigned long currentTime = millis();
    unsigned long timeSinceLastSend = currentTime - lastMqttSendTime;
    
    if (shouldPrintData) {
        Serial.println("\n=== Sensor Readings (60-second update) ===");
    }
    
    // Track if any threshold was exceeded for climate status printing
    bool anyThresholdExceeded = false;
    
    for (size_t i = 0; i < devices.size(); i++) {
        Device* device = devices[i];
        if (device == nullptr) {
            Serial.print("Error: Null device pointer at index ");
            Serial.println(i);
            continue;
        }
        if (!device->isInitialized()) {
            Serial.print("Error: Uninitialized device at index ");
            Serial.print(i);
            Serial.print(", Type: ");
            Serial.print(device->getType());
            Serial.print(", Address: 0x");
            Serial.println(device->getI2CAddress(), HEX);
            delay(100);
            continue;
        }
        
        I2CHandler::selectTCA(device->getTCAChannel());
        auto data = device->readData();        
        
        for (const auto& channel : device->getChannels()) {
            String channelKey = channel.first;            String deviceName = device->getType() + "_" + String(device->getDeviceIndex());
            String deviceLabel = device->getDeviceLabel();
            String displayName = deviceName;
            if (deviceLabel.length() > 0) {
                displayName += " (" + deviceLabel + ")";
            }
            String deviceSpecificKey = deviceName + "_" + channelKey; // Make key device-specific
            std::string key = std::string(deviceSpecificKey.c_str()); // Convert to std::string
            float value = data[channelKey].toFloat(); // Use String key and convert to float
            String currentTime;
            // Only fetch time from RTC if RTC is connected and initialized
            if (rtc && rtc->isInitialized()) {
                currentTime = TimeHandler::getCurrentTime(*rtc);
            } else {
                currentTime = "";
            }
            String projectNr = Configuration::getProjectNumber();
            String showcaseId = Configuration::getShowcaseId();
            
            // Get the last value and threshold for this sensor channel
            float lastValue = lastSensorValues[key];            float threshold = device->getThreshold(channelKey);
            
            // Always print data every 60 seconds, aligned with MQTT sending, even if no change
            if (shouldPrintData) {
                Serial.print(deviceName);
                Serial.print(" - ");
                Serial.print(channelKey);
                Serial.print(": ");
                Serial.print(value);
                Serial.print(" (");
                Serial.print(channel.second);
                Serial.println(")");
            }
            
            // This conditional is used for logging to SD and determining meaningful changes
            float valueDiff = abs(value - lastValue);
            bool shouldLog = valueDiff >= threshold;
            bool shouldSendMqtt = shouldPrintData || shouldLog; // Send via MQTT if 60-second cycle OR threshold exceeded
            
            // Track if any threshold was exceeded
            if (shouldLog) {
                anyThresholdExceeded = true;
            }
              // Store data for MQTT sending if it's either the 60-second cycle OR threshold exceeded
            if (shouldSendMqtt) {
                SensorData sensorData;
                sensorData.deviceName = displayName;  // Use display name with label
                sensorData.projectNr = projectNr;
                sensorData.showcaseId = showcaseId;
                sensorData.sensorType = channel.second;
                sensorData.sensorValue = value;
                sensorData.currentTime = currentTime;
                sensorData.deviceIndex = device->getDeviceIndex();
                sensorData.changed = true;
                
                changedSensorData[key] = sensorData;
                  if (shouldLog) {
                    Serial.print("THRESHOLD EXCEEDED - ");
                    Serial.print(displayName);
                    Serial.print(" ");
                    Serial.print(channelKey);
                    Serial.print(": ");
                    Serial.print(value);
                    Serial.print(" (diff: ");
                    Serial.print(valueDiff);
                    Serial.print(" >= ");
                    Serial.print(threshold);
                    Serial.println(")");
                    
                    // Send data to MQTT immediately when threshold exceeded
                    if (WiFi.status() == WL_CONNECTED && client.connected()) {
                        sendSensorDataOverMQTT(sensorData);
                    }
                }
            }
            
            // Log to SD only when threshold is exceeded
            if (shouldLog) {
                lastSensorValues[key] = value;                if (channel.second != "Time") {
                    // Log to SD only when the value has changed beyond threshold
                    logDataToSD(displayName, currentTime, value, channel.second);
                }
            }
            // No else clause needed since we always update changedSensorData on the 60-second cycle        }
    }
      // Check if it's time to send all changed data via MQTT
    if (shouldPrintData) {
        if (WiFi.status() == WL_CONNECTED && client.connected()) {
            Serial.println("\nSensor data collected - sending to MQTT...");
            // Send all collected sensor data for this cycle
            for (auto& item : changedSensorData) {
                SensorData& data = item.second;
                if (data.changed) {
                    sendSensorDataOverMQTT(data);
                    data.changed = false; // Reset changed flag
                }
            }
            lastMqttSendTime = millis(); // Update last send time
            Serial.println("Sent all changed sensor data via MQTT");
        } else if (WiFi.status() != WL_CONNECTED) {
            Serial.println("\nSensor data collected - WiFi not connected, data logged to SD only");
        } else if (!client.connected()) {
            Serial.println("\nSensor data collected - MQTT not connected, data logged to SD only");
        }
    }
    }
}

// Global status system functions
void printAllSystemStatus() {
    Serial.println("\n========================================");
    Serial.println("         SYSTEM STATUS UPDATE");
    Serial.println("========================================");
    
    // Print all status modules
    TimeHandler::printTimeStatus(rtc);
    WifiMqttHandler::printConnectionStatus(client);
    SDHandler::printSDCardStatus();
    I2CHandler::printI2CBusStatus(tcaScanResults);
    
    if (climateController != nullptr) {
        climateController->printClimateStatus();
    }
    
    Serial.println("========================================");
    Serial.println("       END STATUS UPDATE");
    Serial.println("========================================\n");
}

void updateGlobalStatusSystem() {
    // Check if it's time for a status update (aligned with MQTT timer)
    bool timeForStatusUpdate = (millis() - lastStatusUpdateTime >= Configuration::getStatusUpdateInterval());
    
    if (timeForStatusUpdate) {
        printAllSystemStatus();
        lastStatusUpdateTime = millis();
    }
}

// Update function to include sensor type parameter
void logDataToSD(const String& deviceName, const String& currentTime, float value, const String& sensorType) {
    String json = "{\"device\":\"" + deviceName + "\",\"type\":\"" + sensorType + "\",\"timestamp\":\"" + currentTime + "\",\"value\":" + String(value) + "}";
    SDHandler::logJson(json.c_str());
}

void printDebugInfo() {
  Serial.print("SSID: ");
  Serial.println(Configuration::getWiFiSSID());
  Serial.print("Password: ");
  Serial.println(Configuration::getWiFiPassword());
  Serial.print("MQTT Server: ");
  Serial.println(Configuration::getMqttsServer());
  Serial.print("MQTT Port: ");
  Serial.println(Configuration::getMqttsPort());
  Serial.print("Client ID: ");
  Serial.println(clientId);
  Serial.print("Topic: ");
  Serial.println(topic);
}

void createSensors() {
  // Declare and initialize devicesConfig
  JsonObject devicesConfig = Configuration::getDevicesConfig();
}

void printCreatedSensors() {
  Serial.println("Created sensors:");
  for (Device* device : devices) {
    Serial.print("Type: ");
    Serial.println(device->getType());
    Serial.print("Device Name: ");
    Serial.println(device->getDeviceName());
    Serial.print("Device Label: ");
    Serial.println(device->getDeviceLabel().length() > 0 ? device->getDeviceLabel() : "No Label");
    Serial.print("Address: 0x");
    Serial.println(device->getI2CAddress(), HEX);
    Serial.print("TCA Channel: ");
    Serial.println(device->getTCAChannel());
    Serial.print("Device Index: ");
    Serial.println(device->getDeviceIndex());
    Serial.print("Initialized: ");
    Serial.println(device->isInitialized() ? "Yes" : "No");
    Serial.println();
  }
}

// Function to initialize the climate controller
void initializeClimateController() {
    if (Configuration::isClimateControllerEnabled()) {
        Serial.println("Initializing Climate Controller...");
        climateController = ClimateController::createFromDeviceRegistry();
        
        if (climateController != nullptr) {
            // Initialize ClimateConfig system
            ClimateConfig& climateConfig = ClimateConfig::getInstance();
            if (!climateConfig.begin()) {
                Serial.println("Failed to initialize ClimateConfig EEPROM");
                return;
            }
            
            // Try to load from ClimateConfig.json first
            bool configLoadedFromFile = false;
            
            // First try SD card
            if (climateConfig.loadFromJsonFile("/ClimateConfig.json")) {
                Serial.println("Climate configuration loaded from SD card ClimateConfig.json");
                configLoadedFromFile = true;
            }
            // Then try SPIFFS if SD failed
            else if (climateConfig.loadFromJsonFile("/data/ClimateConfig.json")) {
                Serial.println("Climate configuration loaded from SPIFFS ClimateConfig.json");
                configLoadedFromFile = true;
            }
            
            // If ClimateConfig.json doesn't exist or is incomplete, fall back to main config.json
            if (!configLoadedFromFile) {
                Serial.println("ClimateConfig.json not found or incomplete, using values from main config.json");
                
                // Get values from main Configuration class
                float temperatureSetpoint = Configuration::getClimateTemperatureSetpoint();
                float humiditySetpoint = Configuration::getClimateHumiditySetpoint();
                String climateMode = Configuration::getClimateMode();
                String humidityMode = Configuration::getHumidityMode();
                
                // Apply these values to ClimateConfig
                climateConfig.setTemperatureSetpoint(temperatureSetpoint);
                climateConfig.setHumiditySetpoint(humiditySetpoint);
                climateConfig.setClimateMode(climateMode);
                climateConfig.setHumidityMode(humidityMode);
                
                // Save the configuration to ClimateConfig.json for future use
                if (climateConfig.createDefaultJsonFile("/data/ClimateConfig.json")) {
                    Serial.println("Created default ClimateConfig.json from main config values");
                } else {
                    Serial.println("Warning: Could not create ClimateConfig.json file");
                }
                
                // Also save to EEPROM
                climateConfig.saveSettings();
            }
            
            // Get final configuration values from ClimateConfig
            float temperatureSetpoint = climateConfig.getTemperatureSetpoint();
            float humiditySetpoint = climateConfig.getHumiditySetpoint();
            String climateMode = climateConfig.getClimateMode();
            String humidityMode = climateConfig.getHumidityMode();
            
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
            
            // Print loaded configuration
            Serial.println("Climate Controller configured with:");
            Serial.print("  Temperature Setpoint: ");
            Serial.print(temperatureSetpoint);
            Serial.println("°C");
            Serial.print("  Humidity Setpoint: ");
            Serial.print(humiditySetpoint);
            Serial.println("%");
            Serial.print("  Climate Mode: ");
            Serial.println(climateMode);
            Serial.print("  Humidity Mode: ");
            Serial.println(humidityMode);
            
        } else {
            Serial.println("Failed to initialize climate controller from DeviceRegistry");
        }
    } else {
        Serial.println("Climate controller disabled in configuration");
    }
}

// Function to update the climate controller (called every loop)
void updateClimateController() {
    ClimateController::updateControllerWithTiming(climateController);
}

// Function to show current temperature and humidity
void showTemperatureAndHumidity() {
    if (climateController != nullptr) {
        float currentTemperature = climateController->getCurrentTemperature();
        float currentHumidity = climateController->getCurrentHumidity();
        
        Serial.print("Current Temperature: ");
        Serial.print(currentTemperature);
        Serial.println(" °C");
        Serial.print("Current Humidity: ");
        Serial.print(currentHumidity);
        Serial.println(" %");
    } else {
        Serial.println("Climate controller not initialized");
    }
}

// Function to initialize the display device
void initializeDisplayDevice() {
    Serial.println("Initializing Display Device...");
    
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
    } else {
        Serial.println("Display device not found or not initialized");
        displayDevice = nullptr;
    }
}

// Function to update the display with current climate status
void updateDisplayWithClimateStatus() {
    if (displayDevice == nullptr || !displayDevice->isInitialized()) {
        return; // No display available
    }
    
    if (climateController == nullptr) {
        // Show error message if climate controller is not available
        displayDevice->displayError("No Climate Ctrl");
        return;
    }
    
    // Get current climate data
    float currentTemp = climateController->getCurrentTemperature();
    float currentHum = climateController->getCurrentHumidity();
    float tempSetpoint = climateController->getTemperatureSetpoint();
    float humSetpoint = climateController->getHumiditySetpoint();
    
    // Update display with climate status
    displayDevice->displayClimateStatus(currentTemp, currentHum, tempSetpoint, humSetpoint);
}

// Function to test PSRAM
void testPSRAM() {
    Serial.println("\n=== PSRAM Test ===");
    
    // Check if PSRAM is available
    if (psramFound()) {
        Serial.println("✅ PSRAM detected!");
        
        // Get PSRAM size
        size_t psramSize = ESP.getPsramSize();
        size_t freePsram = ESP.getFreePsram();
        
        Serial.print("PSRAM Total Size: ");
        Serial.print(psramSize / 1024 / 1024);
        Serial.println(" MB");
        
        Serial.print("PSRAM Free Size: ");
        Serial.print(freePsram / 1024 / 1024);
        Serial.println(" MB");
        
        // Test PSRAM allocation
        void* testPtr = ps_malloc(1024 * 1024); // Allocate 1MB in PSRAM
        if (testPtr != nullptr) {
            Serial.println("✅ PSRAM allocation test successful");
            free(testPtr);
        } else {
            Serial.println("❌ PSRAM allocation test failed");
        }
    } else {
        Serial.println("❌ PSRAM not detected!");
        Serial.println("Check hardware connections and configuration");
    }
    Serial.println("==================\n");
}


