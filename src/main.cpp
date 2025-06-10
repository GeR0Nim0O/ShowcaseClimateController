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

// Sensor reading throttling to prevent SHT self-heating (10% duty cycle)
unsigned long lastSensorReadTime = 0; // Last time sensors were actually read
const unsigned long SENSOR_READ_INTERVAL = 10000; // Read sensors every 10 seconds (10% of 100s cycle)

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

// AutoTune mode selection enum
enum class AutoTuneMode {
    SKIP,
    NORMAL,
    FAST
};

// Function declarations
void readAndPrintInitialSensorData(); // Add this function prototype
void readAndSendDataFromDevices();
void printDebugInfo();
void printCreatedSensors(); // Declare the function here
void logDataToSD(const String& deviceName, const String& currentTime, float value, const String& sensorType); // Update function declaration
void initializeClimateController(); // Function to initialize climate controller
void updateClimateController(); // Function to update climate controller
void showTemperatureAndHumidity(); // Function to show current temperature and humidity
void testPSRAM(); // Function to test PSRAM

// AutoTune functions
AutoTuneMode promptForAutoTune(); // Function to prompt user for AutoTune mode selection
void handleSerialCommands(); // Function to handle serial commands for AutoTune control

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
  Serial.println("=== Showcase Climate Controller ===");
  
  // Test PSRAM first
  testPSRAM();
  
  // Initialize SD card and handle configuration early
  if (!SDHandler::initializeSDCardAndConfig()) {
    Serial.println("WARNING: Failed to initialize SD card. Continuing with fallback configuration.");
  }
  
  // Handle configuration loading with priority for project config
  bool configLoaded = false;
  String sdWifiSSID = "";
  String projectWifiSSID = "";
  
  // First, always try to load the project configuration from SPIFFS (uploaded data folder)
  if (Configuration::loadConfigFromCodebase()) {
    projectWifiSSID = Configuration::getWiFiSSID();
    configLoaded = true;
    
    // Now check if SD card has different config
    JsonDocument sdConfig;
    File sdFile = SD.open("/config.json", FILE_READ);
    if (sdFile) {
      String sdConfigContent = sdFile.readString();
      sdFile.close();
      
      DeserializationError error = deserializeJson(sdConfig, sdConfigContent);
      if (!error && !sdConfig["wifi"]["ssid"].isNull()) {
        sdWifiSSID = sdConfig["wifi"]["ssid"].as<String>();
        
        if (sdWifiSSID != projectWifiSSID) {
          Serial.println("Config mismatch detected - updating SD card");
          if (SDHandler::forceUpdateSDConfig()) {
            Serial.println("SD card updated successfully");
          } else {
            Serial.println("SD update failed - continuing with project config");
          }
        }
      }
    }
  } else {
    // Fallback to SD card config if project config fails
    if (Configuration::loadConfigFromSD("/config.json")) {
      Serial.println("SD card configuration loaded");
      configLoaded = true;
    }
  }
  
  if (!configLoaded) {
    Serial.println("ERROR: Configuration loading failed!");
    while(1) { delay(1000); }
  }  
  
  // Initialize I2C and devices
  I2CHandler::initializeI2C();
  tcaScanResults = I2CHandler::TCAScanner();
  I2CHandler::printTCAScanResults(tcaScanResults);
  
  devices = Configuration::initializeDevices(tcaScanResults, rtc);
  Configuration::initializeEachDevice(devices);
  devices.erase(std::remove(devices.begin(), devices.end(), nullptr), devices.end());
  
  Serial.print("Initialized ");
  Serial.print(devices.size());
  Serial.println(" devices");
  
  // Initialize climate controller
  initializeClimateController();
  
  // Prompt for AutoTune if climate controller is enabled
  AutoTuneMode autoTuneMode = AutoTuneMode::SKIP;
  if (Configuration::isClimateControllerEnabled() && climateController != nullptr) {
    autoTuneMode = promptForAutoTune();
  }
  
  // Initialize display device
  initializeDisplayDevice();
  
  // Read initial sensor data
  readAndPrintInitialSensorData();
  
  // Setup MQTT identifiers
  clientId = Configuration::getProjectNumber() + "_" + Configuration::getShowcaseId();
  topic = Configuration::getDeviceName() + "/" + Configuration::getProjectNumber() + "/" + Configuration::getShowcaseId();
  
  // Print debugging information
  printDebugInfo();
  
  // WiFi connection attempt
  Serial.println("Attempting WiFi connection...");
  bool wifiConnected = WifiMqttHandler::connectToWiFiWithCheck(Configuration::getWiFiSSID(), Configuration::getWiFiPassword());
  
  if (!wifiConnected) {
    Serial.println("WiFi connection failed - running in offline mode");
    offlineMode = true;
  } else {
    Serial.println("WiFi connected");
    offlineMode = false;
    
    // Initialize time synchronization
    if (rtc && rtc->isInitialized()) {
      TimeHandler::fetchTime(*rtc);
    }

    // Attempt MQTT connection
    if (!WifiMqttHandler::connectToMqttBrokerWithCheck(client, espClient, 
        Configuration::getMqttsServer(), rootCACertificate, 
        Configuration::getMqttsPort(), clientId, topic,
        Configuration::getFlespiToken())) {
      Serial.println("MQTT connection failed - will retry in main loop");
    } else {
      Serial.println("MQTT connected");
    }
  }
  
  Serial.println("Setup complete");
  
  // Turn off DAC now that setup is complete
  if (Configuration::isClimateControllerEnabled() && climateController != nullptr) {
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    GP8403dac* dac = (GP8403dac*)registry.getDeviceByType("DAC", 0);
    if (dac != nullptr && dac->isInitialized()) {
      dac->setChannelVoltage(0, 1.0);
    }
  }
  
  delay(500);
  setupComplete = true;
  
  // Start AutoTune if requested
  if (autoTuneMode != AutoTuneMode::SKIP && Configuration::isClimateControllerEnabled() && climateController != nullptr) {
    Serial.println("Starting PID AutoTune...");
    
    if (autoTuneMode == AutoTuneMode::NORMAL) {
      climateController->startTemperatureAutoTune();
    } else if (autoTuneMode == AutoTuneMode::FAST) {
      climateController->startTemperatureAutoTuneFast();
    }
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

  // Handle serial commands for AutoTune control
  handleSerialCommands();
}

// New function to read and print initial sensor values
void readAndPrintInitialSensorData() {
  for (size_t i = 0; i < devices.size(); i++) {
    Device* device = devices[i];
    if (device == nullptr || !device->isInitialized()) {
      continue;
    }
      String deviceName = device->getType() + "_" + String(device->getDeviceIndex());
    
    I2CHandler::selectTCA(device->getTCAChannel());
    auto data = device->readData();
      for (const auto& channel : device->getChannels()) {
      String channelKey = channel.first;
      String deviceSpecificKey = deviceName + "_" + channelKey; // Make key device-specific
      std::string key = std::string(deviceSpecificKey.c_str());
      float value = data[channelKey].toFloat();
      
      // Store initial value in lastSensorValues to establish baseline
      lastSensorValues[key] = value;
    }
  }
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
        Device* device = devices[i];        if (device == nullptr) {
            continue;
        }
        if (!device->isInitialized()) {
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
  Serial.print("Created ");
  Serial.print(devices.size());
  Serial.println(" devices");
}

// Function to initialize the climate controller
void initializeClimateController() {
    if (Configuration::isClimateControllerEnabled()) {
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
            
            // First try SPIFFS (priority for climate-specific settings with auto-tune results)
            if (climateConfig.loadFromJsonFile("/ClimateConfig.json")) {
                configLoadedFromFile = true;
            }
            // Then try SD card if SPIFFS failed (fallback to default settings)
            else if (climateConfig.loadFromJsonFile("/ClimateConfig.json")) {
                configLoadedFromFile = true;
            }
            
            // If ClimateConfig.json doesn't exist or is incomplete, fall back to main config.json
            if (!configLoadedFromFile) {
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
                climateConfig.createDefaultJsonFile("/ClimateConfig.json");
            }
            
            // Get final configuration values from ClimateConfig
            float temperatureSetpoint = climateConfig.getTemperatureSetpoint();
            float humiditySetpoint = climateConfig.getHumiditySetpoint();
            String climateMode = climateConfig.getClimateMode();
            String humidityMode = climateConfig.getHumidityMode();
            
            // Convert string modes to boolean flags
            bool temperatureEnabled = (climateMode != "OFF");
            bool humidityEnabled = (humidityMode != "OFF");
            
            // Configure all parameters at once
            climateController->configure(temperatureSetpoint, humiditySetpoint, temperatureEnabled, humidityEnabled);
            
            Serial.print("Climate Controller: T=");
            Serial.print(temperatureSetpoint, 1);
            Serial.print("°C, H=");
            Serial.print(humiditySetpoint, 0);
            Serial.println("%");
            
        } else {
            Serial.println("Failed to initialize climate controller");
        }
    } else {
        Serial.println("Climate controller disabled");
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
        Serial.print(currentTemperature, 2);
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
    // Get Display device from DeviceRegistry
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    displayDevice = (Display*)registry.getDeviceByType("Display", 0);
    
    if (displayDevice != nullptr && displayDevice->isInitialized()) {
        // Show initial startup message
        displayDevice->clear();
        displayDevice->setCursor(0, 0);
        displayDevice->print("Climate Control");
        displayDevice->setCursor(0, 1);
        displayDevice->print("Initializing...");
        delay(2000);
    } else {
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

// Function to prompt user for AutoTune with mode selection
AutoTuneMode promptForAutoTune() {
    Serial.println();
    Serial.println("========================================");
    Serial.println("       PID AutoTune Configuration");
    Serial.println("========================================");
    Serial.println("AutoTune will automatically determine optimal PID parameters");
    Serial.println("for temperature control by analyzing system response.");
    Serial.println();
    Serial.println("Available AutoTune modes:");
    Serial.println("  N - NORMAL  (2-4 hours, highest accuracy, production use)");
    Serial.println("  F - FAST    (15-30 minutes, good accuracy, testing/demo)");
    Serial.println("  S - SKIP    (No AutoTune, use current PID settings)");
    Serial.println();
    Serial.println("WARNING: AutoTune process will cause temperature fluctuations!");
    Serial.println();
    Serial.print("Select AutoTune mode (N/F/S) within 10 seconds: ");
    Serial.flush();
    
    unsigned long startTime = millis();
    const unsigned long timeoutMs = 10000; // 10 seconds for mode selection
    bool inputReceived = false;
    String response = "";
    
    // Wait for input or timeout
    while (millis() - startTime < timeoutMs && !inputReceived) {
        if (Serial.available()) {
            // Read the entire input string
            response = Serial.readStringUntil('\n');
            response.trim(); // Remove whitespace and control characters
            inputReceived = true;
            break;
        }
        delay(10); // Small delay to prevent excessive CPU usage
    }
    
    if (inputReceived) {
        Serial.println(response); // Echo the response
        
        // Parse the response (case insensitive)
        if (response.equalsIgnoreCase("N") || response.equalsIgnoreCase("NORMAL")) {
            Serial.println("✓ NORMAL AutoTune SELECTED");
            Serial.println("  Duration: 2-4 hours for complete thermal analysis");
            Serial.println("  Accuracy: Highest - recommended for production use");
            Serial.println("========================================");
            Serial.println();
            return AutoTuneMode::NORMAL;
        } else if (response.equalsIgnoreCase("F") || response.equalsIgnoreCase("FAST")) {
            Serial.println("✓ FAST AutoTune SELECTED");
            Serial.println("  Duration: 15-30 minutes for quick analysis");
            Serial.println("  Accuracy: Good - suitable for testing and demos");
            Serial.println("  WARNING: Less accurate than normal mode!");
            Serial.println("========================================");
            Serial.println();
            return AutoTuneMode::FAST;
        } else if (response.equalsIgnoreCase("S") || response.equalsIgnoreCase("SKIP")) {
            Serial.println("→ AutoTune SKIPPED");
            Serial.println("  Using current PID parameters from configuration");
            Serial.println("========================================");
            Serial.println();
            return AutoTuneMode::SKIP;
        } else {
            Serial.println("→ Invalid selection - AutoTune SKIPPED");
            Serial.println("========================================");
            Serial.println();
            return AutoTuneMode::SKIP;
        }
    } else {
        Serial.println("TIMEOUT");
        Serial.println("→ AutoTune SKIPPED (no input received)");
        Serial.println("========================================");
        Serial.println();
        return AutoTuneMode::SKIP;
    }
}

// Function to handle serial commands for AutoTune control
void handleSerialCommands() {
    if (!Serial.available()) {
        return;
    }
    
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove leading/trailing whitespace
    command.toLowerCase(); // Make command case-insensitive
    
    if (command == "help" || command == "?") {
        Serial.println();
        Serial.println("========================================");
        Serial.println("       Available Serial Commands");
        Serial.println("========================================");
        Serial.println("help, ?          - Show this help menu");
        Serial.println("autotune start   - Start temperature AutoTune");
        Serial.println("autotune stop    - Stop AutoTune");
        Serial.println("autotune status  - Show AutoTune status");
        Serial.println("autotune results - Show saved AutoTune results");
        Serial.println("autotune clear   - Clear saved AutoTune results");
        Serial.println("status           - Show system status");
        Serial.println("========================================");
        Serial.println();
    }
    else if (command == "autotune start") {
        if (!Configuration::isClimateControllerEnabled() || climateController == nullptr) {
            Serial.println("ERROR: Climate controller not enabled or not initialized");
            return;
        }
        
        if (climateController->isAutoTuning()) {
            Serial.println("AutoTune is already running");
            return;
        }
        
        Serial.println();
        Serial.println("========================================");
        Serial.println("       STARTING PID AutoTune");
        Serial.println("========================================");
        Serial.println("AutoTune process starting...");
        Serial.println("This may take several minutes to complete.");
        Serial.println("Temperature will fluctuate during calibration.");
        Serial.println("Use 'autotune stop' to cancel if needed.");
        Serial.println("========================================");
        Serial.println();
        
        if (climateController->startTemperatureAutoTune()) {
            Serial.println("✓ AutoTune started successfully");
        } else {
            Serial.println("✗ Failed to start AutoTune");
        }
    }
    else if (command == "autotune stop") {
        if (!Configuration::isClimateControllerEnabled() || climateController == nullptr) {
            Serial.println("ERROR: Climate controller not enabled or not initialized");
            return;
        }
        
        if (!climateController->isAutoTuning()) {
            Serial.println("AutoTune is not currently running");
            return;
        }
        
        Serial.println("Stopping AutoTune...");
        climateController->stopAutoTune();
        Serial.println("✓ AutoTune stopped");
    }    else if (command == "autotune status") {
        if (!Configuration::isClimateControllerEnabled() || climateController == nullptr) {
            Serial.println("ERROR: Climate controller not enabled or not initialized");
            return;
        }
        
        Serial.println();
        climateController->printAutoTuneStatus();
        Serial.println();
    }else if (command == "autotune results") {
        // Load climate config to get AutoTune results
        ClimateConfig& config = ClimateConfig::getInstance();
        if (config.hasAutoTuneResults()) {
            Serial.println();
            Serial.println("========================================");
            Serial.println("       Saved AutoTune Results");
            Serial.println("========================================");
            Serial.print("Kp (Proportional): ");
            Serial.println(config.getAutoTuneKp(), 4);
            Serial.print("Ki (Integral):      ");
            Serial.println(config.getAutoTuneKi(), 4);
            Serial.print("Kd (Derivative):    ");
            Serial.println(config.getAutoTuneKd(), 4);
            Serial.println("========================================");
            Serial.println("These values are automatically used for PID control.");
            Serial.println();
        } else {
            Serial.println("No AutoTune results found.");
            Serial.println("Run 'autotune start' to generate optimal PID parameters.");
        }
    }    else if (command == "autotune clear") {        Serial.println("Clearing saved AutoTune results...");        ClimateConfig& config = ClimateConfig::getInstance();
        config.clearAutoTuneResults();
        config.saveToJsonFile("/ClimateConfig.json");
        config.saveToJsonFile("/config.json");
        Serial.println("✓ AutoTune results cleared. Default PID parameters will be used.");
    }
    else if (command == "status") {
        Serial.println();
        Serial.println("========================================");
        Serial.println("         System Status");
        Serial.println("========================================");
        Serial.print("Setup Complete: ");
        Serial.println(setupComplete ? "YES" : "NO");
        Serial.print("WiFi Status: ");
        Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
        Serial.print("MQTT Status: ");
        Serial.println(client.connected() ? "Connected" : "Disconnected");
        Serial.print("Climate Controller: ");
        if (Configuration::isClimateControllerEnabled() && climateController != nullptr) {
            Serial.println("Enabled");
            Serial.print("  AutoTune Active: ");
            Serial.println(climateController->isAutoTuning() ? "YES" : "NO");
        } else {
            Serial.println("Disabled or Not Initialized");
        }
        Serial.print("Offline Mode: ");
        Serial.println(offlineMode ? "YES" : "NO");
        Serial.println("========================================");
        Serial.println();
    }
    else if (command.length() > 0) {
        Serial.print("Unknown command: '");
        Serial.print(command);
        Serial.println("'");
        Serial.println("Type 'help' for available commands.");
    }
}


