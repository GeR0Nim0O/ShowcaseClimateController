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

#define BUTTON_PIN 34
#define DEBOUNCE_DELAY 50 // Debounce delay in milliseconds
bool buttonPressed = false;
unsigned long lastDebounceTime = 0; // Last time the button state was toggled
bool stopSendingMQTT = false;
unsigned long buttonPressTime = 0;

WiFiClientSecure espClient; // Use WiFiClientSecure instead of WiFiClient
PubSubClient client(espClient);

// Custom WiFi settings
String customSSID = "Ron&Rowie_Gast";
String customPassword = "Gast@Ron&Rowie";
bool useCustomWiFi = true; // Set to true to use custom WiFi settings

// Custom MQTT settings
String customMqttServer = "mqtt.flespi.io";
int customMqttPort = 8883;
String customFlespiToken = "ONz40m0iGTFbiFMcp14lLnt1Eb31qnPulPkg5DkJUuGGY6OhJhN1iPqImaRT0qbp";
bool useCustomMqtt = true; // Set to true to use custom MQTT settings

// MQTT throttling settings
bool throttleMqtt = true; // Enable MQTT throttling to send data only once every minute
unsigned long mqttThrottleInterval = 60000; // 1 minute in milliseconds
unsigned long lastMqttSendTime = 0; // Last time data was sent via MQTT

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
const unsigned long timeFetchInterval = 3600000; // Fetch time every 1 hour

bool setupComplete = false; // Flag to indicate setup completion
bool offlineMode = false; // Flag to indicate if running in offline mode

std::vector<Device*> devices; // Vector to hold device pointers
std::map<uint8_t, std::vector<uint8_t>> tcaScanResults; // Map to store TCA scan results
DS3231rtc* rtc; // Pointer to hold RTC device

struct StringComparator {
    bool operator()(const String& a, const String& b) const {
        return a.compareTo(b) < 0;
    }
};

std::map<std::string, float> lastSensorValues; // Changed from String to std::string

String ssid;
String password;
String mqtt_server;
int mqtt_port;
String projectNr;
String showcaseId;
String deviceName;
String utc;

String clientId;
String topic;

int wifiRetryCount = 0;
int mqttRetryCount = 0;
int apiRetryCount = 0;
int ntpRetryCount = 0;

// Global variables for climate controller
PCF8574gpio* gpioExpander = nullptr;
SHTsensor* climateTemperatureSensor = nullptr;
GP8403dac* climateDac = nullptr;  // Add DAC pointer
ClimateController* climateController = nullptr;
bool climateControllerEnabled = true;

// Climate control parameters
float temperatureSetpoint = 22.0;
float humiditySetpoint = 50.0;
bool autoFanControlEnabled = true;
ClimateMode climateMode = ClimateMode::AUTO;
HumidityMode humidityMode = HumidityMode::AUTO;

void readAndPrintInitialSensorData(); // Add this function prototype
void readAndSendDataFromDevices();
void handleButtonPress();
void printDebugInfo();
void printCreatedSensors(); // Declare the function here
void logDataToSD(const String& deviceName, const String& currentTime, float value, const String& sensorType); // Update function declaration
void sendDataOverMQTT(const String& deviceName, const String& projectNr, const String& showcaseId, const char* sensorType, float sensorValue, const String& currentTime, int deviceIndex);
void setCustomMqttSettings();
void setMqttThrottling(bool enable, unsigned long interval = 60000); // Declare the function here
void sendAllChangedSensorData(); // Add this function declaration
void initializeClimateController(); // Function to initialize climate controller
void updateClimateController(); // Function to update climate controller
void printClimateControlStatus(); // Add climate status printing function

void setup()
{
  Serial.begin(115200);
  delay(5000); 

  // Initialize I2C bus
  I2CHandler::initializeI2C(); 

  // Perform I2C scan before connecting to any devices
  I2CHandler::scanI2C(); 

  // Initialize SD card and configuration
  if (!SDHandler::initializeSDCardAndConfig()) {
    Serial.println("Failed to initialize SD card.");
    return;
  }

  // Load device configurations from config.json
  if (!Configuration::loadConfigFromSD("/config.json"))
  {
    Serial.println("Failed to load config from SD card. Using default or last known values.");
    // Set default or last known values here
    Configuration::setWiFiSSID("Ron");
    Configuration::setWiFiPassword("ikweethet");
    Configuration::setMqttsServer("mqtt.flespi.io");
    Configuration::setMqttsPort(8883);
    Configuration::setFlespiToken("ONz40m0iGTFbiFMcp14lLnt1Eb31qnPulPkg5DkJUuGGY6OhJhN1iPqImaRT0qbp"); // Replace with your actual token
    Configuration::setProjectNumber("12345");
    Configuration::setShowcaseId("67");
    Configuration::setDeviceName("TEST");
    Configuration::setTimezone("UTC");
  } else {
    Serial.println("Config loaded successfully from SD card.");
  }

  // Print devices configuration
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

  // Set custom WiFi settings
  if (useCustomWiFi) {
    Configuration::setWiFiSSID(customSSID);
    Configuration::setWiFiPassword(customPassword);
  }

  // Set custom MQTT settings
  setCustomMqttSettings();

  clientId = Configuration::getProjectNumber() + "_" + Configuration::getShowcaseId();
  topic = Configuration::getDeviceName() + "/" + Configuration::getProjectNumber() + "/" + Configuration::getShowcaseId();
  // Print debugging information
  printDebugInfo();

  // Try to connect to WiFi (don't stop if it fails)
  Serial.println("Attempting WiFi connection...");
  bool wifiConnected = WifiMqttHandler::connectToWiFiWithCheck(Configuration::getWiFiSSID(), Configuration::getWiFiPassword());
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

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Initialize button pin

  delay(500);

  setupComplete = true; // Indicate that setup is complete
  Serial.println("Setup complete: " + String(setupComplete));

  // Configure MQTT throttling - set to true to only send once per minute
  setMqttThrottling(true, 60000); // Enable throttling, every 60 seconds
}

void loop() {
  if (!setupComplete) {
    return; // Exit loop if setup is not complete
  }

  static unsigned long lastSendTime = 0;
  static unsigned long lastConnectionRetry = 0;
  static bool mqttRetryDone = false;
  static int wifiReconnectAttempts = 0;
  static int mqttReconnectAttempts = 0;
  static bool wifiSkipped = false;
  static bool mqttSkipped = false;
  const int maxReconnectAttempts = 5;
  const unsigned long connectionRetryInterval = 60000; // 1 minute

  // Check if it's time to send MQTT data (every minute)
  bool timeToSendMqtt = throttleMqtt && (millis() - lastMqttSendTime >= mqttThrottleInterval);
    // Only attempt reconnections every minute, aligned with MQTT sending schedule
  if (timeToSendMqtt || (millis() - lastConnectionRetry >= connectionRetryInterval)) {
    
    // Print connection status for debugging every minute
    Serial.print("Connection Status - WiFi: ");
    Serial.print(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.print(", MQTT: ");
    Serial.println(client.connected() ? "Connected" : "Disconnected");
    
    // Try to reconnect WiFi if disconnected and not skipped
    if (WiFi.status() != WL_CONNECTED && !wifiSkipped) {
      wifiReconnectAttempts++;
      Serial.print("Reconnecting to WiFi... Attempt ");
      Serial.print(wifiReconnectAttempts);
      Serial.print("/");
      Serial.println(maxReconnectAttempts);
      
      if (wifiReconnectAttempts <= maxReconnectAttempts) {
        WiFi.disconnect();
        WiFi.begin(Configuration::getWiFiSSID().c_str(), Configuration::getWiFiPassword().c_str());
        mqttReconnectAttempts = 0; // Reset MQTT attempts when WiFi reconnects
        mqttSkipped = false; // Reset MQTT skip flag when WiFi reconnects
        
        // Wait for WiFi connection with timeout
        unsigned long wifiStartTime = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - wifiStartTime < 15000)) {
          delay(500);
          Serial.print(".");
        }
        
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("\nWiFi connected successfully!");
        } else {
          Serial.println("\nWiFi connection timeout");
        }
      } else {
        Serial.println("Max WiFi reconnection attempts reached. Skipping WiFi reconnection.");
        wifiSkipped = true;
      }
    }

    // Try to reconnect MQTT if disconnected and not skipped and WiFi is connected
    if (WiFi.status() == WL_CONNECTED && !client.connected() && !mqttSkipped) {
      mqttReconnectAttempts++;
      Serial.print("Reconnecting to MQTT broker... Attempt ");
      Serial.print(mqttReconnectAttempts);
      Serial.print("/");
      Serial.println(maxReconnectAttempts);
        if (mqttReconnectAttempts <= maxReconnectAttempts) {
        WifiMqttHandler::connectToMqttBroker(client, espClient, 
                                         Configuration::getMqttsServer().c_str(), 
                                         rootCACertificate, 
                                         Configuration::getMqttsPort(), 
                                         clientId.c_str(), topic.c_str(),
                                         Configuration::getFlespiToken().c_str());
        
        if (client.connected()) {
          Serial.println("MQTT connected successfully!");
        } else {
          Serial.println("MQTT connection failed");
        }
      } else {
        Serial.println("Max MQTT reconnection attempts reached. Skipping MQTT reconnection.");
        mqttSkipped = true;
      }
    }
    
    lastConnectionRetry = millis();
  }
  // Reset reconnection attempts when WiFi reconnects successfully
  if (WiFi.status() == WL_CONNECTED && wifiSkipped) {
    Serial.println("WiFi reconnected successfully. Resetting WiFi attempt counter.");
    wifiReconnectAttempts = 0;
    wifiSkipped = false;
    mqttReconnectAttempts = 0;
    mqttSkipped = false;
    offlineMode = false;
  }

  // Reset MQTT reconnection attempts when MQTT reconnects successfully
  if (client.connected() && mqttSkipped) {
    Serial.println("MQTT reconnected successfully. Resetting MQTT attempt counter.");
    mqttReconnectAttempts = 0;
    mqttSkipped = false;
  }

  // Update offline mode status
  offlineMode = (WiFi.status() != WL_CONNECTED);

  if (client.connected()) {
    client.loop(); // Ensure the MQTT client loop is called to maintain the connection
  }

  TimeHandler::fetchCurrentTimePeriodically(rtc, lastTimeFetch, timeFetchInterval);

  // Read and send data from each device
  readAndSendDataFromDevices();
  
  // Update climate controller
  if (climateControllerEnabled && climateController != nullptr) {
    updateClimateController();
  }

  handleButtonPress();

  // Add a direct check here to periodically send data
  if (throttleMqtt && (millis() - lastMqttSendTime >= mqttThrottleInterval)) {
      if (WiFi.status() == WL_CONNECTED && client.connected()) {
          sendAllChangedSensorData();
      }
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
    if (WiFi.status() != WL_CONNECTED || !client.connected() || stopSendingMQTT) {
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
    bool shouldPrintData = throttleMqtt && (millis() - lastMqttSendTime >= mqttThrottleInterval);
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
            float lastValue = lastSensorValues[key];
            float threshold = device->getThreshold(channelKey);
              // Debug: Print threshold information (remove this later)
            if (shouldPrintData && (channelKey == "T" || channelKey == "H")) {
                Serial.print("DEBUG: Device ");
                Serial.print(displayName);
                Serial.print(" Channel ");
                Serial.print(channelKey);
                Serial.print(" (key: ");
                Serial.print(deviceSpecificKey);
                Serial.print(") threshold: ");
                Serial.print(threshold);
                Serial.print(", lastValue: ");
                Serial.print(lastValue);
                Serial.print(", currentValue: ");
                Serial.print(value);
                Serial.print(", diff: ");
                Serial.println(abs(value - lastValue));
            }
            
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
                    
                    // NEW: Send data to MQTT immediately when threshold exceeded
                    if (WiFi.status() == WL_CONNECTED && client.connected() && !stopSendingMQTT) {
                        // Send this specific sensor data immediately via MQTT
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
            // No else clause needed since we always update changedSensorData on the 60-second cycle
        }
    }
    
    // Print climate controller status when thresholds are exceeded or during 60-second cycle
    if ((anyThresholdExceeded || shouldPrintData) && climateController != nullptr) {
        printClimateControlStatus();
    }
    
    // Check if it's time to send all changed data via MQTT
    if (shouldPrintData) {
        if (WiFi.status() == WL_CONNECTED && client.connected()) {
            Serial.println("\nSensor data collected - sending to MQTT...");
            sendAllChangedSensorData();
        } else if (WiFi.status() != WL_CONNECTED) {
            Serial.println("\nSensor data collected - WiFi not connected, data logged to SD only");
        } else if (!client.connected()) {
            Serial.println("\nSensor data collected - MQTT not connected, data logged to SD only");
        }
    }
}

// Add new function to print climate control status
void printClimateControlStatus() {
    if (climateController == nullptr) {
        return;
    }
    
    Serial.println("\n=== Climate Control Status ===");
    
    // Print current temperature and humidity readings
    Serial.print("Current Temperature: ");
    Serial.print(climateController->getCurrentTemperature(), 1);
    Serial.print("°C (setpoint: ");
    Serial.print(climateController->getTemperatureSetpoint(), 1);
    Serial.println("°C)");
    
    Serial.print("Current Humidity: ");
    Serial.print(climateController->getCurrentHumidity(), 1);
    Serial.print("% (setpoint: ");
    Serial.print(climateController->getHumiditySetpoint(), 1);
    Serial.println("%)");
    
    // Print temperature control status with percentages
    Serial.print("Temperature Control: ");
    if (climateController->isHeating()) {
        Serial.print("Heating ");
        Serial.print(climateController->getHeatingPower(), 0);
        Serial.println("%");
    } else if (climateController->isCooling()) {
        Serial.print("Cooling ");
        Serial.print(climateController->getCoolingPower(), 0);
        Serial.println("%");
    } else {
        Serial.println("OFF");
    }
    
    // Print humidity control status
    Serial.print("Humidity Control: ");
    if (climateController->isHumidifying()) {
        Serial.println("Humidifying");
    } else if (climateController->isDehumidifying()) {
        Serial.println("Dehumidifying");
    } else {
        Serial.println("OFF");
    }
    
    // Print fan status
    Serial.print("Fans: Interior ");
    Serial.print(climateController->isFanInteriorOn() ? "ON" : "OFF");
    Serial.print(", Exterior ");
    Serial.println(climateController->isFanExteriorOn() ? "ON" : "OFF");
    
    Serial.println("=============================");
}

// Update function to include sensor type parameter
void logDataToSD(const String& deviceName, const String& currentTime, float value, const String& sensorType) {
    String json = "{\"device\":\"" + deviceName + "\",\"type\":\"" + sensorType + "\",\"timestamp\":\"" + currentTime + "\",\"value\":" + String(value) + "}";
    SDHandler::logJson(json.c_str());
}

void sendAllChangedSensorData() {
    if (WiFi.status() != WL_CONNECTED || !client.connected() || stopSendingMQTT) {
        return;
    }
    
    bool dataWasSent = false;
    
    for (auto& item : changedSensorData) {
        SensorData& data = item.second;
        
        if (data.changed) {
            // Use the new helper function
            sendSensorDataOverMQTT(data);
            
            data.changed = false; // Reset changed flag
            dataWasSent = true;
        }
    }
    
    if (dataWasSent) {
        Serial.println("Sent all changed sensor data via MQTT");
    } else {
        Serial.println("No changed sensor data to send");
    }
    
    lastMqttSendTime = millis(); // Update last send time
}

void handleButtonPress() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != buttonPressed) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading == LOW && !buttonPressed) {
      buttonPressed = true;
      stopSendingMQTT = true;
      buttonPressTime = millis();
      Serial.println("Button pressed. Do you want to copy the config file? Press again to confirm.");
    } else if (reading == LOW && buttonPressed) {
      if (millis() - buttonPressTime < 5000) {
        if (SDHandler::updateConfig()) {
          Serial.println("Config file copied successfully. Resuming MQTT messaging in 5 seconds...");
        } else {
          Serial.println("Failed to copy config file. Resuming MQTT messaging.");
        }
        stopSendingMQTT = false;
        buttonPressed = false; // Reset button state
      } else {
        Serial.println("Timeout. No response received. Resuming normal operation.");
        stopSendingMQTT = false;
        buttonPressed = false; // Reset button state
      }
    } else if (reading == HIGH && buttonPressed) {
      buttonPressed = false;
      if (stopSendingMQTT && (millis() - buttonPressTime) > 5000) {
        Serial.println("Resuming MQTT messaging.");
        stopSendingMQTT = false;
      }
    }
  }
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

// Function to control MQTT throttling settings
void setMqttThrottling(bool enable, unsigned long interval) {
    throttleMqtt = enable;
    mqttThrottleInterval = interval;
    
    Serial.print("MQTT Throttling ");
    if (enable) {
        Serial.print("enabled - sending once every ");
        Serial.print(interval / 1000);
        Serial.println(" seconds");
    } else {
        Serial.println("disabled - sending all data");
    }
}

void setCustomMqttSettings() {
    if (useCustomMqtt) {
        Serial.println("Using custom MQTT settings");
        Configuration::setMqttsServer(customMqttServer);
        Configuration::setMqttsPort(customMqttPort);
        Configuration::setFlespiToken(customFlespiToken);
        
        // Update client ID and topic with new settings
        clientId = Configuration::getProjectNumber() + "_" + Configuration::getShowcaseId();
        topic = Configuration::getDeviceName() + "/" + Configuration::getProjectNumber() + "/" + Configuration::getShowcaseId();
        
        Serial.println("Custom MQTT settings applied:");
        Serial.print("MQTT Server: ");
        Serial.println(customMqttServer);
        Serial.print("MQTT Port: ");
        Serial.println(customMqttPort);
        Serial.print("Flespi Token: ");
        Serial.println(customFlespiToken.substring(0, 4) + "..." + customFlespiToken.substring(customFlespiToken.length() - 4));

        // Also set throttling information
        Serial.print("MQTT Throttling: ");
        Serial.println(throttleMqtt ? "Enabled (sending once every minute)" : "Disabled");
    }
}

// Function to initialize the climate controller
void initializeClimateController() {
  // Add extra safety
  try {
    // Use DeviceRegistry to get devices instead of manual searching
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    
    // Get GPIO expander from DeviceRegistry
    gpioExpander = (PCF8574gpio*)registry.getDeviceByType("GPIO", 0);  // Get first GPIO expander
    if (gpioExpander != nullptr) {
      Serial.println("Found GPIO expander for climate control via DeviceRegistry");
    } else {
      Serial.println("No GPIO expander found in DeviceRegistry");
    }
      // Get temperature/humidity sensor from DeviceRegistry - specifically look for Interior labeled sensor
    climateTemperatureSensor = (SHTsensor*)registry.getDeviceByTypeAndLabel("TemperatureHumidity", "Interior");
    if (climateTemperatureSensor != nullptr) {
      Serial.println("Found INTERIOR temperature/humidity sensor for climate control via DeviceRegistry");
      Serial.print("Using sensor: ");
      Serial.print(climateTemperatureSensor->getDeviceName());
      Serial.print(" with label: ");
      Serial.println(climateTemperatureSensor->getDeviceLabel());
    } else {
      // Fallback to first available sensor if no Interior labeled sensor found
      Serial.println("No Interior labeled sensor found, using first available temperature/humidity sensor");
      climateTemperatureSensor = (SHTsensor*)registry.getDeviceByType("TemperatureHumidity", 0);
      if (climateTemperatureSensor != nullptr) {
        Serial.println("Found temperature/humidity sensor for climate control via DeviceRegistry (fallback)");
      } else {
        Serial.println("No temperature/humidity sensor found in DeviceRegistry");
      }
    }
    
    // Get DAC from DeviceRegistry - using proper DeviceRegistry access pattern
    climateDac = (GP8403dac*)registry.getDeviceByType("DAC", 0);  // Get first DAC
    if (climateDac != nullptr) {
      Serial.print("Found DAC device via DeviceRegistry: ");
      Serial.print(climateDac->getType());
      Serial.print(" with name: ");
      Serial.println(climateDac->getDeviceName());
    } else {
      Serial.println("No DAC found in DeviceRegistry");
    }
    
    // Create climate controller if we found the required devices
    if (gpioExpander != nullptr && climateTemperatureSensor != nullptr) {
      Serial.println("Creating climate controller with devices");
      
      // Create with safe checks
      try {
        Serial.println("Allocating climate controller...");
        climateController = new ClimateController(gpioExpander, climateTemperatureSensor, climateDac);
        
        if (climateController != nullptr) {
          Serial.println("Climate controller allocated, calling begin()");
          
          if (climateController->begin()) {
            Serial.println("Climate controller initialized successfully");
            
            // Set initial parameters
            climateController->setTemperatureSetpoint(temperatureSetpoint);
            climateController->setHumiditySetpoint(humiditySetpoint);
            climateController->setClimateMode(climateMode);
            climateController->setHumidityMode(humidityMode);
            
            // Enable automatic fan control
            climateController->setAutoFanControl(true);
            Serial.println("Automatic fan control enabled");
            
            Serial.print("Climate controller setpoints - Temperature: ");
            Serial.print(temperatureSetpoint);
            Serial.print("°C, Humidity: ");
            Serial.print(humiditySetpoint);
            Serial.println("%");
          } else {
            Serial.println("Failed to initialize climate controller");
            delete climateController;
            climateController = nullptr;
          }
        } else {
          Serial.println("Failed to allocate climate controller");
        }
      }
      catch (...) {
        Serial.println("Exception during climate controller initialization");
        if (climateController != nullptr) {
          delete climateController;
          climateController = nullptr;
        }
      }
    } else {
      Serial.println("Could not find required devices for climate controller");
    }
  }
  catch (...) {
    Serial.println("Exception during device discovery for climate controller");
  }
}

// Function to update the climate controller (called every loop)
void updateClimateController() {
    if (climateController == nullptr) {
        return; // Exit if climate controller is not initialized
    }
    
    // Only update climate controller every few seconds to avoid GPIO conflicts
    static unsigned long lastClimateUpdate = 0;
    if (millis() - lastClimateUpdate >= 5000) { // Update every 5 seconds
        climateController->update();
        lastClimateUpdate = millis();
    }
    
    // Optional: Print climate controller status periodically
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 60000) { // Print status every 60 seconds
        Serial.print("Climate status - Temperature: ");
        Serial.print(climateController->getCurrentTemperature());
        Serial.print("°C (setpoint: ");
        Serial.print(climateController->getTemperatureSetpoint());
        Serial.print("°C), Humidity: ");
        Serial.print(climateController->getCurrentHumidity());
        Serial.print("% (setpoint: ");
        Serial.print(climateController->getHumiditySetpoint());
        Serial.println("%)");
        
        // Print control status
        Serial.print("Climate control - Heating: ");
        Serial.print(climateController->isHeating() ? "ON" : "OFF");
        Serial.print(", Cooling: ");
        Serial.print(climateController->isCooling() ? "ON" : "OFF");
        Serial.print(", Humidifying: ");
        Serial.print(climateController->isHumidifying() ? "ON" : "OFF");
        Serial.print(", Dehumidifying: ");
        Serial.println(climateController->isDehumidifying() ? "ON" : "OFF");
        
        // Basic fan status
        Serial.print("Fan control - Interior: ");
        Serial.print(climateController->isFanInteriorOn() ? "ON" : "OFF");
        Serial.print(", Exterior: ");
        Serial.println(climateController->isFanExteriorOn() ? "ON" : "OFF");
        
        lastStatusPrint = millis();
    }
}
