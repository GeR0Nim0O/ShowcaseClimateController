#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include "Device.h"
#include "BH1705Sensor/BH1705sensor.h"
#include "SHT31Sensor/SHT31sensor.h"
#include "SCALESSensor/SCALESsensor.h"
#include "DS3231Rtc/DS3231rtc.h" // Include the DS3231rtc header

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

void readAndSendDataFromDevices();
void handleButtonPress();
void printDebugInfo();
void printCreatedSensors(); // Declare the function here
void logDataToSD(const String& deviceName, const String& currentTime, float value, const String& sensorType); // Update function declaration
void sendDataOverMQTT(const String& deviceName, const String& projectNr, const String& showcaseId, const char* sensorType, float sensorValue, const String& currentTime, int deviceIndex);
void setCustomMqttSettings();
void setMqttThrottling(bool enable, unsigned long interval = 60000); // Declare the function here
void sendAllChangedSensorData(); // Add this function declaration
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

  // Connect to WiFi
  if (!WifiMqttHandler::connectToWiFiWithCheck(Configuration::getWiFiSSID(), Configuration::getWiFiPassword())) {
    return;
  }

  // Connect to TimeAPI and NTP
  // Only fetch time from RTC if RTC is connected and initialized
  if (rtc && rtc->isInitialized()) {
    TimeHandler::fetchTime(*rtc);
  } else {
    Serial.println("RTC not connected or not initialized. Skipping RTC time fetch.");
  }

  // Connect to MQTT broker
  if (!WifiMqttHandler::connectToMqttBrokerWithCheck(client, espClient, 
      Configuration::getMqttsServer(), rootCACertificate, 
      Configuration::getMqttsPort(), clientId, topic,
      Configuration::getFlespiToken())) {
    Serial.println("Failed to connect to MQTT broker");
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
  static unsigned long lastReconnectAttempt = 0;
  static bool mqttRetryDone = false;

  // Ensure WiFi and MQTT connections are maintained
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastReconnectAttempt > 15000) {
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.begin(Configuration::getWiFiSSID().c_str(), Configuration::getWiFiPassword().c_str());
      lastReconnectAttempt = millis();
      mqttRetryDone = false; // Reset MQTT retry flag when WiFi reconnects
    }
    return; // Wait for WLAN UP before proceeding
  }

  if (!client.connected()) {
    if (millis() - lastReconnectAttempt > 10000) { // Try every 10 seconds
      Serial.println("Reconnecting to MQTT broker...");
      WifiMqttHandler::connectToMqttBroker(client, espClient, 
                                         Configuration::getMqttsServer().c_str(), 
                                         rootCACertificate, 
                                         Configuration::getMqttsPort(), 
                                         clientId.c_str(), topic.c_str(),
                                         Configuration::getFlespiToken().c_str());
      lastReconnectAttempt = millis();
    }
    return; // Wait for MQTT UP before proceeding
  }

  client.loop(); // Ensure the MQTT client loop is called to maintain the connection

  TimeHandler::fetchCurrentTimePeriodically(rtc, lastTimeFetch, timeFetchInterval);

  // Read and send data from each device
  readAndSendDataFromDevices();

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
      std::string key = std::string(channelKey.c_str());
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

void readAndSendDataFromDevices() {
    // Flag to determine if we should print data this cycle (only true every 60 seconds)
    bool shouldPrintData = throttleMqtt && (millis() - lastMqttSendTime >= mqttThrottleInterval);
    
    if (shouldPrintData) {
        Serial.println("\n=== Sensor Readings (60-second update) ===");
    }
    
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
            String channelKey = channel.first;
            std::string key = std::string(channelKey.c_str()); // Convert channelKey to std::string
            float value = data[channelKey].toFloat(); // Use String key and convert to float
            String currentTime;
            // Only fetch time from RTC if RTC is connected and initialized
            if (rtc && rtc->isInitialized()) {
                currentTime = TimeHandler::getCurrentTime(*rtc);
            } else {
                currentTime = "";
            }
            String deviceName = device->getType() + "_" + String(device->getDeviceIndex());
            String projectNr = Configuration::getProjectNumber();
            String showcaseId = Configuration::getShowcaseId();

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
                
                // Always store the data for MQTT sending, regardless of threshold
                SensorData sensorData;
                sensorData.deviceName = deviceName;
                sensorData.projectNr = projectNr;
                sensorData.showcaseId = showcaseId;
                sensorData.sensorType = channel.second;
                sensorData.sensorValue = value;
                sensorData.currentTime = currentTime;
                sensorData.deviceIndex = device->getDeviceIndex();
                sensorData.changed = true;
                
                changedSensorData[key] = sensorData;
            }
            
            // This conditional is still used for logging to SD and determining meaningful changes
            if (abs(value - lastValue) >= threshold) {
                lastSensorValues[key] = value;

                if (channel.second != "Time") {
                    // Log to SD only when the value has changed beyond threshold
                    logDataToSD(deviceName, currentTime, value, channel.second);
                }
            }
            // No else clause needed since we always update changedSensorData on the 60-second cycle
        }
    }
    
    // Check if it's time to send all changed data via MQTT
    if (shouldPrintData) {
        Serial.println("\nSensor data collected - sending to MQTT...");
        sendAllChangedSensorData();
    }
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

// New function to control MQTT throttling settings
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
