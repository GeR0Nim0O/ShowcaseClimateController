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
#include <FS.h>
#include <SPIFFS.h>

// Initialize static member variables
std::map<String, String> Configuration::wifiConfig;
std::map<String, String> Configuration::mqttConfig;
std::map<String, String> Configuration::projectConfig;
std::map<String, String> Configuration::customWifiConfig;
std::map<String, String> Configuration::customMqttConfig;
std::map<String, String> Configuration::mqttThrottlingConfig;
std::map<String, String> Configuration::climateControllerConfig;
std::map<String, String> Configuration::displayConfig;
std::map<String, String> Configuration::systemConfig;
JsonDocument Configuration::devicesConfigDoc;
JsonObject Configuration::devicesConfig = JsonObject();

bool Configuration::loadConfigFromSD(const char* filename) {
    // Parse the JSON file from the SD card
    JsonDocument doc;
    
    if (!SDHandler::readJsonFile(filename, doc)) {
        // Silent failure - let the calling code handle fallback
        return false;
    }
    
    // Load configuration from parsed JSON
    if (!loadConfig(doc.as<JsonObject>())) {
        Serial.println("ERROR: Failed to parse config from SD card!");
        Serial.println("ERROR: config.json may be corrupted or invalid");
        return false;
    }
    
    Serial.println("Configuration loaded successfully from SD card");
    return true;
}

bool Configuration::loadConfigFromCodebase() {
    Serial.println("Loading fallback configuration from SPIFFS...");
    
    // Try to read from SPIFFS first
    if (readProjectConfigJson()) {
        Serial.println("Fallback configuration loaded successfully from SPIFFS");
        return true;
    }
    
    Serial.println("ERROR: Failed to load project config.json from SPIFFS!");
    Serial.println("Please ensure:");
    Serial.println("1. Project config.json file exists in the project root");
    Serial.println("2. SPIFFS filesystem has been uploaded with the config file");
    Serial.println("3. Or insert SD card with valid config.json");
    Serial.println("System cannot continue without configuration.");
    return false;
}

bool Configuration::readProjectConfigJson() {
    // Initialize SPIFFS if not already initialized
    if (!SPIFFS.begin()) {
        Serial.println("SPIFFS initialization failed, trying to format...");
        
        // Try to format SPIFFS and initialize again
        if (!SPIFFS.begin(true)) { // true = format if mount fails
            Serial.println("SPIFFS format and initialization failed");
            return false;
        }
        Serial.println("SPIFFS formatted and initialized successfully");
    }
    
    // Try to open the config.json file from SPIFFS
    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile) {
        Serial.println("Failed to open config.json from SPIFFS");
        Serial.println("Please upload the project's config.json to SPIFFS using:");
        Serial.println("python -m platformio run --target uploadfs");
        return false;
    }
    
    // Read the file content
    String configContent = configFile.readString();
    configFile.close();
    
    if (configContent.length() == 0) {
        Serial.println("Config file is empty");
        return false;
    }
    
    // Parse the JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, configContent);
    if (error) {
        Serial.print("Failed to parse config.json from SPIFFS: ");
        Serial.println(error.c_str());
        return false;
    }
      // Load configuration from parsed JSON
    if (!loadConfig(doc.as<JsonObject>())) {
        Serial.println("Failed to load configuration from SPIFFS");
        return false;
    }
    
    return true;
}

// Removed hardcoded configuration functions - now uses SPIFFS fallback only

bool Configuration::loadConfig(const JsonObject& config) {
    if (config.isNull()) {
        Serial.println("Config is null");
        return false;
    }
    
    // Parse WiFi configuration
    if (!config["wifi"].isNull()) {
        parseWiFiConfig(config["wifi"]);
    }
      // Parse MQTTS configuration (SSL) - prioritize secure MQTT over regular MQTT
    if (!config["mqtts"].isNull()) {
        parseMQTTConfig(config["mqtts"]);
    } else if (!config["mqtt"].isNull()) {
        // Fallback to regular MQTT if MQTTS is not available
        parseMQTTConfig(config["mqtt"]);
    }
    
    // Parse project configuration
    if (!config["project"].isNull()) {
        parseProjectConfig(config["project"]);
    }
    
    // Parse custom WiFi configuration
    if (!config["custom_wifi"].isNull()) {
        parseCustomWifiConfig(config["custom_wifi"]);
    }
    
    // Parse custom MQTT configuration
    if (!config["custom_mqtt"].isNull()) {
        parseCustomMqttConfig(config["custom_mqtt"]);
    }
      // Parse MQTT throttling configuration
    if (!config["mqtt_throttling"].isNull()) {
        parseMqttThrottlingConfig(config["mqtt_throttling"]);
    }
    
    // Parse main program configuration
    if (!config["main_program"].isNull()) {
        parseMainProgramConfig(config["main_program"]);
    }
    
    // Parse climate controller configuration
    if (!config["climate_controller"].isNull()) {
        parseClimateControllerConfig(config["climate_controller"]);
    }
    
    // Parse display configuration
    if (!config["display"].isNull()) {
        parseDisplayConfig(config["display"]);
    }
    
    // Parse system configuration
    if (!config["system"].isNull()) {
        parseSystemConfig(config["system"]);
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

std::vector<Device*> Configuration::initializeDevices(std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, DS3231rtc*& rtc) {
    std::vector<Device*> devices;

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
    
    Serial.print("Total devices in JSON: ");
    Serial.println(devicesConfig.size());
    
    static int deviceIndex = 0;
      // Iterate through each device configuration in the JSON
    for (JsonPair devicePair : devicesConfig) {
        String deviceKey = devicePair.key().c_str();
        JsonObject deviceConfig = devicePair.value().as<JsonObject>();
        
        Serial.print("Processing device: ");
        Serial.println(deviceKey);
        
        if (deviceConfig.isNull()) {
            Serial.print("Skipping invalid device config for: ");
            Serial.println(deviceKey);
            continue;
        }
        
        // Extract device configuration
        String deviceType = deviceConfig["Type"].as<String>();
        String deviceTypeNumber = deviceConfig["TypeNumber"].as<String>();
        String addressStr = deviceConfig["Address"].as<String>();
        String deviceMode = deviceConfig["Mode"].as<String>();
        
        // Extract label with modern ArduinoJson approach
        String deviceLabel = "";
        JsonVariant labelVariant = deviceConfig["Label"];
        if (!labelVariant.isNull() && labelVariant.is<const char*>()) {
            deviceLabel = labelVariant.as<String>();
        }
          // Configuration loaded
        
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
          // Find matching devices
          // Find devices of the same type in scan results
        std::vector<std::pair<uint8_t, uint8_t>> matchingDevices;
        for (const auto& scannedDevice : scannedDevices) {
            if (scannedDevice.first == expectedAddress) {
                matchingDevices.push_back(scannedDevice);
            }
        }
        
        Serial.print("Found ");
        Serial.print(matchingDevices.size());
        Serial.print(" matching devices for type ");
        Serial.print(deviceTypeNumber);
        Serial.print(" at address 0x");
        Serial.print(expectedAddress, HEX);
        Serial.print(", need index ");
        Serial.println(typeIndex);
          // Devices located
        
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
        }        // Get the device at the specified positional index
        uint8_t deviceAddress = matchingDevices[typeIndex].first;
        uint8_t tcaPort = matchingDevices[typeIndex].second;
        
        Serial.print("Creating ");
        Serial.print(deviceKey);
        Serial.print(" (0x");
        Serial.print(deviceAddress, HEX);
        Serial.print(", Port ");
        Serial.print(tcaPort);
        Serial.println(")");
        
        // Parse channels and thresholds
        std::map<String, String> channelNames;
        std::map<String, float> channelThresholds;
        
        JsonObject channels = deviceConfig["Channels"];
        if (!channels.isNull()) {
            for (JsonPair channelPair : channels) {
                String channelKey = channelPair.key().c_str();
                JsonObject channelConfig = channelPair.value().as<JsonObject>();
                  if (!channelConfig.isNull()) {
                    String channelName = channelConfig["Name"].as<String>();
                    if (channelName.isEmpty()) {
                        channelName = channelKey;
                    }
                    float threshold = channelConfig["Threshold"].as<float>();
                    if (threshold == 0.0f) {
                        threshold = 1.0f;
                    }
                    
                    channelNames[channelKey] = channelName;
                    channelThresholds[channelKey] = threshold;
                }
            }
        }
        
        // Create the device
        Device* createdDevice = DeviceRegistry::getInstance().createDeviceWithThresholds(
            &Wire, deviceType, deviceTypeNumber, deviceAddress, tcaPort, 
            channelThresholds, channelNames, deviceIndex, deviceMode        );
        
        if (createdDevice != nullptr) {
            // Apply the label from JSON configuration
            createdDevice->setDeviceLabel(deviceLabel);
            
            // Handle special case for RTC
            if (deviceType.equalsIgnoreCase("RTC") && deviceTypeNumber.equalsIgnoreCase("DS3231")) {
                rtc = static_cast<DS3231rtc*>(createdDevice);
                Serial.println("RTC assigned to global reference");
            }
            
            devices.push_back(createdDevice);
            deviceIndex++;
            
            Serial.println("OK");
        } else {
            Serial.println("FAILED");
        }
    }    Serial.print("Created ");
    Serial.print(devices.size());
    Serial.println(" devices");
    
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
        
        // Wait for any pending interrupts to complete
        yield();
        
        // ESP32's DRAM region is typically in this range
        if (deviceAddr >= 0x3F800000 && deviceAddr <= 0x3FFFFFFF) {
            return true;
        }
    } catch (...) {
        Serial.println("Exception while accessing device memory");
        return false;
    }
    
    // If we got here, object is probably valid
    return true;
}

void Configuration::initializeEachDevice(std::vector<Device*>& devices) {
    Serial.println("Initializing devices...");
    
    // Remove any null devices that might have been accidentally added
    auto it = std::remove_if(devices.begin(), devices.end(), [](Device* d) { return d == nullptr; });
    if (it != devices.end()) {
        devices.erase(it, devices.end());
    }
    
    if (devices.empty()) {
        Serial.println("No devices to initialize!");
        return;
    }
    
    for (size_t i = 0; i < devices.size(); i++) {
        Device* device = devices[i];
        Serial.println(" ---");
          // Check for null pointers
        if (!device) {
            continue;
        }
        
        // Check if device pointer is in valid memory range
        if ((uint32_t)device < 0x3F800000 || (uint32_t)device > 0x3FFFFFFF) {
            continue;
        }

        // Initialize device
        bool success = false;
        try {
            if (device->getType().equalsIgnoreCase("PCF8574GPIO")) {
                I2CHandler::selectTCA(device->getTCAChannel());
                Wire.beginTransmission(device->getI2CAddress());
                int error = Wire.endTransmission();
                
                if (error == 0) {
                    // Set all pins to OUTPUT mode
                    Wire.beginTransmission(device->getI2CAddress());
                    Wire.write(0x00); // Set all pins to LOW (off)
                    Wire.endTransmission();
                    success = true;
                }            } 
            else if (device->getType().equalsIgnoreCase("GP8403dac")) {
                I2CHandler::selectTCA(device->getTCAChannel());
                Wire.beginTransmission(device->getI2CAddress());
                int error = Wire.endTransmission();
                
                if (error == 0) {
                    success = true;
                }
            }            else if (device->getType().equalsIgnoreCase("SHTSensor") || 
                     device->getType().equalsIgnoreCase("Sensor")) {
                I2CHandler::selectTCA(device->getTCAChannel());
                Wire.beginTransmission(device->getI2CAddress());
                int error = Wire.endTransmission();
                
                if (error == 0) {
                    success = true;
                }            }
            else if (device->getType().equalsIgnoreCase("Display") || 
                     device->getType().equalsIgnoreCase("DFR0554Display")) {
                I2CHandler::selectTCA(device->getTCAChannel());
                Wire.beginTransmission(device->getI2CAddress());
                int error = Wire.endTransmission();
                  if (error == 0) {
                    success = true;
                }
            }
            
            if (success) {
                device->begin();
            }
        } catch (...) {
            success = false;
        }
        
    delay(100);
    }
    
    Serial.println("Device initialization complete");
}

// Parse configuration sections
void Configuration::parseWiFiConfig(const JsonObject& config) {
    wifiConfig["ssid"] = config["ssid"].as<String>();
    wifiConfig["password"] = config["password"].as<String>();
}

void Configuration::parseMQTTConfig(const JsonObject& config) {
    mqttConfig["server"] = config["server"].as<String>();
    mqttConfig["port"] = String(config["port"].as<int>());
    mqttConfig["username"] = config["username"].as<String>();
    mqttConfig["password"] = config["password"].as<String>();
    mqttConfig["token"] = config["token"].as<String>();
}

void Configuration::parseProjectConfig(const JsonObject& config) {
    projectConfig["number"] = config["number"].as<String>();
    projectConfig["showcase_id"] = config["showcase_id"].as<String>();
    projectConfig["device"] = config["device"].as<String>();
    projectConfig["timezone"] = config["timezone"].as<String>();
}

void Configuration::parseCustomWifiConfig(const JsonObject& config) {
    customWifiConfig["enable"] = config["enable"].as<bool>() ? "1" : "0";
    customWifiConfig["ssid"] = config["ssid"].as<String>();
    customWifiConfig["password"] = config["password"].as<String>();
}

void Configuration::parseCustomMqttConfig(const JsonObject& config) {
    customMqttConfig["enable"] = config["enable"].as<bool>() ? "1" : "0";
    customMqttConfig["server"] = config["server"].as<String>();
    customMqttConfig["port"] = String(config["port"].as<int>());
    customMqttConfig["token"] = config["token"].as<String>();
}

void Configuration::parseMqttThrottlingConfig(const JsonObject& config) {
    mqttThrottlingConfig["enabled"] = config["enabled"].as<bool>() ? "1" : "0";
    mqttThrottlingConfig["interval_ms"] = String(config["interval_ms"].as<int>());
}

void Configuration::parseMainProgramConfig(const JsonObject& config) {
    std::map<String, String> mainProgramConfig;
    mainProgramConfig["status_update_interval_ms"] = String(config["status_update_interval_ms"].as<int>());
    mainProgramConfig["time_fetch_interval_ms"] = String(config["time_fetch_interval_ms"].as<int>());
    mainProgramConfig["connection_retry_interval_ms"] = String(config["connection_retry_interval_ms"].as<int>());
    mainProgramConfig["startup_delay_ms"] = String(config["startup_delay_ms"].as<int>());
    mainProgramConfig["wifi_connection_timeout_ms"] = String(config["wifi_connection_timeout_ms"].as<int>());
    mainProgramConfig["button_press_timeout_ms"] = String(config["button_press_timeout_ms"].as<int>());
    
    // Store in systemConfig for compatibility with existing getters
    systemConfig["status_update_interval_ms"] = mainProgramConfig["status_update_interval_ms"];
    systemConfig["time_fetch_interval_ms"] = mainProgramConfig["time_fetch_interval_ms"];
    systemConfig["connection_retry_interval_ms"] = mainProgramConfig["connection_retry_interval_ms"];
    systemConfig["startup_delay_ms"] = mainProgramConfig["startup_delay_ms"];
    systemConfig["wifi_connection_timeout_ms"] = mainProgramConfig["wifi_connection_timeout_ms"];
    systemConfig["button_press_timeout_ms"] = mainProgramConfig["button_press_timeout_ms"];
}

void Configuration::parseClimateControllerConfig(const JsonObject& config) {
    climateControllerConfig["enabled"] = config["enabled"].as<bool>() ? "1" : "0";
    climateControllerConfig["temperature_setpoint"] = String(config["temperature_setpoint"].as<double>());
    climateControllerConfig["humidity_setpoint"] = String(config["humidity_setpoint"].as<double>());
    climateControllerConfig["climate_mode"] = config["climate_mode"].as<String>();
    climateControllerConfig["humidity_mode"] = config["humidity_mode"].as<String>();
    climateControllerConfig["auto_fan_control"] = config["auto_fan_control"].as<bool>() ? "1" : "0";
    climateControllerConfig["update_interval_ms"] = String(config["update_interval_ms"].as<int>());
      // Parse safety limits
    if (config["safety_limits"].is<JsonObject>()) {
        JsonObject safetyLimits = config["safety_limits"];
        climateControllerConfig["max_temperature"] = String(safetyLimits["max_temperature"].as<double>());
        climateControllerConfig["min_temperature"] = String(safetyLimits["min_temperature"].as<double>());
        climateControllerConfig["max_humidity"] = String(safetyLimits["max_humidity"].as<double>());
        climateControllerConfig["min_humidity"] = String(safetyLimits["min_humidity"].as<double>());
    } else {
        // Default values if safety_limits section is missing
        climateControllerConfig["max_temperature"] = String(35.0);
        climateControllerConfig["min_temperature"] = String(10.0);
        climateControllerConfig["max_humidity"] = String(80.0);
        climateControllerConfig["min_humidity"] = String(20.0);
    }
      // Parse PID parameters
    if (config["pid_parameters"].is<JsonObject>()) {
        JsonObject pidParams = config["pid_parameters"];
          // Temperature PID
        if (pidParams["temperature"].is<JsonObject>()) {
            JsonObject tempPid = pidParams["temperature"];
            climateControllerConfig["temp_kp"] = String(tempPid["kp"].as<double>());
            climateControllerConfig["temp_ki"] = String(tempPid["ki"].as<double>());
            climateControllerConfig["temp_kd"] = String(tempPid["kd"].as<double>());
        } else {
            climateControllerConfig["temp_kp"] = String(2.0);
            climateControllerConfig["temp_ki"] = String(0.5);
            climateControllerConfig["temp_kd"] = String(0.1);
        }    } else {
        // Default PID values if pid_parameters section is missing
        climateControllerConfig["temp_kp"] = String(2.0);
        climateControllerConfig["temp_ki"] = String(0.5);
        climateControllerConfig["temp_kd"] = String(0.1);
    }
      // Parse control parameters
    if (config["control_parameters"].is<JsonObject>()) {
        JsonObject controlParams = config["control_parameters"];
        climateControllerConfig["temperature_hysteresis"] = String(controlParams["temperature_hysteresis"].as<double>());
        climateControllerConfig["humidity_hysteresis"] = String(controlParams["humidity_hysteresis"].as<double>());
    } else {
        // Default values if control_parameters section is missing
        climateControllerConfig["temperature_hysteresis"] = String(0.1);
        climateControllerConfig["humidity_hysteresis"] = String(0.5);
    }
}

void Configuration::parseDisplayConfig(const JsonObject& config) {
    displayConfig["update_interval_ms"] = String(config["update_interval_ms"].as<int>());
}

void Configuration::parseSystemConfig(const JsonObject& config) {
    systemConfig["status_update_interval_ms"] = String(config["status_update_interval_ms"].as<int>());
    systemConfig["time_fetch_interval_ms"] = String(config["time_fetch_interval_ms"].as<int>());
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

// Custom WiFi configuration getters
bool Configuration::isCustomWifiEnabled() {
    return customWifiConfig["enable"] == "true" || customWifiConfig["enable"] == "1";
}

String Configuration::getCustomWifiSSID() {
    return customWifiConfig["ssid"];
}

String Configuration::getCustomWifiPassword() {
    return customWifiConfig["password"];
}

// Custom MQTT configuration getters
bool Configuration::isCustomMqttEnabled() {
    return customMqttConfig["enable"] == "true" || customMqttConfig["enable"] == "1";
}

String Configuration::getCustomMqttServer() {
    return customMqttConfig["server"];
}

int Configuration::getCustomMqttPort() {
    return customMqttConfig["port"].toInt();
}

String Configuration::getCustomMqttToken() {
    return customMqttConfig["token"];
}

// MQTT Throttling configuration getters
bool Configuration::isMqttThrottlingEnabled() {
    return mqttThrottlingConfig["enabled"] == "true" || mqttThrottlingConfig["enabled"] == "1";
}

unsigned long Configuration::getMqttThrottlingInterval() {
    return mqttThrottlingConfig["interval_ms"].toInt();
}

// Climate Controller configuration getters
bool Configuration::isClimateControllerEnabled() {
    return climateControllerConfig["enabled"] == "true" || climateControllerConfig["enabled"] == "1";
}

float Configuration::getClimateTemperatureSetpoint() {
    return climateControllerConfig["temperature_setpoint"].toFloat();
}

float Configuration::getClimateHumiditySetpoint() {
    return climateControllerConfig["humidity_setpoint"].toFloat();
}

String Configuration::getClimateMode() {
    return climateControllerConfig["climate_mode"];
}

String Configuration::getHumidityMode() {
    return climateControllerConfig["humidity_mode"];
}

bool Configuration::isAutoFanControlEnabled() {
    return climateControllerConfig["auto_fan_control"] == "true" || climateControllerConfig["auto_fan_control"] == "1";
}

unsigned long Configuration::getClimateUpdateInterval() {
    String intervalStr = climateControllerConfig["update_interval_ms"];
    unsigned long interval = intervalStr.toInt();
    
    // Safety check - if the value is unreasonable, use default
    if (interval == 0 || interval > 3600000) { // Max 1 hour
        return 1000;
    }
    
    return interval;
}

// Climate safety limits
float Configuration::getMaxTemperature() {
    return climateControllerConfig["max_temperature"].toFloat();
}

float Configuration::getMinTemperature() {
    return climateControllerConfig["min_temperature"].toFloat();
}

float Configuration::getMaxHumidity() {
    return climateControllerConfig["max_humidity"].toFloat();
}

float Configuration::getMinHumidity() {
    return climateControllerConfig["min_humidity"].toFloat();
}

// Temperature PID parameters
float Configuration::getTemperatureKp() {
    return climateControllerConfig["temp_kp"].toFloat();
}

float Configuration::getTemperatureKi() {
    return climateControllerConfig["temp_ki"].toFloat();
}

float Configuration::getTemperatureKd() {
    return climateControllerConfig["temp_kd"].toFloat();
}

// Control parameters
float Configuration::getTemperatureHysteresis() {
    return climateControllerConfig["temperature_hysteresis"].toFloat();
}

float Configuration::getHumidityHysteresis() {
    return climateControllerConfig["humidity_hysteresis"].toFloat();
}

// Display configuration getters
unsigned long Configuration::getDisplayUpdateInterval() {
    return displayConfig["update_interval_ms"].toInt();
}

// System configuration getters
unsigned long Configuration::getStatusUpdateInterval() {
    return systemConfig["status_update_interval_ms"].toInt();
}

unsigned long Configuration::getTimeFetchInterval() {
    return systemConfig["time_fetch_interval_ms"].toInt();
}

unsigned long Configuration::getConnectionRetryInterval() {
    return systemConfig["connection_retry_interval_ms"].toInt();
}

unsigned long Configuration::getStartupDelay() {
    return systemConfig["startup_delay_ms"].toInt();
}

unsigned long Configuration::getWifiConnectionTimeout() {
    return systemConfig["wifi_connection_timeout_ms"].toInt();
}

unsigned long Configuration::getButtonPressTimeout() {
    return systemConfig["button_press_timeout_ms"].toInt();
}