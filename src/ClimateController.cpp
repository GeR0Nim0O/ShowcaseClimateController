#include "ClimateController.h"
#include "DeviceRegistry/DeviceRegistry.h"
#include "../lib/Config/ClimateConfig/ClimateConfig.h"
#include <SD.h>  // Include SD card library for saving to SD card
#include <math.h> // Include math library for dew point calculations

// Static factory method for automatic device discovery and initialization
ClimateController* ClimateController::createFromDeviceRegistry() {
    try {
        // Use DeviceRegistry to get devices instead of manual searching
        DeviceRegistry& registry = DeviceRegistry::getInstance();
          // Get GPIO expander from DeviceRegistry
        PCF8574gpio* gpioExpander = (PCF8574gpio*)registry.getDeviceByType("GPIO", 0);
        
        // If no PCF8574 found, try to use Relay4Ch devices instead
        Relay4Ch* relay1 = nullptr;
        Relay4Ch* relay2 = nullptr;
          if (gpioExpander == nullptr) {
            Serial.println("No PCF8574 GPIO found, trying Relay4Ch devices...");
            relay1 = (Relay4Ch*)registry.getDeviceByType("Relay", 0);
            relay2 = (Relay4Ch*)registry.getDeviceByType("Relay", 1);
            
            Serial.print("Relay1: ");
            Serial.println(relay1 ? "Found" : "Not found");
            Serial.print("Relay2: ");
            Serial.println(relay2 ? "Found" : "Not found");
            
            if (relay1 == nullptr || relay2 == nullptr) {
                Serial.println("ERROR: No suitable GPIO control devices found");
                return nullptr;
            }
            Serial.println("Found Relay4Ch devices for GPIO control");
        }
          // Get temperature/humidity sensor from DeviceRegistry - specifically look for Interior labeled sensor
        SHTsensor* climateTemperatureSensor = (SHTsensor*)registry.getDeviceByTypeAndLabel("TemperatureHumidity", "Interior");
        if (climateTemperatureSensor == nullptr) {
            // Fallback to first available sensor if no Interior labeled sensor found
            climateTemperatureSensor = (SHTsensor*)registry.getDeviceByType("TemperatureHumidity", 0);
            if (climateTemperatureSensor == nullptr) {
                Serial.println("No temperature/humidity sensor found in DeviceRegistry");
                return nullptr;
            }
        }
          // Get DAC from DeviceRegistry - using proper DeviceRegistry access pattern
        GP8403dac* climateDac = (GP8403dac*)registry.getDeviceByType("DAC", 0);
          // Create climate controller if we found the required devices
        if ((gpioExpander != nullptr || (relay1 != nullptr && relay2 != nullptr)) && climateTemperatureSensor != nullptr) {
              try {
                ClimateController* controller = nullptr;
                
                if (gpioExpander != nullptr) {
                    // Use PCF8574 GPIO expander
                    controller = new ClimateController(gpioExpander, climateTemperatureSensor, climateDac);
                } else {
                    // Use Relay4Ch devices
                    controller = new ClimateController(relay1, relay2, climateTemperatureSensor, climateDac);
                }
                
                if (controller != nullptr) {
                    if (controller->begin()) {
                        Serial.println("Climate controller initialized successfully");
                        
                        // Enable automatic fan control by default
                        controller->setAutoFanControl(true);
                          // Look for radiator sensor for dew point compensation
                        SHTsensor* radiatorSensor = (SHTsensor*)registry.getDeviceByTypeAndLabel("TemperatureHumidity", "Radiator");
                        if (radiatorSensor != nullptr) {
                            controller->setRadiatorSensor(radiatorSensor);
                        }
                        
                        return controller;
                    } else {
                        Serial.println("Failed to initialize climate controller");
                        delete controller;
                        return nullptr;
                    }
                } else {
                    Serial.println("Failed to allocate climate controller");
                    return nullptr;
                }
            } catch (...) {
                Serial.println("Exception during climate controller initialization");
                return nullptr;
            }
        } else {
            Serial.println("Could not find required devices for climate controller");
            return nullptr;
        }
    } catch (...) {
        Serial.println("Exception during device discovery for climate controller");
        return nullptr;
    }
}

ClimateController::ClimateController(PCF8574gpio* gpioExpander, SHTsensor* tempHumSensor, GP8403dac* dac)
    : gpio(nullptr), relay1(nullptr), relay2(nullptr), sensor(nullptr), dac(nullptr),
      temperatureSetpoint(0.0), humiditySetpoint(0.0),
      currentTemperature(0.0), currentHumidity(0.0),
      temperatureControlEnabled(true), humidityControlEnabled(true),
      heatingActive(false), coolingActive(false), 
      humidifyingActive(false), dehumidifyingActive(false),
      tempControlEnabled(false),      fanInteriorActive(false), fanExteriorActive(false),  // Initialize fan states
      autoFanControlEnabled(true),                         // Enable auto fan control by default      lastUpdate(0), updateInterval(500), lastGpioRefresh(0),  // Use default value, will be updated from ClimateConfig in begin()
      heatingPower(0.0), coolingPower(0.0),
      humidifierPower(0.0), dehumidifierPower(0.0),
      temperaturePID(nullptr),      temperatureAutoTuner(nullptr),
      temperatureAutoTuning(false),
      autoTuneSetpoint(0.0), autoTuneOutputStep(0.0), autoTuneStartTime(0),
      currentAutoTuneType(AutoTuneType::NORMAL), expectedAutoTuneDuration(0),      // Will be loaded from ClimateConfig
      tempInput(0.0), tempOutput(0.0), tempSetpoint(0.0),
      lastStatusPrint(0), statusPrintInterval(10000), // Print status every 10 seconds
      lastPrintedTemperature(0.0), lastPrintedHumidity(0.0),
      lastPrintedHeatingActive(false), lastPrintedCoolingActive(false),
      lastPrintedHumidifyingActive(false), lastPrintedDehumidifyingActive(false),
      lastPrintedFanInteriorActive(false),      lastPrintedFanExteriorActive(false),
      lastPrintedTemperatureControlEnabled(true), lastPrintedHumidityControlEnabled(true),
      temperatureThreshold(Configuration::getTemperatureHysteresis()), humidityThreshold(Configuration::getHumidityHysteresis()) {
      // Safely assign the device pointers
    this->gpio = gpioExpander;
    this->sensor = tempHumSensor;
    this->dac = dac;
    
    // Initialize dew point compensation
    this->radiatorSensor = nullptr;
    this->dewPoint = 0.0;
    this->currentRadiatorTemperature = 0.0;
    this->minAllowedCoolingTemperature = 0.0;
    this->lastDewPointUpdate = 0;
      // Pin mappings will be loaded from config.json only - no hardcoded defaults
    pinFanExterior = 255;      // Invalid pin to force config loading
    pinFanInterior = 255;      // Invalid pin to force config loading
    pinHumidify = 255;         // Invalid pin to force config loading
    pinDehumidify = 255;       // Invalid pin to force config loading
    pinTemperatureEnable = 255; // Invalid pin to force config loading
    pinTemperatureCool = 255;   // Invalid pin to force config loading
    pinTemperatureHeat = 255;   // Invalid pin to force config loading// Initialize PID controllers - with safety checks
    try {
        temperaturePID = new PID(&tempInput, &tempOutput, &tempSetpoint, 
                                Configuration::getTemperatureKp(), 
                                Configuration::getTemperatureKi(), 
                                Configuration::getTemperatureKd(), DIRECT);
        
        if (temperaturePID != nullptr) {
            temperaturePID->SetMode(AUTOMATIC);
            temperaturePID->SetOutputLimits(-100, 100); // -100 = full cooling, +100 = full heating
        }
    }catch (...) {
        // Clean up if an exception occurs
        if (temperaturePID != nullptr) {
            delete temperaturePID;
            temperaturePID = nullptr;
        }
    }
}

// New constructor for Relay4Ch devices
ClimateController::ClimateController(Relay4Ch* relay1, Relay4Ch* relay2, SHTsensor* tempHumSensor, GP8403dac* dac)
    : gpio(nullptr), relay1(nullptr), relay2(nullptr), sensor(nullptr), dac(nullptr),
      temperatureSetpoint(0.0), humiditySetpoint(0.0),
      currentTemperature(0.0), currentHumidity(0.0),
      temperatureControlEnabled(true), humidityControlEnabled(true),
      heatingActive(false), coolingActive(false), 
      humidifyingActive(false), dehumidifyingActive(false),
      tempControlEnabled(false),
      fanInteriorActive(false), fanExteriorActive(false),      autoFanControlEnabled(true),
      lastUpdate(0), updateInterval(500), lastGpioRefresh(0),
      heatingPower(0.0), coolingPower(0.0),
      humidifierPower(0.0), dehumidifierPower(0.0),
      temperaturePID(nullptr),
      temperatureAutoTuner(nullptr),
      temperatureAutoTuning(false),
      autoTuneSetpoint(0.0), autoTuneOutputStep(0.0), autoTuneStartTime(0),
      currentAutoTuneType(AutoTuneType::NORMAL), expectedAutoTuneDuration(0),
      tempInput(0.0), tempOutput(0.0), tempSetpoint(0.0),
      lastStatusPrint(0), statusPrintInterval(10000),
      lastPrintedTemperature(0.0), lastPrintedHumidity(0.0),
      lastPrintedHeatingActive(false), lastPrintedCoolingActive(false),
      lastPrintedHumidifyingActive(false), lastPrintedDehumidifyingActive(false),
      lastPrintedFanInteriorActive(false), lastPrintedFanExteriorActive(false),
      lastPrintedTemperatureControlEnabled(true), lastPrintedHumidityControlEnabled(true),
      temperatureThreshold(Configuration::getTemperatureHysteresis()), humidityThreshold(Configuration::getHumidityHysteresis()) {
      
    // Safely assign the device pointers
    this->relay1 = relay1;
    this->relay2 = relay2;
    this->sensor = tempHumSensor;
    this->dac = dac;
    
    // Initialize dew point compensation
    this->radiatorSensor = nullptr;
    this->dewPoint = 0.0;
    this->currentRadiatorTemperature = 0.0;
    this->minAllowedCoolingTemperature = 0.0;
    this->lastDewPointUpdate = 0;
      // Pin mappings will be loaded from config.json only - no hardcoded defaults  
    pinHumidify = 255;         // Invalid pin to force config loading
    pinDehumidify = 255;       // Invalid pin to force config loading
    pinFanInterior = 255;      // Invalid pin to force config loading
    pinFanExterior = 255;      // Invalid pin to force config loading
    
    pinTemperatureEnable = 255; // Invalid pin to force config loading
    pinTemperatureCool = 255;   // Invalid pin to force config loading
    pinTemperatureHeat = 255;   // Invalid pin to force config loading
    
    Serial.println("ClimateController: Using Relay4Ch devices for GPIO control");
    
    // Initialize PID controllers - with safety checks
    try {
        temperaturePID = new PID(&tempInput, &tempOutput, &tempSetpoint, 
                                Configuration::getTemperatureKp(), 
                                Configuration::getTemperatureKi(), 
                                Configuration::getTemperatureKd(), DIRECT);
        
        if (temperaturePID != nullptr) {
            temperaturePID->SetMode(AUTOMATIC);
            temperaturePID->SetOutputLimits(-100, 100); // -100 = full cooling, +100 = full heating
        }
    } catch (...) {
        // Clean up if an exception occurs
        if (temperaturePID != nullptr) {
            delete temperaturePID;
            temperaturePID = nullptr;
        }
    }
}

ClimateController::~ClimateController() {
    // Clean up PID controllers
    if (temperaturePID != nullptr) {
        delete temperaturePID;
        temperaturePID = nullptr;
    }
    
    // Clean up AutoTune controllers
    if (temperatureAutoTuner != nullptr) {
        delete temperatureAutoTuner;
        temperatureAutoTuner = nullptr;
    }
}

bool ClimateController::begin() {
    // Load updateInterval and setpoints from ClimateConfig
    ClimateConfig& climateConfig = ClimateConfig::getInstance();
    updateInterval = climateConfig.getUpdateInterval();
    temperatureSetpoint = climateConfig.getTemperatureSetpoint();
    humiditySetpoint = climateConfig.getHumiditySetpoint();      // Initialize DAC if available
    if (dac != nullptr) {
        // Check if DAC is already initialized (to avoid double initialization)
        if (dac->isInitialized()) {
            // Set DAC to 5V during setup to indicate system is initializing
            dac->setChannelVoltage(0, 5.0);
        } else {
            if (dac->begin()) {
                // Set DAC to 5V during setup to indicate system is initializing
                dac->setChannelVoltage(0, 5.0);
            }
        }
    }
      // Initialize pin mappings
    initializePinMappings();
    
    // Initialize GPIO devices
    if (gpio != nullptr) {
        // PCF8574 GPIO expander
        gpio->forceOutputMode();
        
        // Initialize ALL pins to LOW/false state
        for (int pin = 0; pin < 8; pin++) {
            gpio->writePin(pin, false);
        }
        
        // Force final state to 0x00
        gpio->writeByte(0x00);
        Serial.println("ClimateController: PCF8574 GPIO initialized");    } else if (relay1 != nullptr && relay2 != nullptr) {
        // Relay4Ch devices - use M5Stack-compatible initialization
        Serial.println("Initializing Relay4Ch devices...");
          // Call begin() to establish connection and initialize
        bool relay1Connected = relay1->begin();
        bool relay2Connected = relay2->begin();
        
        if (relay1Connected && relay2Connected) {
            // Check if initialization was successful
            if (relay1->isInitialized() && relay2->isInitialized()) {
                // Initialize all relays to OFF
                relay1->relayAll(false);
                relay2->relayAll(false);
                Serial.println("ClimateController: Relay4Ch devices initialized successfully");
            } else {
                Serial.println("ERROR: Relay4Ch devices failed initialization sequence");
                return false;
            }
        } else {
            Serial.print("ERROR: Relay4Ch connection failed - Relay1: ");
            Serial.print(relay1Connected ? "OK" : "FAIL");
            Serial.print(", Relay2: ");
            Serial.println(relay2Connected ? "OK" : "FAIL");
            return false;
        }
    } else {
        Serial.println("ERROR: No GPIO control devices available");
        return false;
    }
    
    return true;
}

void ClimateController::update() {    unsigned long currentTime = millis();
    
    if (currentTime - lastUpdate >= updateInterval) {
        updateSensorReadings();
        
        // Store previous states to avoid unnecessary writes
        static bool lastTempControlEnabled = false;
        static bool lastHeatingActive = false;
        static bool lastCoolingActive = false;
        static bool lastHumidifyingActive = false;
        static bool lastDehumidifyingActive = false;
        static bool lastFanInteriorActive = false;       // Add fan state tracking
        static bool lastFanExteriorActive = false;       // Add fan state tracking
        
        // Update AutoTune if active
        updateAutoTune();
        
        // Update dew point compensation if enabled
        updateDewPointCompensation();
        
        // Update control logic (normal PID or AutoTune output)
        updateTemperatureControl();
        updateHumidityControl();
        updateFanControl();  // Add fan control update
        
        // Only apply controls if states actually changed
        bool tempStateChanged = (tempControlEnabled != lastTempControlEnabled) ||
                               (heatingActive != lastHeatingActive) ||
                               (coolingActive != lastCoolingActive);
                               
        bool humidityStateChanged = (humidifyingActive != lastHumidifyingActive) ||
                                  (dehumidifyingActive != lastDehumidifyingActive);
          bool fanStateChanged = (fanInteriorActive != lastFanInteriorActive) ||
                              (fanExteriorActive != lastFanExteriorActive);
        
        if (tempStateChanged) {
            applyTemperatureControl();
            lastTempControlEnabled = tempControlEnabled;
            lastHeatingActive = heatingActive;
            lastCoolingActive = coolingActive;
        }
        
        if (humidityStateChanged) {
            applyHumidityControl();
            lastHumidifyingActive = humidifyingActive;
            lastDehumidifyingActive = dehumidifyingActive;
        }
        
        if (fanStateChanged) {
            applyFanControl();
            lastFanInteriorActive = fanInteriorActive;
            lastFanExteriorActive = fanExteriorActive;
        }
        applyDACControls(); // Apply DAC controls        // NEW: Periodically refresh GPIO state to prevent drift
        if (currentTime - lastGpioRefresh >= 10000) { // Every 10 seconds
            if (gpio != nullptr) {
                gpio->refreshOutputState();
            } else if (relay1 != nullptr && relay2 != nullptr) {
                // For Relay4Ch devices, just refresh internal state without re-applying
                relay1->refreshRelayState();
                relay2->refreshRelayState();
            }
            lastGpioRefresh = currentTime;
        }// Enhanced status printing - print on changes or periodically
        printClimateStatusIfChanged();
        
        lastUpdate = currentTime;
    }
}

void ClimateController::updateSensorReadings() {
    if (sensor == nullptr) {
        return;
    }
    
    if (!sensor->isInitialized()) {
        return;
    }
    
    sensor->update();
    
    currentTemperature = sensor->getTemperature();
    currentHumidity = sensor->getHumidity();
      // Update PID inputs
    tempInput = currentTemperature;
    tempSetpoint = temperatureSetpoint;
}

void ClimateController::updateTemperatureControl() {
    if (!temperatureControlEnabled) {
        tempControlEnabled = false;
        heatingActive = false;
        coolingActive = false;
        heatingPower = 0.0;
        coolingPower = 0.0;
        return;
    }
    
    // Skip normal PID computation if AutoTune is active
    // AutoTune will handle tempOutput directly
    if (!temperatureAutoTuning) {
        temperaturePID->Compute();
    }
    
    // AUTO mode logic - can switch between heating and cooling based on PID output
    const float temperatureHysteresis = Configuration::getTemperatureHysteresis();
    if (tempOutput > temperatureHysteresis) {
        heatingActive = true;
        coolingActive = false;
        heatingPower = constrain(tempOutput, 0.0, 100.0);
        coolingPower = 0.0;
    } else if (tempOutput < -temperatureHysteresis) {
        heatingActive = false;
        coolingActive = true;
        heatingPower = 0.0;
        coolingPower = constrain(-tempOutput, 0.0, 100.0);  // Convert negative to positive percentage
        // Apply dew point compensation to cooling power
        coolingPower = limitCoolingOutputForDewPoint(coolingPower);
        // Update cooling active state based on compensated power
        coolingActive = (coolingPower > 0.0);
    } else {
        heatingActive = false;
        coolingActive = false;
        heatingPower = 0.0;
        coolingPower = 0.0;
    }
    tempControlEnabled = (heatingActive || coolingActive);
}

void ClimateController::updateHumidityControl() {
    if (!humidityControlEnabled) {
        humidifyingActive = false;
        dehumidifyingActive = false;
        return;
    }
    
    // Humidity control uses digital on/off devices, not continuous PID control
    // Use hysteresis control instead of PID AutoTune
    const float hysteresis = Configuration::getHumidityHysteresis();
    
    // AUTO mode - can switch between humidifying and dehumidifying
    if (currentHumidity < humiditySetpoint - hysteresis) {
        humidifyingActive = true;
        dehumidifyingActive = false;
    } else if (currentHumidity > humiditySetpoint + hysteresis) {
        humidifyingActive = false;
        dehumidifyingActive = true;
    } else if (currentHumidity >= humiditySetpoint - hysteresis && 
              currentHumidity <= humiditySetpoint + hysteresis) {
        // Within humidity hysteresis of setpoint, turn everything off
        humidifyingActive = false;
        dehumidifyingActive = false;
    }
    // Otherwise keep previous state (hysteresis)
}

void ClimateController::updateFanControl() {
    if (!autoFanControlEnabled) {
        return;
    }
    
    // Determine if ANY climate control is active
    bool tempControlActive = temperatureControlEnabled && 
                            (tempControlEnabled || heatingActive || coolingActive);
    
    bool humidityControlActive = humidityControlEnabled && 
                                (humidifyingActive || dehumidifyingActive);
    
    bool climateControlActive = tempControlActive || humidityControlActive;
    
    // Both fans on when climate control is active
    fanInteriorActive = climateControlActive;
    fanExteriorActive = climateControlActive;  // Both fans on when climate active
}

void ClimateController::applyFanControl() {
    safeWritePin(pinFanInterior, fanInteriorActive);
    safeWritePin(pinFanExterior, fanExteriorActive);
}

void ClimateController::applyTemperatureControl() {
    Serial.printf("ApplyTemperatureControl: Enable=%d (pin %d), Heat=%d (pin %d), Cool=%d (pin %d)\n", 
                  tempControlEnabled, pinTemperatureEnable, 
                  heatingActive, pinTemperatureHeat, 
                  coolingActive, pinTemperatureCool);
    
    safeWritePin(pinTemperatureEnable, tempControlEnabled);
    safeWritePin(pinTemperatureHeat, heatingActive);
    safeWritePin(pinTemperatureCool, coolingActive);
}

void ClimateController::applyHumidityControl() {
    safeWritePin(pinHumidify, humidifyingActive);
    safeWritePin(pinDehumidify, dehumidifyingActive);
}

void ClimateController::applyDACControls() {
    if (!dac || !dac->isInitialized()) {
        return; // No DAC available or not initialized
    }
    
    try {
        // Use channel 0 for temperature control power
        float dacVoltage = 1.0; // Base voltage for 0% power (1V minimum)
        
        if (heatingActive && heatingPower > 0) {
            // Convert heating power (0-100%) to voltage (1-5V)
            // 0% = 1V, 100% = 5V
            dacVoltage = 1.0 + (heatingPower / 100.0) * 4.0;
        } else if (coolingActive && coolingPower > 0) {
            // Convert cooling power (0-100%) to voltage (1-5V)
            // 0% = 1V, 100% = 5V
            dacVoltage = 1.0 + (coolingPower / 100.0) * 4.0;
        }
        
        // Clamp voltage to safe range (1-5V)
        if (dacVoltage > 5.0) dacVoltage = 5.0;
        if (dacVoltage < 1.0) dacVoltage = 1.0;
        
        // Set DAC output voltage
        dac->setChannelVoltage(0, dacVoltage);
        
    } catch (...) {
        // Silently handle DAC errors
    }
}

void ClimateController::setDACSetupMode(bool setupActive) {
    if (!dac || !dac->isInitialized()) {
        return; // No DAC available or not initialized
    }
    
    try {
        if (setupActive) {
            // Set DAC to 5V during setup to indicate system is initializing
            dac->setChannelVoltage(0, 5.0);
            Serial.println("DAC set to 5V - setup mode active");
        } else {
            // Set DAC to 1V when setup is finished (0% power in 1-5V range)
            dac->setChannelVoltage(0, 1.0);
            Serial.println("DAC set to 1V - setup mode finished (0% power)");
        }
    } catch (...) {
        Serial.println("Error setting DAC setup mode");
    }
}

/*
void ClimateController::emergencyShutdown() {
    safeWritePin(pinTemperatureEnable, false);
    safeWritePin(pinTemperatureHeat, false);
    safeWritePin(pinTemperatureCool, false);
    safeWritePin(pinHumidify, false);
    safeWritePin(pinDehumidify, false);
    
    // Also disable DAC output (only channel 0 is used)
    if (dac) {
        try {
            dac->setChannelVoltage(0, 0.0);
        } catch (...) {
            Serial.println("Error turning off DAC in emergency shutdown");
        }
    }
    
    heatingActive = false;
    coolingActive = false;
    humidifyingActive = false;
    dehumidifyingActive = false;
    tempControlEnabled = false;
    heatingPower = 0.0;
    coolingPower = 0.0;
    humidifierPower = 0.0;
    dehumidifierPower = 0.0;
    
    Serial.println("EMERGENCY SHUTDOWN: Safety limits exceeded!");
}
*/

// Pin mapping helper methods
void ClimateController::initializePinMappings() {
    // Load pin mappings from configuration only - no hardcoded defaults
    // Pin mappings must be defined in config.json
      try {
        // Use channel names that match config.json exactly        pinFanExterior = getPinFromChannelName("ExteriorFanRelay");  // RLY3 in Relay4Ch1
        pinFanInterior = getPinFromChannelName("InteriorFanRelay");  // RLY2 in Relay4Ch1  
        pinHumidify = getPinFromChannelName("HumdifyRelay");         // RLY0 in Relay4Ch1
        pinDehumidify = getPinFromChannelName("DehumidifyRelay");    // RLY1 in Relay4Ch1
        pinTemperatureEnable = getPinFromChannelName("EnableTemperatureRelay"); // RLY0 in Relay4Ch2
        pinTemperatureCool = getPinFromChannelName("TemperatureCoolRelay");     // RLY1 in Relay4Ch2
        pinTemperatureHeat = getPinFromChannelName("TemperatureHeatRelay");     // RLY2 in Relay4Ch2
        
        // Debug: Print all pin assignments
        Serial.printf("Pin assignments: Humidify=%d, Dehumidify=%d, FanInt=%d, FanExt=%d\n", 
                     pinHumidify, pinDehumidify, pinFanInterior, pinFanExterior);
        Serial.printf("Temperature pins: Enable=%d, Cool=%d, Heat=%d\n", 
                     pinTemperatureEnable, pinTemperatureCool, pinTemperatureHeat);
        
        // Verify all pins were loaded from config
        if (pinFanExterior == 255 || pinFanInterior == 255 || pinHumidify == 255 || 
            pinDehumidify == 255 || pinTemperatureEnable == 255 || 
            pinTemperatureCool == 255 || pinTemperatureHeat == 255) {
            Serial.println("ERROR: Pin mappings not found in config.json - climate control disabled");
        } else {
            Serial.println("Pin mappings loaded successfully from config.json");
        }
    }
    catch (...) {
        Serial.println("ERROR: Failed to load pin mappings from config.json");
    }
}

uint8_t ClimateController::getPinFromChannelName(const String& channelName) {
    JsonObject devicesConfig = Configuration::getDevicesConfig();
    
    // Configuration must be available - no hardcoded defaults
    if (devicesConfig.isNull() || devicesConfig.size() == 0) {
        Serial.print("ERROR: No devices configuration found for channel: ");
        Serial.println(channelName);
        return 255; // Invalid pin number to indicate error
    }
    
    if (devicesConfig["PCF8574"].is<JsonObject>()) {
        JsonObject pcfConfig = devicesConfig["PCF8574"];
        if (pcfConfig["Channels"].is<JsonObject>()) {
            JsonObject channels = pcfConfig["Channels"];
            
            // Search through all IO channels to find the one with matching name
            for (JsonPair channel : channels) {
                JsonObject channelObj = channel.value();
                if (channelObj["Name"].is<const char*>() && 
                    String(channelObj["Name"].as<const char*>()) == channelName) {
                    // Extract pin number from channel key (e.g., "IO0" -> 0)
                    String channelKey = channel.key().c_str();
                    if (channelKey.startsWith("IO")) {
                        uint8_t pin = channelKey.substring(2).toInt();
                        return pin;
                    }
                }
            }
        }    }
    
    // Check for Relay4Ch devices and search their channels
    for (JsonPair device : devicesConfig) {
        JsonObject deviceObj = device.value();
        if (deviceObj["Type"].is<const char*>() && 
            String(deviceObj["Type"].as<const char*>()) == "Relay" &&
            deviceObj["TypeNumber"].is<const char*>() && 
            String(deviceObj["TypeNumber"].as<const char*>()) == "Relay4Ch") {
            
            if (deviceObj["Channels"].is<JsonObject>()) {
                JsonObject channels = deviceObj["Channels"];
                
                // Search through all relay channels to find the one with matching name
                for (JsonPair channel : channels) {
                    JsonObject channelObj = channel.value();
                    if (channelObj["Name"].is<const char*>() && 
                        String(channelObj["Name"].as<const char*>()) == channelName) {
                        // Extract pin number from channel key (e.g., "RLY0" -> 0)
                        String channelKey = channel.key().c_str();
                        if (channelKey.startsWith("RLY")) {
                            uint8_t pin = channelKey.substring(3).toInt();
                            return pin;
                        }
                    }
                }
            }
        }
    }
    
    // Channel not found in configuration
    Serial.print("ERROR: Channel '");
    Serial.print(channelName);
    Serial.println("' not found in config.json");
    return 255; // Invalid pin number to indicate error
}

// Add this implementation of the safeWritePin method we previously added to the header
bool ClimateController::safeWritePin(uint8_t pin, bool value) {
    Serial.printf("safeWritePin called: pin=%d, value=%s\n", pin, value ? "true" : "false");
    
    // If using PCF8574 GPIO expander
    if (gpio != nullptr && gpio->isInitialized()) {
        Serial.println("Using PCF8574 GPIO device");
        try {
            gpio->writePin(pin, value);
            
            // Verify the write
            uint8_t currentState = gpio->getGPIOState();
            bool actualState = (currentState & (1 << pin)) != 0;
            
            return (actualState == value);
        } catch (...) {
            return false;
        }
    }
    
    // If using Relay4Ch devices
    if (relay1 != nullptr && relay2 != nullptr && 
        relay1->isInitialized() && relay2->isInitialized()) {
        
        Serial.println("Using Relay4Ch devices");
        try {
            // Map pins to correct relay device and channel based on configuration
            if (pin == pinHumidify) {
                // HumdifyRelay → RLY0 on Relay4Ch1
                return relay1->relayWrite(0, value);
            } else if (pin == pinDehumidify) {
                // DehumidifyRelay → RLY1 on Relay4Ch1
                return relay1->relayWrite(1, value);
            } else if (pin == pinFanInterior) {
                // InteriorFanRelay → RLY2 on Relay4Ch1
                return relay1->relayWrite(2, value);
            } else if (pin == pinFanExterior) {
                // ExteriorFanRelay → RLY3 on Relay4Ch1
                return relay1->relayWrite(3, value);            } else if (pin == pinTemperatureEnable) {
                // EnableTemperatureRelay → RLY0 on Relay4Ch2
                Serial.printf("TemperatureEnable: Setting RLY0 on Relay4Ch2 to %s\n", value ? "ON" : "OFF");
                return relay2->relayWrite(0, value);
            } else if (pin == pinTemperatureCool) {
                // TemperatureCoolRelay → RLY1 on Relay4Ch2
                Serial.printf("TemperatureCool: Setting RLY1 on Relay4Ch2 to %s\n", value ? "ON" : "OFF");
                return relay2->relayWrite(1, value);
            } else if (pin == pinTemperatureHeat) {
                // TemperatureHeatRelay → RLY2 on Relay4Ch2
                Serial.printf("TemperatureHeat: Setting RLY2 on Relay4Ch2 to %s\n", value ? "ON" : "OFF");
                return relay2->relayWrite(2, value);
            } else {
                Serial.printf("Unknown pin mapping: pin=%d\n", pin);
            }
            return false;
        } catch (...) {
            Serial.println("Exception in relay control");
            return false;
        }
    }
    
    Serial.println("ERROR: No GPIO control devices available");
    return false;
}

void ClimateController::setFanInterior(bool enable) {
    if (autoFanControlEnabled && !enable) {
        autoFanControlEnabled = false;
    }
    
    fanInteriorActive = enable;
    safeWritePin(pinFanInterior, enable);
}

void ClimateController::setFanExterior(bool enable) {
    if (autoFanControlEnabled && !enable) {
        autoFanControlEnabled = false;
    }
    
    fanExteriorActive = enable;
    safeWritePin(pinFanExterior, enable);
}

void ClimateController::printClimateStatus() {
    Serial.println("\n=== Climate Controller Status ===");
    
    if (!sensor || !sensor->isInitialized()) {
        Serial.println("Climate Controller: Sensor not available");
        Serial.println("==================================");
        return;
    }
      // Print current readings with external sensor info
    Serial.print("Interior Temperature: ");
    Serial.print(getCurrentTemperature(), 2);
    Serial.print("°C (setpoint: ");
    Serial.print(getTemperatureSetpoint(), 2);
    Serial.println("°C)");
    
    Serial.print("Interior Humidity: ");
    Serial.print(getCurrentHumidity(), 1);
    Serial.print("% (setpoint: ");
    Serial.print(getHumiditySetpoint(), 1);
    Serial.println("%)");
      // Get external temperature/humidity from device registry
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    SHTsensor* externalSensor = (SHTsensor*)registry.getDeviceByTypeAndLabel("TemperatureHumidity", "Exterior");
      if (externalSensor != nullptr && externalSensor->isInitialized()) {
        // Get the most recent data from the sensor
        auto externalData = externalSensor->readData();
        
        Serial.print("External Temperature: ");
        Serial.print(externalData["T"].toFloat(), 2);
        Serial.println("°C");
        
        Serial.print("External Humidity: ");
        Serial.print(externalData["H"].toFloat(), 1);
        Serial.println("%");    } else {
        Serial.println("External Conditions: [Sensor not available]");
    }
    
    // Print radiator sensor and dew point compensation information
    if (radiatorSensor != nullptr && radiatorSensor->isInitialized()) {
        Serial.print("Radiator Temperature: ");
        Serial.print(getCurrentRadiatorTemperature(), 2);
        Serial.println("°C");
        
        if (isDewPointCompensationEnabled()) {
            Serial.print("Dew Point: ");
            Serial.print(getDewPoint(), 2);
            Serial.print("°C, Min Cooling Temp: ");
            Serial.print(getMinAllowedCoolingTemperature(), 2);
            Serial.println("°C");
            
            if (getCurrentRadiatorTemperature() <= getMinAllowedCoolingTemperature()) {
                Serial.println("⚠️  Dew Point Protection: COOLING LIMITED");
            } else {
                Serial.println("✓ Dew Point Protection: OK");
            }
        } else {
            Serial.println("Dew Point Compensation: DISABLED");
        }
    } else {
        Serial.println("Radiator Sensor: [Not available - dew point compensation disabled]");
    }
      // Print control status
    Serial.print("Temperature Control: ");
    Serial.println(temperatureControlEnabled ? "ENABLED" : "DISABLED");
    
    Serial.print("Humidity Control: ");
    Serial.println(humidityControlEnabled ? "ENABLED" : "DISABLED");
    
    // Print active controls
    Serial.print("Temperature Control: ");
    if (isHeating()) {
        Serial.print("Heating (");
        Serial.print(getHeatingPower(), 0);
        Serial.println("%)");
    } else if (isCooling()) {
        Serial.print("Cooling (");
        Serial.print(getCoolingPower(), 0);
        Serial.println("%)");
    } else {
        Serial.println("Standby");
    }
    
    Serial.print("Humidity Control: ");
    if (isHumidifying()) {
        Serial.println("Humidifying");
    } else if (isDehumidifying()) {
        Serial.println("Dehumidifying");
    } else {
        Serial.println("Standby");
    }
    
    // Print fan status
    Serial.print("Ventilation: Interior ");
    Serial.print(isFanInteriorOn() ? "ON" : "OFF");
    Serial.print(", Exterior ");
    Serial.print(isFanExteriorOn() ? "ON" : "OFF");    if (isAutoFanControlEnabled()) {
        Serial.println(" (Auto)");
    } else {
        Serial.println(" (Manual)");
    }
      // Print AutoTune status with detailed information
    Serial.println("");
    Serial.println("--- PID Control & AutoTune Status ---");
      // Always show current setpoints
    Serial.print("Current Setpoints: Temp ");
    Serial.print(getTemperatureSetpoint(), 2);
    Serial.print("°C, Humidity ");
    Serial.print(getHumiditySetpoint(), 1);
    Serial.println("%");
      Serial.print("PID AutoTune: ");    if (temperatureAutoTuning) {
        Serial.println("ACTIVE");
          // Show AutoTune progress information with percentage
        Serial.print("  → Target: ");
        Serial.print(autoTuneSetpoint, 2);
        Serial.print("°C, Runtime: ");
        unsigned long autoTuneRuntime = millis() - autoTuneStartTime;
        Serial.print(autoTuneRuntime / 60000);
        Serial.print(":");
        Serial.printf("%02d", (autoTuneRuntime / 1000) % 60);
        
        // Calculate and show progress percentage
        if (expectedAutoTuneDuration > 0) {
            float progressPercentage = (float)autoTuneRuntime / (float)expectedAutoTuneDuration * 100.0;
            if (progressPercentage > 100.0) progressPercentage = 100.0;
            Serial.print(" (");
            Serial.print(progressPercentage, 1);
            Serial.print("%)");
        }
        
        Serial.print(", Output: ");
        Serial.print(abs(tempOutput), 1);
        Serial.print("% (");
        Serial.print(tempOutput > 0 ? "Heat" : "Cool");
        Serial.println(")");
          // Show AutoTune mode
        const char* autoTuneTypeName = "Unknown";
        switch (currentAutoTuneType) {
            case AutoTuneType::NORMAL: autoTuneTypeName = "Normal"; break;
            case AutoTuneType::FAST: autoTuneTypeName = "Fast"; break;
        }
        Serial.print("  → Mode: ");
        Serial.print(autoTuneTypeName);
        Serial.println(" - Analyzing system response, fluctuations expected");
    } else {
        Serial.println("DISABLED");
    }
    
    Serial.println("==================================");
}

// New method to print status only if there are significant changes
void ClimateController::printClimateStatusIfChanged() {
    unsigned long currentTime = millis();
    
    // Check if we should print due to time interval
    bool timeToPrint = (currentTime - lastStatusPrint >= statusPrintInterval);
    
    // Check if there are significant state or sensor changes
    bool stateChanged = hasSignificantStateChange();
    bool sensorChanged = hasSignificantSensorChange();
    
    if (timeToPrint || stateChanged || sensorChanged) {
        Serial.print("[Climate] ");
        if (stateChanged) Serial.print("STATE CHANGE - ");
        if (sensorChanged) Serial.print("SENSOR CHANGE - ");
        if (timeToPrint && !stateChanged && !sensorChanged) Serial.print("PERIODIC UPDATE - ");
        
        printClimateStatus();
        updateStatusPrintTracking();
        lastStatusPrint = currentTime;
    }
}

// Check if there are significant state changes
bool ClimateController::hasSignificantStateChange() {
    return (heatingActive != lastPrintedHeatingActive) ||
           (coolingActive != lastPrintedCoolingActive) ||
           (humidifyingActive != lastPrintedHumidifyingActive) ||
           (dehumidifyingActive != lastPrintedDehumidifyingActive) ||
           (fanInteriorActive != lastPrintedFanInteriorActive) ||
           (fanExteriorActive != lastPrintedFanExteriorActive) ||           (lastPrintedTemperatureControlEnabled != temperatureControlEnabled) ||
           (lastPrintedHumidityControlEnabled != humidityControlEnabled);
}

// Check if there are significant sensor value changes
bool ClimateController::hasSignificantSensorChange() {
    float tempDiff = abs(currentTemperature - lastPrintedTemperature);
    float humDiff = abs(currentHumidity - lastPrintedHumidity);
    
    return (tempDiff >= temperatureThreshold) || (humDiff >= humidityThreshold);
}

// Update tracking variables with current values
void ClimateController::updateStatusPrintTracking() {
    lastPrintedTemperature = currentTemperature;
    lastPrintedHumidity = currentHumidity;
    lastPrintedHeatingActive = heatingActive;
    lastPrintedCoolingActive = coolingActive;
    lastPrintedHumidifyingActive = humidifyingActive;
    lastPrintedDehumidifyingActive = dehumidifyingActive;
    lastPrintedFanInteriorActive = fanInteriorActive;    lastPrintedFanExteriorActive = fanExteriorActive;
    lastPrintedTemperatureControlEnabled = temperatureControlEnabled;
    lastPrintedHumidityControlEnabled = humidityControlEnabled;
}

// Static method for controlled update with timing management
void ClimateController::updateControllerWithTiming(ClimateController* controller) {
    if (controller == nullptr) {
        return; // Exit if climate controller is not available
    }
      // Only update climate controller every 10 seconds to prevent SHT sensor self-heating
    static unsigned long lastClimateUpdate = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastClimateUpdate >= 10000) { // Update every 10 seconds (10% duty cycle)
        controller->update();
        lastClimateUpdate = currentTime;
    }
}

// Configuration method for setting all parameters at once
void ClimateController::configure(float tempSetpoint, float humSetpoint, bool enableTemperatureControl, bool enableHumidityControl) {
    setTemperatureSetpoint(tempSetpoint);
    setHumiditySetpoint(humSetpoint);
    setTemperatureControlEnabled(enableTemperatureControl);
    setHumidityControlEnabled(enableHumidityControl);
    
    Serial.println("Climate controller configured:");
    Serial.print("Temperature setpoint: ");
    Serial.print(tempSetpoint);
    Serial.println("°C");
    Serial.print("Humidity setpoint: ");
    Serial.print(humSetpoint);
    Serial.println("%");
    Serial.print("Temperature control: ");
    Serial.println(enableTemperatureControl ? "ENABLED" : "DISABLED");
    Serial.print("Humidity control: ");
    Serial.println(enableHumidityControl ? "ENABLED" : "DISABLED");
}

// Reload configuration from Configuration class and ClimateConfig
void ClimateController::reloadConfiguration() {
    Serial.println("Reloading climate controller configuration...");
    
    // Try to load from ClimateConfig JSON file first
    ClimateConfig& climateConfig = ClimateConfig::getInstance();
    if (climateConfig.loadFromJsonFile()) {
        Serial.println("Loaded settings from ClimateConfig JSON file");
        
        // Update setpoints from ClimateConfig
        setTemperatureSetpoint(climateConfig.getTemperatureSetpoint());
        setHumiditySetpoint(climateConfig.getHumiditySetpoint());
          // Update control modes - convert from strings to boolean flags
        String climateMode = climateConfig.getClimateMode();
        setTemperatureControlEnabled(climateMode != "OFF");
        
        String humidityMode = climateConfig.getHumidityMode();
        setHumidityControlEnabled(humidityMode != "OFF");
        
        // Update auto fan control
        setAutoFanControl(climateConfig.getAutoFanControl());
        
        // Update update interval from ClimateConfig
        updateInterval = climateConfig.getUpdateInterval();
          // Update PID parameters from ClimateConfig
        if (temperaturePID != nullptr) {
            // Check if we have AutoTune results to use
            if (climateConfig.hasAutoTuneResults()) {
                Serial.println("Using AutoTune results for temperature PID");
                temperaturePID->SetTunings(
                    climateConfig.getAutoTuneKp(),
                    climateConfig.getAutoTuneKi(),
                    climateConfig.getAutoTuneKd()
                );
                Serial.print("AutoTune Kp: "); Serial.println(climateConfig.getAutoTuneKp(), 4);
                Serial.print("AutoTune Ki: "); Serial.println(climateConfig.getAutoTuneKi(), 4);
                Serial.print("AutoTune Kd: "); Serial.println(climateConfig.getAutoTuneKd(), 4);
            } else {
                Serial.println("Using default PID parameters for temperature");
                temperaturePID->SetTunings(
                    climateConfig.getTemperatureKp(),
                    climateConfig.getTemperatureKi(),
                    climateConfig.getTemperatureKd()
                );
            }
            Serial.println("Temperature PID parameters updated from ClimateConfig");        }
        
    } else {
        Serial.println("Failed to load from ClimateConfig, falling back to Configuration class");
        
        // Fallback to original Configuration class
        updateInterval = Configuration::getClimateUpdateInterval();
        
        // Update PID parameters for temperature controller
        if (temperaturePID != nullptr) {
            temperaturePID->SetTunings(
                Configuration::getTemperatureKp(),
                Configuration::getTemperatureKi(),
                Configuration::getTemperatureKd()
            );            Serial.println("Temperature PID parameters updated from Configuration");
        }
    }
    
    Serial.print("Update interval set to: ");
    Serial.print(updateInterval);
    Serial.println(" ms");
    
    Serial.println("Configuration reload completed");
}

// Set temperature PID parameters
void ClimateController::setTemperaturePID(double kp, double ki, double kd) {
    if (temperaturePID != nullptr) {
        temperaturePID->SetTunings(kp, ki, kd);
        Serial.print("Temperature PID updated - Kp: ");
        Serial.print(kp);
        Serial.print(", Ki: ");
        Serial.print(ki);
        Serial.print(", Kd: ");
        Serial.println(kd);
    } else {
        Serial.println("Error: Temperature PID controller not initialized");
    }
}

// Update ClimateConfig JSON file with current controller settings
void ClimateController::updateClimateConfigFile() {
    Serial.println("Updating ClimateConfig JSON file with current settings...");
    
    ClimateConfig& climateConfig = ClimateConfig::getInstance();
    
    // Update ClimateConfig with current controller settings
    climateConfig.setTemperatureSetpoint(temperatureSetpoint);
    climateConfig.setHumiditySetpoint(humiditySetpoint);
    climateConfig.setAutoFanControl(autoFanControlEnabled);
    climateConfig.setUpdateInterval(updateInterval);
      // Convert boolean flags to strings and set modes
    String climateMode = temperatureControlEnabled ? "AUTO" : "OFF";
    climateConfig.setClimateMode(climateMode);
    
    String humidityMode = humidityControlEnabled ? "AUTO" : "OFF";
    climateConfig.setHumidityMode(humidityMode);
      // Save to JSON file
    if (climateConfig.saveToJsonFile("/ClimateConfig.json")) {
        Serial.println("ClimateConfig JSON file updated successfully");  
    } else {
        Serial.println("Failed to update ClimateConfig JSON file");
    }
}

// PID AutoTune functionality
bool ClimateController::startTemperatureAutoTune(double targetSetpoint, double outputStep, double noiseband, unsigned int lookBack) {
    if (temperatureAutoTuning) {
        Serial.println("Temperature AutoTune already in progress");
        return false;
    }
    
    if (temperaturePID == nullptr || sensor == nullptr) {
        Serial.println("Temperature PID or sensor not initialized");
        return false;
    }
    
    // Clean up any existing AutoTuner
    if (temperatureAutoTuner != nullptr) {
        delete temperatureAutoTuner;
        temperatureAutoTuner = nullptr;
    }
    
    // Use current temperature setpoint if not specified
    if (targetSetpoint == 0.0) {
        targetSetpoint = temperatureSetpoint;
    }    // Set default parameters optimized for slow-responding climate system
    if (outputStep == 0.0) {
        // Get configured output step from ClimateConfig
        ClimateConfig& climateConfig = ClimateConfig::getInstance();
        outputStep = climateConfig.getAutoTuneOutputStep();
        
        Serial.print("Using configured AutoTune output step: ");
        Serial.print(outputStep);
        Serial.println("% for balanced system response");
    }
    if (noiseband == 0.0) {
        noiseband = 0.3;    // Reduced from default for better precision
    }
    if (lookBack == 0) {
        lookBack = 1800;    // Increased to 30 minutes (1800 sec) for slow thermal response
    }
    
    // Create new AutoTuner
    temperatureAutoTuner = new PID_ATune(&tempInput, &tempOutput);
    if (temperatureAutoTuner == nullptr) {
        Serial.println("Failed to create temperature AutoTuner");
        return false;
    }
    
    // Configure AutoTuner parameters
    temperatureAutoTuner->SetNoiseBand(noiseband);
    temperatureAutoTuner->SetOutputStep(outputStep);
    temperatureAutoTuner->SetLookbackSec((int)lookBack);
    
    // Set the setpoint for autotuning
    autoTuneSetpoint = targetSetpoint;
    autoTuneOutputStep = outputStep;
    tempSetpoint = targetSetpoint;
      // Switch PID to manual mode for autotuning
    temperaturePID->SetMode(MANUAL);
    
    // Start autotuning
    temperatureAutoTuning = true;
    autoTuneStartTime = millis();
    currentAutoTuneType = AutoTuneType::NORMAL;
    expectedAutoTuneDuration = 3 * 60 * 60 * 1000; // 3 hours in milliseconds (middle of 2-4 hour range)
    
    Serial.println("=== Temperature PID AutoTune Started ===");
    Serial.print("Target Setpoint: ");
    Serial.print(targetSetpoint);
    Serial.println("°C");    Serial.print("Output Step: ");
    Serial.print(outputStep);
    Serial.println("% (configurable power level for system response)");
    Serial.print("Noise Band: ");
    Serial.print(noiseband);
    Serial.println("°C");
    Serial.print("Look Back: ");
    Serial.print(lookBack);
    Serial.println(" seconds (30 minutes for thermal lag)");
    Serial.println("Expected duration: 2-4 hours for complete analysis");
    Serial.println("======================================");
    
    return true;
}

void ClimateController::stopAutoTune() {
    if (temperatureAutoTuning) {
        temperatureAutoTuning = false;
        
        // Return PID to automatic mode
        if (temperaturePID != nullptr) {
            temperaturePID->SetMode(AUTOMATIC);
        }
        
        // Clean up AutoTuner
        if (temperatureAutoTuner != nullptr) {
            delete temperatureAutoTuner;
            temperatureAutoTuner = nullptr;
        }
        
        // Reset AutoTune tracking variables
        expectedAutoTuneDuration = 0;
        currentAutoTuneType = AutoTuneType::NORMAL;
        
        Serial.println("Temperature AutoTune stopped");
    }
}

void ClimateController::updateAutoTune() {
    // Update temperature AutoTune
    if (temperatureAutoTuning && temperatureAutoTuner != nullptr) {
        tempInput = currentTemperature;
        
        // Provide periodic progress updates during AutoTune
        static unsigned long lastProgressUpdate = 0;
        unsigned long currentTime = millis();        // Show basic progress every 30 seconds during AutoTune
        if (currentTime - lastProgressUpdate >= 30000) {
            unsigned long autoTuneRuntime = (currentTime - autoTuneStartTime) / 1000;
            unsigned long autoTuneRuntimeMs = currentTime - autoTuneStartTime;
            
            float progressPercentage = 0.0;
            if (expectedAutoTuneDuration > 0) {
                progressPercentage = (float)autoTuneRuntimeMs / (float)expectedAutoTuneDuration * 100.0;
                if (progressPercentage > 100.0) progressPercentage = 100.0;
            }
            
            Serial.print("AutoTune: ");
            Serial.print(autoTuneRuntime);
            Serial.print("s (");
            Serial.print(progressPercentage, 0);
            Serial.print("%) ");
            Serial.print(currentTemperature, 1);
            Serial.print("°C -> ");
            Serial.print(autoTuneSetpoint, 1);
            Serial.println("°C");
            lastProgressUpdate = currentTime;
        }
          if (temperatureAutoTuner->Runtime()) {
            // AutoTune is complete
            temperatureAutoTuning = false;
            
            // Get the tuned parameters
            double kp = temperatureAutoTuner->GetKp();
            double ki = temperatureAutoTuner->GetKi();
            double kd = temperatureAutoTuner->GetKd();
            
            // Apply the new parameters
            temperaturePID->SetTunings(kp, ki, kd);
            temperaturePID->SetMode(AUTOMATIC);            // Update ClimateConfig with new parameters
            ClimateConfig& climateConfig = ClimateConfig::getInstance();
            climateConfig.setTemperaturePID(kp, ki, kd);
              // Save AutoTune results separately for future use
            climateConfig.setAutoTuneResults(kp, ki, kd);
              // Update all JSON configuration files with the new AutoTune results
            // SPIFFS files
            climateConfig.saveToJsonFile("/ClimateConfig.json");
            climateConfig.saveToJsonFile("/config.json");
            
            // SD card files if SD is available
            if (SD.begin()) {
                climateConfig.saveToJsonFile("/sd/ClimateConfig.json");
                climateConfig.saveToJsonFile("/sd/config.json");
                Serial.println("AutoTune results saved to SD card files");
                SD.end();
            }
            
            Serial.println("=== Temperature AutoTune Complete ===");
            Serial.print("Optimized Kp: ");
            Serial.println(kp, 4);
            Serial.print("Optimized Ki: ");
            Serial.println(ki, 4);
            Serial.print("Optimized Kd: ");
            Serial.println(kd, 4);            Serial.println("AutoTune results saved to configuration");
            Serial.println("====================================");
            
            // Clean up
            delete temperatureAutoTuner;
            temperatureAutoTuner = nullptr;
            
            // Reset AutoTune tracking variables
            expectedAutoTuneDuration = 0;
            currentAutoTuneType = AutoTuneType::NORMAL;
        } else {
            // Check for timeout - force completion if expected duration exceeded by 50%
            unsigned long autoTuneRuntimeMs = currentTime - autoTuneStartTime;
            bool autoTuneTimedOut = false;
            
            if (expectedAutoTuneDuration > 0) {
                unsigned long timeoutThreshold = expectedAutoTuneDuration + (expectedAutoTuneDuration / 2); // 150% of expected duration
                if (autoTuneRuntimeMs >= timeoutThreshold) {
                    autoTuneTimedOut = true;
                    
                    Serial.println("");
                    Serial.println("=== AutoTune TIMEOUT - Forcing Completion ===");
                    Serial.print("Runtime: ");
                    Serial.print(autoTuneRuntimeMs / 1000);
                    Serial.print(" seconds (exceeded ");
                    Serial.print(expectedAutoTuneDuration / 1000);
                    Serial.println(" second expected duration)");
                    Serial.println("AutoTune algorithm hasn't converged naturally.");
                    Serial.println("Using best available parameters...");
                    Serial.println("============================================");
                }
            }
            
            if (autoTuneTimedOut) {
                // Force AutoTune completion with timeout
                temperatureAutoTuning = false;
                
                // Try to get the best available parameters from the AutoTuner
                double kp = temperatureAutoTuner->GetKp();
                double ki = temperatureAutoTuner->GetKi();
                double kd = temperatureAutoTuner->GetKd();
                
                // If parameters are invalid/zero, use reasonable defaults based on current config
                ClimateConfig& climateConfig = ClimateConfig::getInstance();
                if (kp <= 0.0 || ki <= 0.0 || kd < 0.0) {
                    // Use current PID parameters as fallback
                    kp = climateConfig.getTemperatureKp();
                    ki = climateConfig.getTemperatureKi();
                    kd = climateConfig.getTemperatureKd();
                    
                    Serial.println("WARNING: AutoTune parameters invalid - using current config values");
                    Serial.print("Fallback Kp: ");
                    Serial.println(kp, 4);
                    Serial.print("Fallback Ki: ");
                    Serial.println(ki, 4);
                    Serial.print("Fallback Kd: ");
                    Serial.println(kd, 4);
                } else {
                    // Apply the partial AutoTune results
                    temperaturePID->SetTunings(kp, ki, kd);
                      // Update ClimateConfig with new parameters
                    climateConfig.setTemperaturePID(kp, ki, kd);
                    
                    // Save AutoTune results with timeout flag                    climateConfig.setAutoTuneResults(kp, ki, kd);
                      // Update all JSON configuration files with the new AutoTune results
                    // SPIFFS files
                    climateConfig.saveToJsonFile("/ClimateConfig.json");
                    climateConfig.saveToJsonFile("/config.json");
                    
                    // SD card files if SD is available
                    if (SD.begin()) {
                        climateConfig.saveToJsonFile("/sd/ClimateConfig.json");
                        climateConfig.saveToJsonFile("/sd/config.json");
                        Serial.println("AutoTune timeout results saved to SD card files");
                        SD.end();
                    }
                    
                    Serial.println("=== Temperature AutoTune Complete (TIMEOUT) ===");
                    Serial.print("Partial Kp: ");
                    Serial.println(kp, 4);
                    Serial.print("Partial Ki: ");
                    Serial.println(ki, 4);
                    Serial.print("Partial Kd: ");
                    Serial.println(kd, 4);
                    Serial.println("WARNING: Results may be suboptimal due to timeout");
                    Serial.println("Consider running AutoTune again for better results");
                    Serial.println("==============================================");
                }
                
                // Return PID to automatic mode
                temperaturePID->SetMode(AUTOMATIC);
                
                // Clean up
                delete temperatureAutoTuner;
                temperatureAutoTuner = nullptr;
                  // Reset AutoTune tracking variables
                expectedAutoTuneDuration = 0;
                currentAutoTuneType = AutoTuneType::NORMAL;
            } else {
                // AutoTune is still running, use the output
                double rawOutput = temperatureAutoTuner->GetOutputStep();
                
                // Determine if we should heat or cool based on temperature error
                double error = autoTuneSetpoint - currentTemperature;
            
                if (error > 0) {
                    // Temperature is below setpoint, heating needed (positive output)
                    tempOutput = abs(rawOutput);
                } else {
                    // Temperature is above setpoint, cooling needed (negative output)
                    tempOutput = -abs(rawOutput);
                }
            }
        }
    }
}

void ClimateController::getAutoTuneResults(double& kp, double& ki, double& kd) {
    kp = ki = kd = 0.0;
    
    if (temperatureAutoTuning && temperatureAutoTuner != nullptr) {
        kp = temperatureAutoTuner->GetKp();
        ki = temperatureAutoTuner->GetKi();
        kd = temperatureAutoTuner->GetKd();
    }
}

void ClimateController::printAutoTuneStatus() {
    if (temperatureAutoTuning) {
        unsigned long elapsed = (millis() - autoTuneStartTime) / 1000;
        unsigned long elapsedMs = millis() - autoTuneStartTime;
        
        Serial.println("=== Temperature AutoTune Status ===");
        Serial.print("Elapsed Time: ");
        Serial.print(elapsed);
        Serial.print(" seconds");
        
        // Calculate and show progress percentage
        if (expectedAutoTuneDuration > 0) {
            float progressPercentage = (float)elapsedMs / (float)expectedAutoTuneDuration * 100.0;
            if (progressPercentage > 100.0) progressPercentage = 100.0;
            Serial.print(" (");
            Serial.print(progressPercentage, 1);
            Serial.print("% complete)");
        }
        Serial.println();
        
        // Show AutoTune mode
        const char* autoTuneTypeName = "Unknown";
        switch (currentAutoTuneType) {
            case AutoTuneType::NORMAL: autoTuneTypeName = "Normal"; break;
            case AutoTuneType::FAST: autoTuneTypeName = "Fast"; break;
        }
        Serial.print("Mode: ");
        Serial.print(autoTuneTypeName);
        Serial.println(" AutoTune");
        
        Serial.print("Current Temperature: ");
        Serial.print(currentTemperature, 2);
        Serial.println("°C");
        Serial.print("Target Setpoint: ");
        Serial.print(autoTuneSetpoint, 2);
        Serial.println("°C");
        Serial.print("Current Output: ");
        Serial.print(tempOutput, 2);
        Serial.println("%");
        
        // Show estimated remaining time if available
        if (expectedAutoTuneDuration > 0 && elapsedMs < expectedAutoTuneDuration) {
            unsigned long remainingMs = expectedAutoTuneDuration - elapsedMs;
            unsigned long remainingMinutes = remainingMs / (60 * 1000);
            Serial.print("Estimated Time Remaining: ");
            Serial.print(remainingMinutes);
            Serial.println(" min");
        }

        // Display current Kp, Ki, Kd values from the autotuner if available
        if (temperatureAutoTuner != nullptr) {
            double kp = temperatureAutoTuner->GetKp();
            double ki = temperatureAutoTuner->GetKi();
            double kd = temperatureAutoTuner->GetKd();
            Serial.print("Current AutoTune Kp: "); Serial.println(kp, 4);
            Serial.print("Current AutoTune Ki: "); Serial.println(ki, 4);
            Serial.print("Current AutoTune Kd: "); Serial.println(kd, 4);
        }
        Serial.println("==================================");
    } else {
        Serial.println("No AutoTune currently running");
    }
}

// Fast AutoTune functionality for testing purposes
bool ClimateController::startTemperatureAutoTuneFast(double targetSetpoint, double outputStep, double noiseband, unsigned int lookBack) {
    if (temperatureAutoTuning) {
        Serial.println("Temperature AutoTune already in progress");
        return false;
    }
    
    if (temperaturePID == nullptr || sensor == nullptr) {
        Serial.println("Temperature PID or sensor not initialized");
        return false;
    }
    
    // Clean up any existing AutoTuner
    if (temperatureAutoTuner != nullptr) {
        delete temperatureAutoTuner;
        temperatureAutoTuner = nullptr;
    }
    
    // Use current temperature setpoint if not specified
    if (targetSetpoint == 0.0) {
        targetSetpoint = temperatureSetpoint;
    }    // Set FAST testing parameters for quick results
    if (outputStep == 0.0) {
        // Get configured FAST autotune output step from ClimateConfig
        ClimateConfig& climateConfig = ClimateConfig::getInstance();
        outputStep = climateConfig.getFastAutoTuneOutputStep();
        
        Serial.print("Using configured AutoTune output step: ");
        Serial.print(outputStep);
        Serial.println("% for FAST mode testing");
    }
    if (noiseband == 0.0) {
        noiseband = 0.5;      // Larger noise band (0.5°C) for quicker detection
    }
    if (lookBack == 0) {
        lookBack = 300;       // Much shorter lookback (5 minutes) for testing
    }
    
    // Create new AutoTuner
    temperatureAutoTuner = new PID_ATune(&tempInput, &tempOutput);
    if (temperatureAutoTuner == nullptr) {
        Serial.println("Failed to create temperature AutoTuner");
        return false;
    }
    
    // Configure AutoTuner parameters
    temperatureAutoTuner->SetNoiseBand(noiseband);
    temperatureAutoTuner->SetOutputStep(outputStep);
    temperatureAutoTuner->SetLookbackSec((int)lookBack);
    
    // Set the setpoint for autotuning
    autoTuneSetpoint = targetSetpoint;
    autoTuneOutputStep = outputStep;
    tempSetpoint = targetSetpoint;
    
    // Switch PID to manual mode for autotuning
    temperaturePID->SetMode(MANUAL);
      // Start autotuning
    temperatureAutoTuning = true;
    autoTuneStartTime = millis();
    currentAutoTuneType = AutoTuneType::FAST;
    expectedAutoTuneDuration = 22.5 * 60 * 1000; // 22.5 minutes in milliseconds (middle of 15-30 minute range)
    
    Serial.println("=== FAST Temperature PID AutoTune Started (TESTING MODE) ===");
    Serial.print("Target Setpoint: ");
    Serial.print(targetSetpoint);
    Serial.println("°C");
    Serial.print("Output Step: ");
    Serial.print(outputStep);
    Serial.println("% (HIGH power for fast testing)");
    Serial.print("Noise Band: ");
    Serial.print(noiseband);
    Serial.println("°C (relaxed for quick detection)");
    Serial.print("Look Back: ");
    Serial.print(lookBack);
    Serial.println(" seconds (5 minutes for fast testing)");
    Serial.println("Expected duration: 15-30 minutes for quick analysis");
    Serial.println("WARNING: Results may be less accurate - use for testing only!");
    Serial.println("===========================================================");
    
    return true;
}

// Set fast testing mode - reduces update intervals for quicker AutoTune testing
void ClimateController::setFastTestingMode(bool enabled) {
    if (enabled) {
        // Store original interval for restoration
        static unsigned long originalInterval = updateInterval;
        
        // Set very fast update interval for testing (100ms instead of 500ms)
        updateInterval = 100;
        statusPrintInterval = 5000; // More frequent status updates (5 seconds)
        
        Serial.println("=== FAST TESTING MODE ENABLED ===");
        Serial.print("Update interval reduced to: ");
        Serial.print(updateInterval);
        Serial.println("ms for rapid AutoTune testing");
        Serial.print("Status updates every: ");
        Serial.print(statusPrintInterval / 1000);
        Serial.println(" seconds");
        Serial.println("WARNING: High CPU usage - for testing only!");
        Serial.println("=================================");
    } else {
        // Restore normal intervals
        ClimateConfig& climateConfig = ClimateConfig::getInstance();
        updateInterval = climateConfig.getUpdateInterval();
        statusPrintInterval = 10000; // Back to 10 seconds
        
        Serial.println("=== FAST TESTING MODE DISABLED ===");
        Serial.print("Update interval restored to: ");
        Serial.print(updateInterval);
        Serial.println("ms for normal operation");
        Serial.println("==================================");
    }
}

// Dew Point Compensation Implementation

bool ClimateController::isDewPointCompensationEnabled() const {
    if (radiatorSensor == nullptr) {
        return false; // Cannot enable without radiator sensor
    }
    
    // TEMPORARY: Force enable dew point compensation for testing
    return true;
    
    // ClimateConfig& climateConfig = ClimateConfig::getInstance();
    // return climateConfig.getDewPointCompensationEnabled();
}

float ClimateController::calculateDewPoint(float temperature, float humidity) const {
    // Magnus formula for dew point calculation
    // Accurate for temperature range -40°C to +50°C and humidity 1% to 100%
    
    // Constants for Magnus formula
    const float a = 17.27;
    const float b = 237.7;
    
    // Calculate gamma
    float gamma = (a * temperature) / (b + temperature) + log(humidity / 100.0);
    
    // Calculate dew point
    float dewPoint = (b * gamma) / (a - gamma);
    
    return dewPoint;
}

void ClimateController::updateDewPointCompensation() {
    if (!isDewPointCompensationEnabled()) {
        return; // Dew point compensation is disabled
    }
    
    unsigned long currentTime = millis();
    ClimateConfig& climateConfig = ClimateConfig::getInstance();
    
    // Check if it's time to update dew point calculation
    if (currentTime - lastDewPointUpdate < climateConfig.getDewPointUpdateInterval()) {
        return; // Not time to update yet
    }
    
    // Update radiator sensor reading
    updateRadiatorSensorReading();
    
    // Calculate dew point using interior temperature and humidity
    dewPoint = calculateDewPoint(currentTemperature, currentHumidity);
    
    // Calculate minimum allowed cooling temperature based on dew point and safety margin
    float safetyMargin = climateConfig.getDewPointSafetyMargin();
    float minCoolingTemp = climateConfig.getMinCoolingTemperature();
    
    // Use the higher of: (dew point + safety margin) or minimum cooling temperature
    minAllowedCoolingTemperature = max(dewPoint + safetyMargin, minCoolingTemp);
    
    lastDewPointUpdate = currentTime;      // Debug output for dew point compensation
    static unsigned long lastDebugPrint = 0;
    if (currentTime - lastDebugPrint >= 30000) { // Print debug every 30 seconds
        Serial.print("DewPoint: ");
        Serial.print(dewPoint, 1);
        Serial.print("°C, MinCool: ");
        Serial.print(minAllowedCoolingTemperature, 1);
        Serial.print("°C, Radiator: ");
        Serial.print(currentRadiatorTemperature, 1);
        Serial.print("°C");
        if (currentRadiatorTemperature <= minAllowedCoolingTemperature) {
            Serial.print(" [COOLING LIMITED]");
        }
        Serial.println();
        lastDebugPrint = currentTime;
    }
}

void ClimateController::updateRadiatorSensorReading() {
    if (radiatorSensor) {
        radiatorSensor->update();
        currentRadiatorTemperature = radiatorSensor->getTemperature();
    }
}

float ClimateController::limitCoolingOutputForDewPoint(float coolingOutput) {
    if (!isDewPointCompensationEnabled() || radiatorSensor == nullptr) {
        return coolingOutput; // No dew point compensation, return original value
    }
    
    // Check if radiator temperature is approaching the dew point limit
    if (currentRadiatorTemperature <= minAllowedCoolingTemperature) {
        // Radiator is at or below the safe cooling temperature
        // Gradually reduce or stop cooling to prevent condensation
        
        float temperatureDifference = minAllowedCoolingTemperature - currentRadiatorTemperature;
        
        if (temperatureDifference >= 1.0) {
            // Radiator is significantly below safe temperature - stop cooling immediately
            Serial.println("Dew Point Protection: Cooling STOPPED - radiator too cold");
            return 0.0;
        } else if (temperatureDifference >= 0.5) {
            // Radiator is approaching unsafe temperature - reduce cooling significantly
            Serial.println("Dew Point Protection: Cooling REDUCED to 20%");
            return coolingOutput * 0.2; // Reduce to 20% of original output
        } else {
            // Radiator is slightly below safe temperature - reduce cooling moderately
            Serial.println("Dew Point Protection: Cooling REDUCED to 50%");
            return coolingOutput * 0.5; // Reduce to 50% of original output
        }
    }
    
    return coolingOutput; // No adjustment needed
}