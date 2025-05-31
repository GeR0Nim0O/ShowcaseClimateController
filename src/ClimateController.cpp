#include "ClimateController.h"
#include "DeviceRegistry/DeviceRegistry.h"
#include "../lib/Config/ClimateConfig/ClimateConfig.h"

// Static factory method for automatic device discovery and initialization
ClimateController* ClimateController::createFromDeviceRegistry() {
    try {
        Serial.println("Initializing ClimateController from DeviceRegistry...");
        
        // Use DeviceRegistry to get devices instead of manual searching
        DeviceRegistry& registry = DeviceRegistry::getInstance();
        
        // Get GPIO expander from DeviceRegistry
        PCF8574gpio* gpioExpander = (PCF8574gpio*)registry.getDeviceByType("GPIO", 0);
        if (gpioExpander != nullptr) {
            Serial.println("Found GPIO expander for climate control via DeviceRegistry");
        } else {
            Serial.println("No GPIO expander found in DeviceRegistry");
            return nullptr;
        }
        
        // Get temperature/humidity sensor from DeviceRegistry - specifically look for Interior labeled sensor
        SHTsensor* climateTemperatureSensor = (SHTsensor*)registry.getDeviceByTypeAndLabel("TemperatureHumidity", "Interior");
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
                return nullptr;
            }
        }
        
        // Get DAC from DeviceRegistry - using proper DeviceRegistry access pattern
        GP8403dac* climateDac = (GP8403dac*)registry.getDeviceByType("DAC", 0);
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
            Serial.println("Creating climate controller with discovered devices");
            
            try {
                Serial.println("Allocating climate controller...");
                ClimateController* controller = new ClimateController(gpioExpander, climateTemperatureSensor, climateDac);
                
                if (controller != nullptr) {
                    Serial.println("Climate controller allocated, calling begin()");
                    
                    if (controller->begin()) {
                        Serial.println("Climate controller initialized successfully");
                        
                        // Enable automatic fan control by default
                        controller->setAutoFanControl(true);
                        Serial.println("Automatic fan control enabled");
                        
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
    : gpio(nullptr), sensor(nullptr), dac(nullptr),
      temperatureSetpoint(22.0), humiditySetpoint(50.0),
      currentTemperature(0.0), currentHumidity(0.0),
      climateMode(ClimateMode::AUTO), humidityMode(HumidityMode::AUTO),
      heatingActive(false), coolingActive(false), 
      humidifyingActive(false), dehumidifyingActive(false),
      tempControlEnabled(false), 
      fanInteriorActive(false), fanExteriorActive(false),  // Initialize fan states
      autoFanControlEnabled(true),                         // Enable auto fan control by default
      lastUpdate(0), updateInterval(Configuration::getClimateUpdateInterval()),
      heatingPower(0.0), coolingPower(0.0), 
      humidifierPower(0.0), dehumidifierPower(0.0),
      temperaturePID(nullptr), humidityPID(nullptr),
      tempInput(0.0), tempOutput(0.0), tempSetpoint(temperatureSetpoint),
      humInput(0.0), humOutput(0.0), humSetpoint(humiditySetpoint) {
    
    // Safely assign the device pointers
    this->gpio = gpioExpander;
    this->sensor = tempHumSensor;
    this->dac = dac;
    
    // Initialize with default pin mappings - will be updated in begin()
    pinFanExterior = 0;
    pinFanInterior = 1;
    pinHumidify = 2;
    pinDehumidify = 3;
    pinTemperatureEnable = 4;
    pinTemperatureCool = 5;
    pinTemperatureHeat = 6;
      // Initialize PID controllers - with safety checks
    try {
        Serial.println("Initializing Temperature PID controller");
        temperaturePID = new PID(&tempInput, &tempOutput, &tempSetpoint, 
                                Configuration::getTemperatureKp(), 
                                Configuration::getTemperatureKi(), 
                                Configuration::getTemperatureKd(), DIRECT);
        
        if (temperaturePID != nullptr) {
            temperaturePID->SetMode(AUTOMATIC);
            temperaturePID->SetOutputLimits(-100, 100); // -100 = full cooling, +100 = full heating
            Serial.println("Temperature PID initialized successfully");
        }
        
        Serial.println("Initializing Humidity PID controller");
        humidityPID = new PID(&humInput, &humOutput, &humSetpoint,
                             Configuration::getHumidityKp(), 
                             Configuration::getHumidityKi(), 
                             Configuration::getHumidityKd(), DIRECT);
        
        if (humidityPID != nullptr) {
            humidityPID->SetMode(AUTOMATIC);
            humidityPID->SetOutputLimits(-100, 100); // -100 = full dehumidify, +100 = full humidify
            Serial.println("Humidity PID initialized successfully");
        }
    }
    catch (...) {
        Serial.println("Exception during PID controller initialization");
        // Clean up if an exception occurs
        if (temperaturePID != nullptr) {
            delete temperaturePID;
            temperaturePID = nullptr;
        }
        if (humidityPID != nullptr) {
            delete humidityPID;
            humidityPID = nullptr;
        }
    }
    
    Serial.println("ClimateController constructor completed successfully");
}

bool ClimateController::begin() {
    // Initialize DAC if available
    if (dac != nullptr) {
        // Check if DAC is already initialized (to avoid double initialization)
        if (dac->isInitialized()) {
            // Test DAC by setting it to 0V initially
            dac->setChannelVoltage(0, 0.0);
        } else {
            if (dac->begin()) {
                // Test DAC by setting it to 0V initially
                dac->setChannelVoltage(0, 0.0);
            }
        }
    }
    
    // Initialize pin mappings
    initializePinMappings();
    
    // Ensure GPIO is in output mode and all pins start LOW
    if (gpio != nullptr) {
        gpio->forceOutputMode();
        
        // Initialize ALL pins to LOW/false state
        for (int pin = 0; pin < 8; pin++) {
            gpio->writePin(pin, false);
        }
        
        // Force final state to 0x00
        gpio->writeByte(0x00);
    }
    
    return true;
}

void ClimateController::update() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastUpdate >= updateInterval) {
        // CRITICAL FIX: Store current GPIO state before any operations
        uint8_t gpioStateBefore = 0x00;
        if (gpio != nullptr) {
            gpioStateBefore = gpio->getGPIOState();
        }
        
        updateSensorReadings();
        
        // Store previous states to avoid unnecessary writes
        static bool lastTempControlEnabled = false;
        static bool lastHeatingActive = false;
        static bool lastCoolingActive = false;
        static bool lastHumidifyingActive = false;
        static bool lastDehumidifyingActive = false;
        static bool lastFanInteriorActive = false;       // Add fan state tracking
        static bool lastFanExteriorActive = false;       // Add fan state tracking
        
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
        
        // NEW: Always apply fan control if climate control is active to ensure fans stay on
        bool forceApplyFans = (fanInteriorActive || fanExteriorActive);
        
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
        
        if (fanStateChanged || forceApplyFans) {
            applyFanControl();
            lastFanInteriorActive = fanInteriorActive;
            lastFanExteriorActive = fanExteriorActive;
        }
        
        applyDACControls(); // Apply DAC controls
        
        // CRITICAL FIX: Verify GPIO state wasn't corrupted during update
        if (gpio != nullptr) {
            uint8_t gpioStateAfter = gpio->getGPIOState();
            if (gpioStateAfter != gpioStateBefore && !tempStateChanged && !humidityStateChanged && !fanStateChanged) {
                Serial.print("WARNING: Unexpected GPIO state change in ClimateController::update()! Before: 0x");
                Serial.print(gpioStateBefore, HEX);
                Serial.print(", After: 0x");
                Serial.println(gpioStateAfter, HEX);
                
                // This suggests an external interference - force refresh our desired state
                Serial.println("Re-applying climate control due to state corruption");
                applyTemperatureControl();
                applyHumidityControl();
                applyFanControl();  // Re-apply fan control
            }
        }
          // NEW: Periodically refresh GPIO state to prevent drift
        static unsigned long lastGpioRefresh = 0;
        if (currentTime - lastGpioRefresh >= 10000) { // Every 10 seconds
            if (gpio != nullptr) {
                gpio->refreshOutputState();
                // Force re-apply fan control during refresh
                if (fanInteriorActive || fanExteriorActive) {
                    applyFanControl();
                }
            }
            lastGpioRefresh = currentTime;
        }
        
        lastUpdate = currentTime;
    }
}

void ClimateController::updateSensorReadings() {
    sensor->update();
    currentTemperature = sensor->getTemperature();
    currentHumidity = sensor->getHumidity();
    
    // Update PID inputs
    tempInput = currentTemperature;
    tempSetpoint = temperatureSetpoint;
    humInput = currentHumidity;
    humSetpoint = humiditySetpoint;
}

void ClimateController::updateTemperatureControl() {
    if (climateMode == ClimateMode::OFF) {
        tempControlEnabled = false;
        heatingActive = false;
        coolingActive = false;
        heatingPower = 0.0;
        coolingPower = 0.0;
        return;
    }
    
    temperaturePID->Compute();
    
    switch (climateMode) {
        case ClimateMode::HEATING:
            heatingActive = (tempOutput > 0);
            coolingActive = false;
            tempControlEnabled = heatingActive;
            heatingPower = heatingActive ? map(tempOutput, 0, 100, 0, 100) : 0.0;
            coolingPower = 0.0;
            break;
            
        case ClimateMode::COOLING:
            heatingActive = false;
            coolingActive = (tempOutput < 0);
            tempControlEnabled = coolingActive;
            heatingPower = 0.0;
            coolingPower = coolingActive ? map(-tempOutput, 0, 100, 0, 100) : 0.0;
            break;
            
        case ClimateMode::AUTO:
            if (tempOutput > 0.1) // Very small deadband for precise control
                heatingActive = true, coolingActive = false, heatingPower = map(tempOutput, 0.1, 100, 0, 100), coolingPower = 0.0;
            else if (tempOutput < -0.1)
                heatingActive = false, coolingActive = true, heatingPower = 0.0, coolingPower = map(-tempOutput, 0.1, 100, 0, 100);
            else
                heatingActive = false, coolingActive = false, heatingPower = 0.0, coolingPower = 0.0;
            tempControlEnabled = (heatingActive || coolingActive);
            break;
            
        case ClimateMode::OFF:
        default:
            tempControlEnabled = false;
            heatingActive = false;
            coolingActive = false;
            heatingPower = 0.0;
            coolingPower = 0.0;
            break;
    }
}

void ClimateController::updateHumidityControl() {
    if (humidityMode == HumidityMode::OFF) {
        humidifyingActive = false;
        dehumidifyingActive = false;
        return;
    }
      // Use a simple hysteresis control for humidity (on/off control)
    const float hysteresis = Configuration::getHumidityHysteresis(); // Get hysteresis from configuration
    
    switch (humidityMode) {
        case HumidityMode::HUMIDIFYING:
            // Only humidifying mode
            if (currentHumidity < humiditySetpoint - hysteresis) {
                humidifyingActive = true;  // Turn on humidifier
            } else if (currentHumidity > humiditySetpoint) {
                humidifyingActive = false; // Turn off humidifier
            }
            // Otherwise keep previous state (hysteresis)
            dehumidifyingActive = false;   // Ensure dehumidifier is off
            break;
            
        case HumidityMode::DEHUMIDIFYING:
            // Only dehumidifying mode
            if (currentHumidity > humiditySetpoint + hysteresis) {
                dehumidifyingActive = true;  // Turn on dehumidifier
            } else if (currentHumidity < humiditySetpoint) {
                dehumidifyingActive = false; // Turn off dehumidifier
            }
            // Otherwise keep previous state (hysteresis)
            humidifyingActive = false;      // Ensure humidifier is off
            break;
            
        case HumidityMode::AUTO:
            // Auto mode - can switch between humidifying and dehumidifying
            if (currentHumidity < humiditySetpoint - hysteresis) {
                humidifyingActive = true;
                dehumidifyingActive = false;
            } else if (currentHumidity > humiditySetpoint + hysteresis) {
                humidifyingActive = false;
                dehumidifyingActive = true;
            } else if (currentHumidity >= humiditySetpoint - 1.0 && 
                      currentHumidity <= humiditySetpoint + 1.0) {
                // Within +/- 1% of setpoint, turn everything off
                humidifyingActive = false;
                dehumidifyingActive = false;
            }
            // Otherwise keep previous state (hysteresis)
            break;
            
        case HumidityMode::OFF:
        default:
            humidifyingActive = false;
            dehumidifyingActive = false;
            break;
    }
}

void ClimateController::updateFanControl() {
    if (!autoFanControlEnabled) {
        return;
    }
    
    // Determine if ANY climate control is active
    bool tempControlActive = (climateMode != ClimateMode::OFF) && 
                            (tempControlEnabled || heatingActive || coolingActive);
    
    bool humidityControlActive = (humidityMode != HumidityMode::OFF) && 
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
        float dacVoltage = 0.0;
        
        if (heatingActive && heatingPower > 0) {
            // Convert heating power (0-100%) to voltage (0-5V)
            dacVoltage = (heatingPower / 100.0) * 5.0;
        } else if (coolingActive && coolingPower > 0) {
            // Convert cooling power (0-100%) to voltage (0-5V)
            dacVoltage = (coolingPower / 100.0) * 5.0;
        }
        
        // Clamp voltage to safe range
        if (dacVoltage > 5.0) dacVoltage = 5.0;
        if (dacVoltage < 0.0) dacVoltage = 0.0;
        
        // Set DAC output voltage
        dac->setChannelVoltage(0, dacVoltage);
        
    } catch (...) {
        // Silently handle DAC errors
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
    Serial.println("ClimateController: Starting initializePinMappings()");
    
    // Initialize with default values matching config.json
    pinFanExterior = 0;
    pinFanInterior = 1;
    pinHumidify = 2;
    pinDehumidify = 3;
    pinTemperatureEnable = 4;
    pinTemperatureCool = 5;
    pinTemperatureHeat = 6;
    
    Serial.println("ClimateController: Default pin mappings set");
    
    // Try to load from configuration with extensive error checking
    try {
        Serial.println("ClimateController: Attempting to load pin mappings from configuration");
        
        pinFanExterior = getPinFromChannelName("FanExterior");
        Serial.print("ClimateController: FanExterior pin: ");
        Serial.println(pinFanExterior);
        
        pinFanInterior = getPinFromChannelName("FanInterior");
        Serial.print("ClimateController: FanInterior pin: ");
        Serial.println(pinFanInterior);
        
        pinHumidify = getPinFromChannelName("Humidify");
        Serial.print("ClimateController: Humidify pin: ");
        Serial.println(pinHumidify);
        
        pinDehumidify = getPinFromChannelName("Dehumidify");
        Serial.print("ClimateController: Dehumidify pin: ");
        Serial.println(pinDehumidify);
        
        pinTemperatureEnable = getPinFromChannelName("TemperatureEnable");
        Serial.print("ClimateController: TemperatureEnable pin: ");
        Serial.println(pinTemperatureEnable);
        
        pinTemperatureCool = getPinFromChannelName("TemperatureCool");
        Serial.print("ClimateController: TemperatureCool pin: ");
        Serial.println(pinTemperatureCool);
        
        pinTemperatureHeat = getPinFromChannelName("TemperatureHeat");
        Serial.print("ClimateController: TemperatureHeat pin: ");
        Serial.println(pinTemperatureHeat);
        
        Serial.println("ClimateController: Pin mappings loaded successfully");
    }
    catch (...) {
        Serial.println("ClimateController: Exception during pin mapping loading, using defaults");
    }
    
    Serial.println("ClimateController: initializePinMappings() completed");
}

uint8_t ClimateController::getPinFromChannelName(const String& channelName) {
    Serial.print("ClimateController: Getting pin for channel: ");
    Serial.println(channelName);
    
    JsonObject devicesConfig = Configuration::getDevicesConfig();
    
    // Add null/empty check for devicesConfig
    if (devicesConfig.isNull() || devicesConfig.size() == 0) {
        Serial.printf("Warning: devicesConfig is null or empty for %s, using default\n", channelName.c_str());
        // Use default mapping
        if (channelName == "FanExterior") return 0;
        if (channelName == "FanInterior") return 1;
        if (channelName == "Humidify") return 2;
        if (channelName == "Dehumidify") return 3;
        if (channelName == "TemperatureEnable") return 4;
        if (channelName == "TemperatureCool") return 5;
        if (channelName == "TemperatureHeat") return 6;
        return 0;
    }
    
    Serial.println("ClimateController: devicesConfig is valid, checking PCF8574 section");
    
    if (devicesConfig["PCF8574"].is<JsonObject>()) {
        Serial.println("ClimateController: Found PCF8574 section");
        JsonObject pcfConfig = devicesConfig["PCF8574"];
        if (pcfConfig["Channels"].is<JsonObject>()) {
            Serial.println("ClimateController: Found Channels section");
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
                        Serial.printf("ClimateController: Found pin %d for channel %s\n", pin, channelName.c_str());
                        return pin;
                    }
                }
            }
        } else {
            Serial.println("ClimateController: No Channels section found in PCF8574");
        }
    } else {
        Serial.println("ClimateController: No PCF8574 section found in devicesConfig");
    }
      // Return default pin if not found in configuration
    Serial.printf("Warning: Pin mapping not found for %s, using default\n", channelName.c_str());
    if (channelName == "FanExterior") return 0;
    if (channelName == "FanInterior") return 1;
    if (channelName == "Humidify") return 2;
    if (channelName == "Dehumidify") return 3;
    if (channelName == "TemperatureEnable") return 4;
    if (channelName == "TemperatureCool") return 5;
    if (channelName == "TemperatureHeat") return 6;
    
    return 0; // Default fallback
}

// Add this implementation of the safeWritePin method we previously added to the header
bool ClimateController::safeWritePin(uint8_t pin, bool value) {
    if (!gpio || !gpio->isInitialized()) {
        return false;
    }
    
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
    Serial.print(getCurrentTemperature(), 1);
    Serial.print("°C (setpoint: ");
    Serial.print(getTemperatureSetpoint(), 1);
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
        Serial.print(externalData["T"].toFloat(), 1);
        Serial.println("°C");
        
        Serial.print("External Humidity: ");
        Serial.print(externalData["H"].toFloat(), 1);
        Serial.println("%");
    } else {
        Serial.println("External Conditions: [Sensor not available]");
    }
    
    // Print control status
    Serial.print("Climate Mode: ");
    switch (climateMode) {
        case ClimateMode::AUTO: Serial.println("AUTO"); break;
        case ClimateMode::HEATING: Serial.println("HEATING ONLY"); break;
        case ClimateMode::COOLING: Serial.println("COOLING ONLY"); break;
        case ClimateMode::OFF: Serial.println("OFF"); break;
    }
    
    Serial.print("Humidity Mode: ");
    switch (humidityMode) {
        case HumidityMode::AUTO: Serial.println("AUTO"); break;
        case HumidityMode::HUMIDIFYING: Serial.println("HUMIDIFYING ONLY"); break;
        case HumidityMode::DEHUMIDIFYING: Serial.println("DEHUMIDIFYING ONLY"); break;
        case HumidityMode::OFF: Serial.println("OFF"); break;
    }
    
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
    Serial.print(isFanExteriorOn() ? "ON" : "OFF");
    if (isAutoFanControlEnabled()) {
        Serial.println(" (Auto)");
    } else {
        Serial.println(" (Manual)");
    }
    
    Serial.println("==================================");
}

// Static method for controlled update with timing management
void ClimateController::updateControllerWithTiming(ClimateController* controller) {
    if (controller == nullptr) {
        return; // Exit if climate controller is not available
    }
    
    // Only update climate controller every few seconds to avoid GPIO conflicts
    static unsigned long lastClimateUpdate = 0;
    if (millis() - lastClimateUpdate >= 5000) { // Update every 5 seconds
        controller->update();
        lastClimateUpdate = millis();
    }
}

// Configuration method for setting all parameters at once
void ClimateController::configure(float tempSetpoint, float humSetpoint, ClimateMode climateMode, HumidityMode humidityMode) {
    setTemperatureSetpoint(tempSetpoint);
    setHumiditySetpoint(humSetpoint);
    setClimateMode(climateMode);
    setHumidityMode(humidityMode);
    
    Serial.println("Climate controller configured:");
    Serial.print("Temperature setpoint: ");
    Serial.print(tempSetpoint);
    Serial.println("°C");
    Serial.print("Humidity setpoint: ");
    Serial.print(humSetpoint);
    Serial.println("%");
    Serial.print("Climate mode: ");
    switch (climateMode) {
        case ClimateMode::AUTO: Serial.println("AUTO"); break;
        case ClimateMode::HEATING: Serial.println("HEATING"); break;
        case ClimateMode::COOLING: Serial.println("COOLING"); break;
        case ClimateMode::OFF: Serial.println("OFF"); break;
    }
    Serial.print("Humidity mode: ");
    switch (humidityMode) {
        case HumidityMode::AUTO: Serial.println("AUTO"); break;
        case HumidityMode::HUMIDIFYING: Serial.println("HUMIDIFYING"); break;
        case HumidityMode::DEHUMIDIFYING: Serial.println("DEHUMIDIFYING"); break;
        case HumidityMode::OFF: Serial.println("OFF"); break;
    }
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
        
        // Update control modes
        String climateMode = climateConfig.getClimateMode();
        if (climateMode == "HEATING") {
            setClimateMode(ClimateMode::HEATING);
        } else if (climateMode == "COOLING") {
            setClimateMode(ClimateMode::COOLING);
        } else if (climateMode == "OFF") {
            setClimateMode(ClimateMode::OFF);
        } else {
            setClimateMode(ClimateMode::AUTO);
        }
        
        String humidityMode = climateConfig.getHumidityMode();
        if (humidityMode == "HUMIDIFY") {
            setHumidityMode(HumidityMode::HUMIDIFYING);
        } else if (humidityMode == "DEHUMIDIFY") {
            setHumidityMode(HumidityMode::DEHUMIDIFYING);
        } else if (humidityMode == "OFF") {
            setHumidityMode(HumidityMode::OFF);
        } else {
            setHumidityMode(HumidityMode::AUTO);
        }
        
        // Update auto fan control
        setAutoFanControl(climateConfig.getAutoFanControl());
        
        // Update update interval from ClimateConfig
        updateInterval = climateConfig.getUpdateInterval();
        
        // Update PID parameters from ClimateConfig
        if (temperaturePID != nullptr) {
            temperaturePID->SetTunings(
                climateConfig.getTemperatureKp(),
                climateConfig.getTemperatureKi(),
                climateConfig.getTemperatureKd()
            );
            Serial.println("Temperature PID parameters updated from ClimateConfig");
        }
        
        if (humidityPID != nullptr) {
            humidityPID->SetTunings(
                climateConfig.getHumidityKp(),
                climateConfig.getHumidityKi(),
                climateConfig.getHumidityKd()
            );
            Serial.println("Humidity PID parameters updated from ClimateConfig");
        }
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
            );
            Serial.println("Temperature PID parameters updated from Configuration");
        }
        
        // Update PID parameters for humidity controller
        if (humidityPID != nullptr) {
            humidityPID->SetTunings(
                Configuration::getHumidityKp(),
                Configuration::getHumidityKi(),
                Configuration::getHumidityKd()
            );
            Serial.println("Humidity PID parameters updated from Configuration");
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

// Set humidity PID parameters
void ClimateController::setHumidityPID(double kp, double ki, double kd) {
    if (humidityPID != nullptr) {
        humidityPID->SetTunings(kp, ki, kd);
        Serial.print("Humidity PID updated - Kp: ");
        Serial.print(kp);
        Serial.print(", Ki: ");
        Serial.print(ki);
        Serial.print(", Kd: ");
        Serial.println(kd);
    } else {
        Serial.println("Error: Humidity PID controller not initialized");
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
    
    // Convert enums to strings and set modes
    String climateMode;
    switch (this->climateMode) {
        case ClimateMode::HEATING: climateMode = "HEATING"; break;
        case ClimateMode::COOLING: climateMode = "COOLING"; break;
        case ClimateMode::OFF: climateMode = "OFF"; break;
        default: climateMode = "AUTO"; break;
    }
    climateConfig.setClimateMode(climateMode);
    
    String humidityMode;
    switch (this->humidityMode) {
        case HumidityMode::HUMIDIFYING: humidityMode = "HUMIDIFY"; break;
        case HumidityMode::DEHUMIDIFYING: humidityMode = "DEHUMIDIFY"; break;
        case HumidityMode::OFF: humidityMode = "OFF"; break;
        default: humidityMode = "AUTO"; break;
    }
    climateConfig.setHumidityMode(humidityMode);
    
    // Save to JSON file
    if (climateConfig.updateJsonFile("/data/ClimateConfig.json")) {
        Serial.println("ClimateConfig JSON file updated successfully");
    } else {
        Serial.println("Failed to update ClimateConfig JSON file");
    }
}