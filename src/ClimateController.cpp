#include "ClimateController.h"

// Safety limits
#define MAX_TEMPERATURE 35.0
#define MIN_TEMPERATURE 10.0
#define MAX_HUMIDITY 80.0
#define MIN_HUMIDITY 20.0

// Default PID parameters
#define DEFAULT_TEMP_KP 2.0
#define DEFAULT_TEMP_KI 0.5
#define DEFAULT_TEMP_KD 0.1

#define DEFAULT_HUM_KP 1.0
#define DEFAULT_HUM_KI 0.2
#define DEFAULT_HUM_KD 0.05

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
      lastUpdate(0), updateInterval(1000),
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
                                DEFAULT_TEMP_KP, DEFAULT_TEMP_KI, DEFAULT_TEMP_KD, DIRECT);
        
        if (temperaturePID != nullptr) {
            temperaturePID->SetMode(AUTOMATIC);
            temperaturePID->SetOutputLimits(-100, 100); // -100 = full cooling, +100 = full heating
            Serial.println("Temperature PID initialized successfully");
        }
        
        Serial.println("Initializing Humidity PID controller");
        humidityPID = new PID(&humInput, &humOutput, &humSetpoint,
                             DEFAULT_HUM_KP, DEFAULT_HUM_KI, DEFAULT_HUM_KD, DIRECT);
        
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
    const float hysteresis = 0.5; // Small hysteresis for precise humidity control
    
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
