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
    Serial.println("ClimateController: *** ENTERING begin() method ***");
    Serial.flush();
    delay(100);
    
    // Initialize DAC if available
    if (dac != nullptr) {
        Serial.println("ClimateController: Checking DAC device...");
        
        // Check if DAC is already initialized (to avoid double initialization)
        if (dac->isInitialized()) {
            Serial.println("ClimateController: DAC already initialized, skipping begin()");
            
            // Test DAC connection and functionality
            if (dac->isConnected()) {
                Serial.println("ClimateController: DAC connection verified");
                // Test DAC by setting it to 0V initially
                if (dac->setChannelVoltage(0, 0.0)) {
                    Serial.println("ClimateController: DAC test write successful");
                } else {
                    Serial.println("ClimateController: WARNING - DAC test write failed");
                }
            } else {
                Serial.println("ClimateController: WARNING - DAC not responding to connection test");
            }
        } else {
            Serial.println("ClimateController: DAC not initialized, calling begin()...");
            if (dac->begin()) {
                Serial.println("ClimateController: DAC initialized successfully");
                // Test DAC by setting it to 0V initially
                if (dac->setChannelVoltage(0, 0.0)) {
                    Serial.println("ClimateController: DAC test write successful");
                } else {
                    Serial.println("ClimateController: WARNING - DAC test write failed");
                }
            } else {
                Serial.println("ClimateController: WARNING - DAC initialization failed");
            }
        }
    } else {
        Serial.println("ClimateController: No DAC device available");
    }
    
    // Initialize pin mappings
    initializePinMappings();
    
    // NEW: Ensure GPIO is in output mode and force refresh
    if (gpio != nullptr) {
        Serial.println("ClimateController: Ensuring GPIO is in output mode");
        gpio->forceOutputMode();
        
        // Initialize all control pins to known safe state
        gpio->writePin(pinTemperatureEnable, false);
        gpio->writePin(pinTemperatureHeat, false);
        gpio->writePin(pinTemperatureCool, false);
        gpio->writePin(pinHumidify, false);
        gpio->writePin(pinDehumidify, false);
        
        Serial.println("ClimateController: GPIO pins initialized to safe state");
    }
    
    Serial.println("ClimateController: begin() completed successfully");
    Serial.flush();
    delay(100);
    
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
                    Serial.println("Refreshing fan control during GPIO refresh");
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
    
    // Add debug output to see PID values
    Serial.print("DEBUG: Temperature PID - Input: ");
    Serial.print(tempInput);
    Serial.print("°C, Setpoint: ");
    Serial.print(tempSetpoint);
    Serial.print("°C, Output: ");
    Serial.println(tempOutput);
    
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
    
    // Log output when state changes for debugging
    static bool lastHumidifyingActive = false;
    static bool lastDehumidifyingActive = false;
    
    if (humidifyingActive != lastHumidifyingActive || 
        dehumidifyingActive != lastDehumidifyingActive) {
        
        Serial.print("Humidity control update - Current: ");
        Serial.print(currentHumidity);
        Serial.print("% Setpoint: ");
        Serial.print(humiditySetpoint);
        Serial.print("% Humidifier: ");
        Serial.print(humidifyingActive ? "ON" : "OFF");
        Serial.print(" Dehumidifier: ");
        Serial.println(dehumidifyingActive ? "ON" : "OFF");
        
        lastHumidifyingActive = humidifyingActive;
        lastDehumidifyingActive = dehumidifyingActive;
    }
}

void ClimateController::updateFanControl() {
    if (!autoFanControlEnabled) {
        // If auto fan control is disabled, don't change fan states
        return;
    }
    
    // Determine if ANY climate control is active (temperature OR humidity control)
    bool tempControlActive = (climateMode != ClimateMode::OFF) && 
                            (tempControlEnabled || heatingActive || coolingActive);
    
    bool humidityControlActive = (humidityMode != HumidityMode::OFF) && 
                                (humidifyingActive || dehumidifyingActive);
    
    bool climateControlActive = tempControlActive || humidityControlActive;
    
    // SIMPLIFIED FAN LOGIC: Both fans always on when ANY climate control is active
    fanInteriorActive = climateControlActive;
    fanExteriorActive = climateControlActive;  // Both fans on when climate active
    
    // Debug output when fan states change
    static bool lastFanInteriorActive = false;
    static bool lastFanExteriorActive = false;
    
    if (fanInteriorActive != lastFanInteriorActive || fanExteriorActive != lastFanExteriorActive) {
        Serial.print("Fan control update - Interior: ");
        Serial.print(fanInteriorActive ? "ON" : "OFF");
        Serial.print(", Exterior: ");
        Serial.print(fanExteriorActive ? "ON" : "OFF");
        Serial.print(" (Climate active: ");
        Serial.print(climateControlActive ? "YES" : "NO");
        Serial.print(", Temp control: ");
        Serial.print(tempControlActive ? "YES" : "NO");
        Serial.print(", Humidity control: ");
        Serial.print(humidityControlActive ? "YES" : "NO");
        Serial.println(")");
        
        lastFanInteriorActive = fanInteriorActive;
        lastFanExteriorActive = fanExteriorActive;
    }
}

void ClimateController::applyFanControl() {
    Serial.print("ClimateController: Applying fan control - Interior: ");
    Serial.print(fanInteriorActive ? "ON" : "OFF");
    Serial.print(", Exterior: ");
    Serial.print(fanExteriorActive ? "ON" : "OFF");
    Serial.print(" (pins ");
    Serial.print(pinFanInterior);
    Serial.print(" and ");
    Serial.print(pinFanExterior);
    Serial.println(")");
    
    // Apply interior fan state
    bool interiorResult = safeWritePin(pinFanInterior, fanInteriorActive);
    Serial.print("Interior fan pin ");
    Serial.print(pinFanInterior);
    Serial.print(" result: ");
    Serial.println(interiorResult ? "SUCCESS" : "FAILED");
    
    // Apply exterior fan state
    bool exteriorResult = safeWritePin(pinFanExterior, fanExteriorActive);
    Serial.print("Exterior fan pin ");
    Serial.print(pinFanExterior);
    Serial.print(" result: ");
    Serial.println(exteriorResult ? "SUCCESS" : "FAILED");
    
    // NEW: Additional debug - show GPIO state after fan control
    if (gpio != nullptr) {
        uint8_t gpioState = gpio->getGPIOState();
        Serial.print("GPIO state after fan control: 0x");
        Serial.print(gpioState, HEX);
        Serial.print(" (pin ");
        Serial.print(pinFanInterior);
        Serial.print("=");
        Serial.print((gpioState & (1 << pinFanInterior)) ? "HIGH" : "LOW");
        Serial.print(", pin ");
        Serial.print(pinFanExterior);
        Serial.print("=");
        Serial.print((gpioState & (1 << pinFanExterior)) ? "HIGH" : "LOW");
        Serial.println(")");
    }
}

void ClimateController::applyDACControls() {
    // Skip if no DAC device available
    if (!dac) {
        return; // Reduce log spam
    }
    
    // Rate limiting: Only update DAC if enough time has passed since last update
    static unsigned long lastDACUpdate = 0;
    static float lastDACVoltage = -1.0; // Initialize to invalid value
    const unsigned long DAC_UPDATE_INTERVAL = 1000; // Update DAC max every 1000ms
    const float VOLTAGE_THRESHOLD = 0.1; // 100mV threshold
    
    unsigned long currentTime = millis();
    
    // Calculate desired voltage based on current state
    float desiredVoltage = 0.0;
    String operation = "idle";
    
    if (heatingActive) {
        desiredVoltage = (heatingPower / 100.0) * 5.0;
        operation = "heating";
    } else if (coolingActive) {
        desiredVoltage = (coolingPower / 100.0) * 5.0;
        operation = "cooling";
    }
    
    // Check if we should update (time passed OR significant voltage change)
    bool timeToUpdate = (currentTime - lastDACUpdate >= DAC_UPDATE_INTERVAL);
    bool significantChange = (abs(desiredVoltage - lastDACVoltage) >= VOLTAGE_THRESHOLD);
    bool forceUpdate = (lastDACVoltage < 0); // First time
    
    if (!timeToUpdate && !significantChange && !forceUpdate) {
        return; // Skip this update
    }
    
    // Check if device is still connected before attempting operation
    if (!dac->isConnected()) {
        // Only log connection issues every 10 seconds to reduce spam
        static unsigned long lastConnErrorLog = 0;
        if (currentTime - lastConnErrorLog > 10000) {
            Serial.println("DAC: Device not connected");
            lastConnErrorLog = currentTime;
        }
        return;
    }
    
    // Log what we're about to do
    Serial.print("DAC: Setting ");
    Serial.print(desiredVoltage, 2);
    Serial.print("V (");
    Serial.print(operation);
    if (heatingActive) {
        Serial.print(" at ");
        Serial.print(heatingPower, 1);
        Serial.print("%");
    } else if (coolingActive) {
        Serial.print(" at ");
        Serial.print(coolingPower, 1);
        Serial.print("%");
    }
    Serial.println(")");
    
    // Attempt to set the new voltage
    bool success = dac->setChannelVoltage(0, desiredVoltage);
    
    if (success) {
        lastDACVoltage = desiredVoltage;
        lastDACUpdate = currentTime;
        // Only log success for significant changes to reduce spam
        if (significantChange || forceUpdate) {
            Serial.println("DAC: Voltage set successfully");
        }
    } else {
        Serial.println("DAC: Failed to set voltage");
        // Don't update lastDACUpdate on failure, so we'll try again sooner
    }
}

void ClimateController::setHeatingPower(float percentage) {
    heatingPower = constrain(percentage, 0.0, 100.0);
    if (dac && heatingActive) {
        float voltage = (heatingPower / 100.0) * 5.0; // Changed to 5.0V max
        dac->setChannelVoltage(0, voltage);
    }
}

void ClimateController::setCoolingPower(float percentage) {
    coolingPower = constrain(percentage, 0.0, 100.0);
    if (dac && coolingActive) {
        float voltage = (coolingPower / 100.0) * 5.0; // Changed to 5.0V max
        dac->setChannelVoltage(0, voltage);
    }
}

/*
bool ClimateController::checkSafetyLimits() {
    // Check if temperature and humidity are within safety limits
    bool tempSafe = (currentTemperature >= MIN_TEMPERATURE && currentTemperature <= MAX_TEMPERATURE);
    bool humiditySafe = (currentHumidity >= MIN_HUMIDITY && currentHumidity <= MAX_HUMIDITY);
    
    // Return true if both are safe, false if either is unsafe
    return tempSafe && humiditySafe;
}
*/

void ClimateController::applyTemperatureControl() {
    // Debug output before applying controls
    Serial.print("ClimateController: Applying temperature control - Enable: ");
    Serial.print(tempControlEnabled ? "ON" : "OFF");
    Serial.print(", Heat: ");
    Serial.print(heatingActive ? "ON" : "OFF");
    Serial.print(", Cool: ");
    Serial.println(coolingActive ? "ON" : "OFF");
    
    // Update main temperature enable pin
    bool enableResult = safeWritePin(pinTemperatureEnable, tempControlEnabled);
    Serial.print("Temperature enable pin result: ");
    Serial.println(enableResult ? "SUCCESS" : "FAILED");
    
    // Update heating and cooling pins based on currently active mode
    bool heatResult = safeWritePin(pinTemperatureHeat, heatingActive);
    bool coolResult = safeWritePin(pinTemperatureCool, coolingActive);
    
    Serial.print("Heat pin result: ");
    Serial.print(heatResult ? "SUCCESS" : "FAILED");
    Serial.print(", Cool pin result: ");
    Serial.println(coolResult ? "SUCCESS" : "FAILED");
    
    // Debug output
    if (heatingActive) {
        Serial.print("Temperature control: Heating at ");
        Serial.print(heatingPower);
        Serial.println("%");
    } else if (coolingActive) {
        Serial.print("Temperature control: Cooling at ");
        Serial.print(coolingPower);
        Serial.println("%");
    } else if (!tempControlEnabled) {
        Serial.println("Temperature control: Disabled");
    } else {
        Serial.println("Temperature control: Idle");
    }
}

void ClimateController::applyHumidityControl() {
    // Debug output before applying controls
    Serial.print("ClimateController: Applying humidity control - Humidify: ");
    Serial.print(humidifyingActive ? "ON" : "OFF");
    Serial.print(", Dehumidify: ");
    Serial.println(dehumidifyingActive ? "ON" : "OFF");
    
    // Update humidify and dehumidify pins based on currently active mode
    bool humidifyResult = safeWritePin(pinHumidify, humidifyingActive);
    bool dehumidifyResult = safeWritePin(pinDehumidify, dehumidifyingActive);
    
    Serial.print("Humidify pin result: ");
    Serial.print(humidifyResult ? "SUCCESS" : "FAILED");
    Serial.print(", Dehumidify pin result: ");
    Serial.println(dehumidifyResult ? "SUCCESS" : "FAILED");
    
    // Debug output
    if (humidifyingActive) {
        Serial.println("Humidity control: Humidifying");
    } else if (dehumidifyingActive) {
        Serial.println("Humidity control: Dehumidifying");
    } else {
        Serial.println("Humidity control: Idle");
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
    if (!gpio) {
        Serial.println("ClimateController: GPIO device is null");
        return false;
    }
    
    if (!gpio->isInitialized()) {
        Serial.println("ClimateController: GPIO device not initialized");
        return false;
    }
    
    try {
        Serial.print("ClimateController: Writing pin ");
        Serial.print(pin);
        Serial.print(" = ");
        Serial.print(value ? "HIGH" : "LOW");
        Serial.print(" - ");
        
        gpio->writePin(pin, value);
        
        // Verify the write by reading back the GPIO state
        uint8_t currentState = gpio->getGPIOState();
        bool actualState = (currentState & (1 << pin)) != 0;
        
        if (actualState == value) {
            Serial.println("VERIFIED");
            return true;
        } else {
            Serial.print("VERIFICATION FAILED (expected ");
            Serial.print(value ? "HIGH" : "LOW");
            Serial.print(", got ");
            Serial.print(actualState ? "HIGH" : "LOW");
            Serial.println(")");
            return false;
        }
    } catch (...) {
        Serial.print("ClimateController: Exception writing to pin ");
        Serial.println(pin);
        return false;
    }
}

// Implementation of the testDAC method declared in the header
void ClimateController::testDAC() {
    if (!dac) {
        Serial.println("ClimateController: No DAC device available for testing");
        return;
    }
    
    Serial.println("ClimateController: Starting comprehensive DAC test sequence...");
    
    // First verify DAC is connected and responding
    bool isConnected = dac->isConnected();
    Serial.print("ClimateController: DAC connection test: ");
    Serial.println(isConnected ? "PASSED" : "FAILED");
    
    if (!isConnected) {
        Serial.println("ClimateController: DAC not responding, aborting test");
        return;
    }
    
    try {
        // Test both channels separately with detailed logging
        Serial.println("=== Testing Channel A ===");
        float testVoltagesA[] = {0.0f, 1.0f, 2.5f, 5.0f, 0.0f};
        int numSteps = sizeof(testVoltagesA) / sizeof(testVoltagesA[0]);
        
        for (int i = 0; i < numSteps; i++) {
            float voltage = testVoltagesA[i];
            Serial.print("Setting Channel A to ");
            Serial.print(voltage, 2);
            Serial.println("V");
            
            bool success = dac->setChannelVoltage(0, voltage);
            Serial.print("Result: ");
            Serial.println(success ? "SUCCESS" : "FAILED");
            
            delay(2000); // 2 second hold for measurement
        }
        
        Serial.println("\n=== Testing Channel B ===");
        float testVoltagesB[] = {0.0f, 1.0f, 2.5f, 5.0f, 0.0f};
        
        for (int i = 0; i < numSteps; i++) {
            float voltage = testVoltagesB[i];
            Serial.print("Setting Channel B to ");
            Serial.print(voltage, 2);
            Serial.println("V");
            
            bool success = dac->setChannelVoltage(1, voltage);
            Serial.print("Result: ");
            Serial.println(success ? "SUCCESS" : "FAILED");
            
            delay(2000); // 2 second hold for measurement
        }
        
        // Test both channels simultaneously
        Serial.println("\n=== Testing Both Channels Simultaneously ===");
        Serial.println("Setting both channels to 2.5V");
        uint16_t dacValue = 2047; // Approximately 2.5V
        bool success = dac->setBothChannels(dacValue, dacValue);
        Serial.print("Both channels result: ");
        Serial.println(success ? "SUCCESS" : "FAILED");
        
        delay(3000);
        
        // Reset both channels to 0V
        Serial.println("Resetting both channels to 0V");
        dac->setBothChannels(0, 0);
        
        Serial.println("ClimateController: DAC test completed");
        
    } catch (...) {
        Serial.println("ClimateController: Exception during DAC testing");
        // Try to reset DAC to 0V on error
        try {
            dac->setChannelVoltage(0, 0.0f);
            dac->setChannelVoltage(1, 0.0f);
        } catch (...) {
            Serial.println("ClimateController: Failed to reset DAC after exception");
        }
    }
}

void ClimateController::setFanInterior(bool enable) {
    Serial.print("ClimateController: Manual setting interior fan to ");
    Serial.print(enable ? "ON" : "OFF");
    Serial.print(" (pin ");
    Serial.print(pinFanInterior);
    Serial.print(") - ");
    
    // If auto fan control is enabled, disable it temporarily for manual control
    if (autoFanControlEnabled && !enable) {
        Serial.println("NOTICE: Disabling auto fan control for manual operation");
        autoFanControlEnabled = false;
    }
    
    fanInteriorActive = enable;
    
    if (safeWritePin(pinFanInterior, enable)) {
        Serial.println("SUCCESS");
    } else {
        Serial.println("FAILED");
    }
}

void ClimateController::setFanExterior(bool enable) {
    Serial.print("ClimateController: Manual setting exterior fan to ");
    Serial.print(enable ? "ON" : "OFF");
    Serial.print(" (pin ");
    Serial.print(pinFanExterior);
    Serial.print(") - ");
    
    // If auto fan control is enabled, disable it temporarily for manual control
    if (autoFanControlEnabled && !enable) {
        Serial.println("NOTICE: Disabling auto fan control for manual operation");
        autoFanControlEnabled = false;
    }
    
    fanExteriorActive = enable;
    
    if (safeWritePin(pinFanExterior, enable)) {
        Serial.println("SUCCESS");
    } else {
        Serial.println("FAILED");
    }
}
