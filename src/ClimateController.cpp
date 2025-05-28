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
    : gpio(gpioExpander), sensor(tempHumSensor), dac(dac),
      temperatureSetpoint(22.0), humiditySetpoint(50.0),
      currentTemperature(0.0), currentHumidity(0.0),
      climateMode(ClimateMode::AUTO), humidityMode(HumidityMode::AUTO),
      heatingActive(false), coolingActive(false), 
      humidifyingActive(false), dehumidifyingActive(false),
      tempControlEnabled(false), lastUpdate(0), updateInterval(1000),
      heatingPower(0.0), coolingPower(0.0), 
      humidifierPower(0.0), dehumidifierPower(0.0) {
    
    // Initialize pin mappings from configuration
    initializePinMappings();
    
    // Initialize PID controllers
    temperaturePID = new PID(&tempInput, &tempOutput, &tempSetpoint, 
                            DEFAULT_TEMP_KP, DEFAULT_TEMP_KI, DEFAULT_TEMP_KD, DIRECT);
    humidityPID = new PID(&humInput, &humOutput, &humSetpoint,
                         DEFAULT_HUM_KP, DEFAULT_HUM_KI, DEFAULT_HUM_KD, DIRECT);
    
    temperaturePID->SetMode(AUTOMATIC);
    temperaturePID->SetOutputLimits(-100, 100); // -100 = full cooling, +100 = full heating
    
    humidityPID->SetMode(AUTOMATIC);
    humidityPID->SetOutputLimits(-100, 100); // -100 = full dehumidify, +100 = full humidify
}

bool ClimateController::begin() {
    if (!gpio || !sensor) {
        Serial.println("ClimateController: Invalid device pointers");
        return false;
    }
    
    if (!gpio->isConnected() || !sensor->isConnected()) {
        Serial.println("ClimateController: Devices not connected");
        return false;
    }
    
    // Check if DAC is available but don't fail if it's not
    if (dac) {
        if (!dac->isConnected()) {
            Serial.println("ClimateController: Warning - DAC not connected, using digital control only");
            dac = nullptr; // Disable DAC control if not connected
        } else {
            Serial.println("ClimateController: DAC connected, using analog control");
            // Initialize DAC to zero outputs
            dac->setChannelVoltage(0, 0.0);
            dac->setChannelVoltage(1, 0.0);
        }
    } else {
        Serial.println("ClimateController: No DAC available, using digital control only");
    }
    
    // Initialize all outputs to safe state
    gpio->writePin(pinTemperatureEnable, false);
    gpio->writePin(pinTemperatureHeat, false);
    gpio->writePin(pinTemperatureCool, false);
    gpio->writePin(pinHumidify, false);
    gpio->writePin(pinDehumidify, false);
    
    Serial.println("ClimateController initialized successfully");
    return true;
}

void ClimateController::update() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastUpdate >= updateInterval) {
        updateSensorReadings();
        
        if (checkSafetyLimits()) {
            updateTemperatureControl();
            updateHumidityControl();
            applyTemperatureControl();
            applyHumidityControl();
            applyDACControls(); // Apply DAC controls
        } else {
            emergencyShutdown();
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
            if (tempOutput > 5) { // Deadband to prevent oscillation
                heatingActive = true;
                coolingActive = false;
                heatingPower = map(tempOutput, 5, 100, 0, 100);
                coolingPower = 0.0;
            } else if (tempOutput < -5) {
                heatingActive = false;
                coolingActive = true;
                heatingPower = 0.0;
                coolingPower = map(-tempOutput, 5, 100, 0, 100);
            } else {
                heatingActive = false;
                coolingActive = false;
                heatingPower = 0.0;
                coolingPower = 0.0;
            }
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
        humidifierPower = 0.0;
        dehumidifierPower = 0.0;
        return;
    }
    
    humidityPID->Compute();
    
    switch (humidityMode) {
        case HumidityMode::HUMIDIFYING:
            humidifyingActive = (humOutput > 0);
            dehumidifyingActive = false;
            humidifierPower = humidifyingActive ? map(humOutput, 0, 100, 0, 100) : 0.0;
            dehumidifierPower = 0.0;
            break;
            
        case HumidityMode::DEHUMIDIFYING:
            humidifyingActive = false;
            dehumidifyingActive = (humOutput < 0);
            humidifierPower = 0.0;
            dehumidifierPower = dehumidifyingActive ? map(-humOutput, 0, 100, 0, 100) : 0.0;
            break;
            
        case HumidityMode::AUTO:
            if (humOutput > 5) { // Deadband to prevent oscillation
                humidifyingActive = true;
                dehumidifyingActive = false;
                humidifierPower = map(humOutput, 5, 100, 0, 100);
                dehumidifierPower = 0.0;
            } else if (humOutput < -5) {
                humidifyingActive = false;
                dehumidifyingActive = true;
                humidifierPower = 0.0;
                dehumidifierPower = map(-humOutput, 5, 100, 0, 100);
            } else {
                humidifyingActive = false;
                dehumidifyingActive = false;
                humidifierPower = 0.0;
                dehumidifierPower = 0.0;
            }
            break;
            
        case HumidityMode::OFF:
        default:
            humidifyingActive = false;
            dehumidifyingActive = false;
            humidifierPower = 0.0;
            dehumidifierPower = 0.0;
            break;
    }
}

void ClimateController::applyDACControls() {
    // Skip if no DAC device available
    if (!dac || !dac->isConnected()) return;
    
    // Set temperature power - use channelA for temperature control
    if (heatingActive) {
        dac->setTemperaturePower(heatingPower);
    } else if (coolingActive) {
        dac->setTemperaturePower(coolingPower);
    } else {
        dac->setTemperaturePower(0.0);
    }
    
    // Set humidity power - use channelB for humidity control
    if (humidifyingActive) {
        dac->setHumidityPower(humidifierPower);
    } else if (dehumidifyingActive) {
        dac->setHumidityPower(dehumidifierPower);
    } else {
        dac->setHumidityPower(0.0);
    }
}

void ClimateController::setHeatingPower(float percentage) {
    heatingPower = constrain(percentage, 0.0, 100.0);
    if (dac && heatingActive) {
        dac->setTemperaturePower(heatingPower);
    }
}

void ClimateController::setCoolingPower(float percentage) {
    coolingPower = constrain(percentage, 0.0, 100.0);
    if (dac && coolingActive) {
        dac->setTemperaturePower(coolingPower);
    }
}

void ClimateController::setHumidifierPower(float percentage) {
    humidifierPower = constrain(percentage, 0.0, 100.0);
    if (dac && humidifyingActive) {
        dac->setHumidityPower(humidifierPower);
    }
}

void ClimateController::setDehumidifierPower(float percentage) {
    dehumidifierPower = constrain(percentage, 0.0, 100.0);
    if (dac && dehumidifyingActive) {
        dac->setHumidityPower(dehumidifierPower);
    }
}

void ClimateController::emergencyShutdown() {
    gpio->writePin(pinTemperatureEnable, false);
    gpio->writePin(pinTemperatureHeat, false);
    gpio->writePin(pinTemperatureCool, false);
    gpio->writePin(pinHumidify, false);
    gpio->writePin(pinDehumidify, false);
    
    // Also disable DAC outputs
    if (dac) {
        dac->setChannelVoltage(0, 0.0);
        dac->setChannelVoltage(1, 0.0);
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

// Pin mapping helper methods
void ClimateController::initializePinMappings() {
    // Initialize with default values matching config.json
    pinFanExterior = 0;
    pinFanInterior = 1;
    pinHumidify = 2;
    pinDehumidify = 3;
    pinTemperatureEnable = 4;
    pinTemperatureCool = 5;
    pinTemperatureHeat = 6;
    
    // Try to load from configuration
    pinFanExterior = getPinFromChannelName("FanExterior");
    pinFanInterior = getPinFromChannelName("FanInterior");
    pinHumidify = getPinFromChannelName("Humidify");
    pinDehumidify = getPinFromChannelName("Dehumidify");
    pinTemperatureEnable = getPinFromChannelName("TemperatureEnable");
    pinTemperatureCool = getPinFromChannelName("TemperatureCool");
    pinTemperatureHeat = getPinFromChannelName("TemperatureHeat");
}

uint8_t ClimateController::getPinFromChannelName(const String& channelName) {
    JsonObject devicesConfig = Configuration::getDevicesConfig();
    
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
                        return channelKey.substring(2).toInt();
                    }
                }
            }
        }
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
