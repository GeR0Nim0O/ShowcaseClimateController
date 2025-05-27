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

ClimateController::ClimateController(PCF8574gpio* gpioExpander, SHT31sensor* tempHumSensor)
    : gpio(gpioExpander), sensor(tempHumSensor),
      temperatureSetpoint(22.0), humiditySetpoint(50.0),
      currentTemperature(0.0), currentHumidity(0.0),
      climateMode(ClimateMode::AUTO), humidityMode(HumidityMode::AUTO),
      heatingActive(false), coolingActive(false), 
      humidifyingActive(false), dehumidifyingActive(false),
      tempControlEnabled(false), lastUpdate(0), updateInterval(1000) {
    
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
        return;
    }
    
    temperaturePID->Compute();
    
    switch (climateMode) {
        case ClimateMode::HEATING:
            heatingActive = (tempOutput > 0);
            coolingActive = false;
            tempControlEnabled = heatingActive;
            break;
            
        case ClimateMode::COOLING:
            heatingActive = false;
            coolingActive = (tempOutput < 0);
            tempControlEnabled = coolingActive;
            break;
            
        case ClimateMode::AUTO:
            if (tempOutput > 5) { // Deadband to prevent oscillation
                heatingActive = true;
                coolingActive = false;
            } else if (tempOutput < -5) {
                heatingActive = false;
                coolingActive = true;
            } else {
                heatingActive = false;
                coolingActive = false;
            }
            tempControlEnabled = (heatingActive || coolingActive);
            break;
            
        case ClimateMode::OFF:
        default:
            tempControlEnabled = false;
            heatingActive = false;
            coolingActive = false;
            break;
    }
}

void ClimateController::updateHumidityControl() {
    if (humidityMode == HumidityMode::OFF) {
        humidifyingActive = false;
        dehumidifyingActive = false;
        return;
    }
    
    humidityPID->Compute();
    
    switch (humidityMode) {
        case HumidityMode::HUMIDIFYING:
            humidifyingActive = (humOutput > 0);
            dehumidifyingActive = false;
            break;
            
        case HumidityMode::DEHUMIDIFYING:
            humidifyingActive = false;
            dehumidifyingActive = (humOutput < 0);
            break;
            
        case HumidityMode::AUTO:
            if (humOutput > 5) { // Deadband to prevent oscillation
                humidifyingActive = true;
                dehumidifyingActive = false;
            } else if (humOutput < -5) {
                humidifyingActive = false;
                dehumidifyingActive = true;
            } else {
                humidifyingActive = false;
                dehumidifyingActive = false;
            }
            break;
            
        case HumidityMode::OFF:
        default:
            humidifyingActive = false;
            dehumidifyingActive = false;
            break;
    }
}

void ClimateController::applyTemperatureControl() {
    gpio->writePin(pinTemperatureEnable, tempControlEnabled);
    gpio->writePin(pinTemperatureHeat, heatingActive);
    gpio->writePin(pinTemperatureCool, coolingActive);
}

void ClimateController::applyHumidityControl() {
    gpio->writePin(pinHumidify, humidifyingActive);
    gpio->writePin(pinDehumidify, dehumidifyingActive);
}

void ClimateController::setTemperaturePID(double kp, double ki, double kd) {
    temperaturePID->SetTunings(kp, ki, kd);
}

void ClimateController::setHumidityPID(double kp, double ki, double kd) {
    humidityPID->SetTunings(kp, ki, kd);
}

void ClimateController::setFanInterior(bool enable) {
    gpio->setFanInterior(enable);
}

void ClimateController::setFanExterior(bool enable) {
    gpio->setFanExterior(enable);
}

bool ClimateController::checkSafetyLimits() {
    return (currentTemperature >= MIN_TEMPERATURE && currentTemperature <= MAX_TEMPERATURE &&
            currentHumidity >= MIN_HUMIDITY && currentHumidity <= MAX_HUMIDITY);
}

void ClimateController::emergencyShutdown() {
    gpio->setTemperatureEnable(false);
    gpio->setTemperatureHeat(false);
    gpio->setTemperatureCool(false);
    gpio->setHumidify(false);
    gpio->setDehumidify(false);
    
    heatingActive = false;
    coolingActive = false;
    humidifyingActive = false;
    dehumidifyingActive = false;
    tempControlEnabled = false;
    
    Serial.println("EMERGENCY SHUTDOWN: Safety limits exceeded!");
}
