#include "Interface.h"
#include "DeviceRegistry/DeviceRegistry.h"
#include <Arduino.h>

Interface::Interface()
    : climateController(nullptr),
      display(nullptr),
      encoder(nullptr),
      currentMenu(MENU_DEFAULT),
      lastActivityTime(0),
      timeoutMs(10000), // 10 seconds default timeout
      menuActive(false),
      lastEncoderValue(0),
      lastButtonState(false),
      previousAutoTuneActive(false),
      autoTuneCompleteTime(0),
      showingAutoTuneComplete(false),
      adjustmentStep(1.0) {
}

Interface::~Interface() {
    // Destructor - devices are managed by DeviceRegistry
}

bool Interface::begin() {
    // Find and configure display and encoder devices
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    
    // Find display device
    if (!display) {
        display = static_cast<Display*>(registry.getDeviceByType("Display"));
        if (!display) {
            Serial.println("Interface: No Display device found");
            return false;
        }
    }
    
    // Find rotary encoder device
    if (!encoder) {
        encoder = static_cast<RotaryEncoder*>(registry.getDeviceByType("RotaryEncoder"));
        if (!encoder) {
            Serial.println("Interface: No RotaryEncoder device found");
            return false;
        }
    }
    
    // Validate all devices are connected
    if (!validateDevices()) {
        Serial.println("Interface: Device validation failed");
        return false;
    }
      // Initialize encoder state
    if (encoder->isConnected()) {
        lastEncoderValue = encoder->getPosition();
        lastButtonState = encoder->isButtonPressed();
    }
    
    // Initialize display
    resetToDefault();
      Serial.println("Interface: Initialized successfully");
    return true;
}

void Interface::update() {
    if (!validateDevices()) {
        return;
    }
    
    // Update encoder and display devices
    encoder->update();
    display->update();
    
    // Handle encoder inputs
    handleEncoderButton();
    handleEncoderRotation();
    
    // Check for timeout
    if (menuActive && (millis() - lastActivityTime > timeoutMs)) {
        resetToDefault();
    }
    
    // Update display
    updateDisplay();
}

void Interface::setClimateController(ClimateController* controller) {
    climateController = controller;
}

void Interface::setDisplay(Display* disp) {
    display = disp;
}

void Interface::setEncoder(RotaryEncoder* enc) {
    encoder = enc;
}

void Interface::handleEncoderButton() {
    if (!encoder || !encoder->isConnected()) return;
    
    bool currentButtonState = encoder->isButtonPressed();
    
    // Detect button press (rising edge)
    if (currentButtonState && !lastButtonState) {
        updateActivity();
        nextMenu();
    }
    
    lastButtonState = currentButtonState;
}

void Interface::handleEncoderRotation() {
    if (!encoder || !encoder->isConnected()) return;
    
    int currentValue = encoder->getPosition();
    int delta = currentValue - lastEncoderValue;
    
    if (delta != 0) {
        updateActivity();
        
        // Only adjust settings when in a menu (not default screen)
        if (menuActive && currentMenu != MENU_DEFAULT) {
            adjustCurrentSetting(delta > 0 ? 1 : -1);
        }
        
        lastEncoderValue = currentValue;
    }
}

void Interface::updateDisplay() {
    if (!display || !display->isConnected()) return;
    
    switch (currentMenu) {
        case MENU_DEFAULT:
            displayDefault();
            break;
        case MENU_TEMP_SETPOINT:
            displayTempSetpoint();
            break;
        case MENU_HUMIDITY_SETPOINT:
            displayHumiditySetpoint();
            break;
        case MENU_TEMP_CONTROL_ENABLE:
            displayTempControlEnable();
            break;
        case MENU_HUMIDITY_CONTROL_ENABLE:
            displayHumidityControlEnable();
            break;
    }
}

void Interface::resetToDefault() {
    currentMenu = MENU_DEFAULT;
    menuActive = false;
    updateActivity();
}

void Interface::displayDefault() {
    if (!climateController) {
        display->clear();
        display->setCursor(0, 0);
        display->print("No Climate Ctrl");
        display->setCursor(0, 1);
        display->print("Available");
        return;
    }
    
    // Get current readings from climate controller
    float currentTemp = climateController->getCurrentTemperature();
    float currentHum = climateController->getCurrentHumidity();
    
    // Check AutoTune status and handle state transitions
    bool currentAutoTuneActive = climateController->isAutoTuning();
    
    // Detect AutoTune completion
    if (previousAutoTuneActive && !currentAutoTuneActive) {
        // AutoTune just completed
        showingAutoTuneComplete = true;
        autoTuneCompleteTime = millis();
    }
    previousAutoTuneActive = currentAutoTuneActive;
    
    // Line 1: Current temperature and humidity
    display->clear();
    display->setCursor(0, 0);
    display->print("T:" + formatTemperature(currentTemp));
    display->setCursor(8, 0);
    display->print("RH:" + formatHumidity(currentHum));
    
    // Line 2: Show AutoTune status, completion message, or normal control status
    display->setCursor(0, 1);    if (showingAutoTuneComplete) {
        // Show completion message for 3 seconds
        display->print("AutoTune");
        if (millis() - autoTuneCompleteTime > 3000) {
            showingAutoTuneComplete = false;
        }
    } else if (currentAutoTuneActive) {
        // Show AutoTune in progress
        display->print(formatAutoTuneStatus());
    } else {
        // Show normal control status
        String tempStatus = formatTemperatureStatus();
        String humStatus = formatHumidityStatus();
        display->print("T:" + tempStatus + " RH:" + humStatus);
    }
}

void Interface::displayTempSetpoint() {
    if (!climateController) return;
    
    float setpoint = climateController->getTemperatureSetpoint();
    
    display->clear();
    display->setCursor(0, 0);
    display->print("Temp Setpoint:");
    display->setCursor(0, 1);
    display->print(formatTemperature(setpoint) + " C");
    display->setCursor(10, 1);
    display->print("ADJUST");
}

void Interface::displayHumiditySetpoint() {
    if (!climateController) return;
    
    float setpoint = climateController->getHumiditySetpoint();
    
    display->clear();
    display->setCursor(0, 0);
    display->print("RH Setpoint:");
    display->setCursor(0, 1);
    display->print(formatHumidity(setpoint) + "%");
    display->setCursor(10, 1);
    display->print("ADJUST");
}

void Interface::displayTempControlEnable() {
    if (!climateController) return;
    
    bool enabled = climateController->isTemperatureControlEnabled();
    
    display->clear();
    display->setCursor(0, 0);
    display->print("Temp Control:");
    display->setCursor(0, 1);
    display->print(formatOnOff(enabled));
    display->setCursor(10, 1);
    display->print("TOGGLE");
}

void Interface::displayHumidityControlEnable() {
    if (!climateController) return;
    
    bool enabled = climateController->isHumidityControlEnabled();
    
    display->clear();
    display->setCursor(0, 0);
    display->print("RH Control:");
    display->setCursor(0, 1);
    display->print(formatOnOff(enabled));
    display->setCursor(10, 1);
    display->print("TOGGLE");
}

void Interface::nextMenu() {
    // Cycle through menus
    int nextMenuIndex = (static_cast<int>(currentMenu) + 1) % MENU_COUNT;
    currentMenu = static_cast<MenuState>(nextMenuIndex);
    
    // Set menu active flag (except for default)
    menuActive = (currentMenu != MENU_DEFAULT);
    
    Serial.print("Interface: Switched to menu ");
    Serial.println(static_cast<int>(currentMenu));
}

void Interface::adjustCurrentSetting(int direction) {
    if (!climateController) return;
    
    bool settingChanged = false;
    
    switch (currentMenu) {        case MENU_TEMP_SETPOINT: {
            float current = climateController->getTemperatureSetpoint();
            float newValue = current + (direction * 0.1);  // 0.1°C steps for temperature
            // Clamp to reasonable limits
            newValue = constrain(newValue, 10.0f, 40.0f);
            if (newValue != current) {
                climateController->setTemperatureSetpoint(newValue);
                settingChanged = true;
                Serial.print("Interface: Temperature setpoint adjusted to ");
                Serial.println(newValue);
            }
            break;
        }
        
        case MENU_HUMIDITY_SETPOINT: {
            float current = climateController->getHumiditySetpoint();
            float newValue = current + (direction * 1.0);  // 1% steps for humidity
            // Clamp to reasonable limits
            newValue = constrain(newValue, 30.0f, 90.0f);
            if (newValue != current) {
                climateController->setHumiditySetpoint(newValue);
                settingChanged = true;
                Serial.print("Interface: Humidity setpoint adjusted to ");
                Serial.println(newValue);
            }
            break;
        }
        
        case MENU_TEMP_CONTROL_ENABLE: {
            bool current = climateController->isTemperatureControlEnabled();
            climateController->setTemperatureControlEnabled(!current);
            settingChanged = true;
            Serial.print("Interface: Temperature control ");
            Serial.println(!current ? "enabled" : "disabled");
            break;
        }
        
        case MENU_HUMIDITY_CONTROL_ENABLE: {
            bool current = climateController->isHumidityControlEnabled();
            climateController->setHumidityControlEnabled(!current);
            settingChanged = true;
            Serial.print("Interface: Humidity control ");
            Serial.println(!current ? "enabled" : "disabled");
            break;
        }
        
        default:
            break;
    }
    
    // Save changes to configuration file
    if (settingChanged) {
        saveSettingsToConfig();
    }
}

String Interface::formatTemperature(float temp) {
    if (isnan(temp)) return "---";
    return String(temp, 1);
}

String Interface::formatHumidity(float humidity) {
    if (isnan(humidity)) return "---";
    return String((int)round(humidity));
}

String Interface::formatOnOff(bool state) {
    return state ? "ON " : "OFF";
}

String Interface::formatTemperatureStatus() {
    if (!climateController) {
        return "---";
    }
    
    if (!climateController->isTemperatureControlEnabled()) {
        return "OFF";
    }
    
    // Check current heating/cooling status
    if (climateController->isHeating()) {
        return "HEAT";
    } else if (climateController->isCooling()) {
        return "COOL";    } else {
        // Temperature control is enabled but not actively heating or cooling
        return "OK";
    }
}

String Interface::formatHumidityStatus() {
    if (!climateController) {
        return "---";
    }
    
    if (!climateController->isHumidityControlEnabled()) {
        return "OFF";
    }
      // Check current humidifying/dehumidifying status
    if (climateController->isHumidifying()) {
        return "HUM";   // Shortened to fit display
    } else if (climateController->isDehumidifying()) {
        return "DEHUM"; // Shortened to fit display
    } else {
        // Humidity control is enabled but not actively humidifying or dehumidifying
        return "OK";
    }
}

String Interface::formatAutoTuneStatus() {
    return "AutoTune";
}

void Interface::updateActivity() {
    lastActivityTime = millis();
}

bool Interface::validateDevices() {
    return (display && display->isConnected() && 
            encoder && encoder->isConnected());
}

void Interface::saveSettingsToConfig() {
    if (!climateController) {
        Serial.println("Interface: Cannot save settings - no ClimateController available");
        return;
    }
    
    Serial.println("Interface: Saving settings to configuration file...");
    climateController->updateClimateConfigFile();
}
