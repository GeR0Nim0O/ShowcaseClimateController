#include "Interface.h"
#include "DeviceRegistry.h"
#include <Arduino.h>

Interface::Interface(TwoWire* wire, uint8_t address, uint8_t tcaPort, 
                    const std::map<String, String>& channels, int deviceIndex)
    : Device(wire, address, tcaPort, channels, deviceIndex, "Interface"),
      climateController(nullptr),
      display(nullptr),
      encoder(nullptr),
      currentMenu(MENU_DEFAULT),
      lastActivityTime(0),
      timeoutMs(10000), // 10 seconds default timeout
      menuActive(false),
      lastEncoderValue(0),
      lastButtonState(false),
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
        lastEncoderValue = encoder->getEncoderValue();
        lastButtonState = encoder->isButtonPressed();
    }
    
    // Initialize display
    resetToDefault();
    
    Serial.println("Interface: Initialized successfully");
    return true;
}

bool Interface::isConnected() {
    return validateDevices();
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

std::map<String, String> Interface::readData() {
    std::map<String, String> data;
    data["menu_state"] = String(static_cast<int>(currentMenu));
    data["menu_active"] = menuActive ? "1" : "0";
    data["last_activity"] = String(millis() - lastActivityTime);
    return data;
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
    
    int currentValue = encoder->getEncoderValue();
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
    
    // Line 1: Current temperature and humidity
    display->clear();
    display->setCursor(0, 0);
    display->print("T:" + formatTemperature(currentTemp));
    display->setCursor(8, 0);
    display->print("H:" + formatHumidity(currentHum));
    
    // Line 2: Setpoints and status
    display->setCursor(0, 1);
    String tempStatus = climateController->isTemperatureControlEnabled() ? "ON" : "OFF";
    String humStatus = climateController->isHumidityControlEnabled() ? "ON" : "OFF";
    display->print("T:" + tempStatus + " H:" + humStatus);
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
    display->print("Humidity Setpt:");
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
    display->print("Humidity Ctrl:");
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
    
    switch (currentMenu) {
        case MENU_TEMP_SETPOINT: {
            float current = climateController->getTemperatureSetpoint();
            float newValue = current + (direction * adjustmentStep);
            // Clamp to reasonable limits
            newValue = constrain(newValue, 10.0f, 40.0f);
            climateController->setTemperatureSetpoint(newValue);
            Serial.print("Interface: Temperature setpoint adjusted to ");
            Serial.println(newValue);
            break;
        }
        
        case MENU_HUMIDITY_SETPOINT: {
            float current = climateController->getHumiditySetpoint();
            float newValue = current + (direction * adjustmentStep);
            // Clamp to reasonable limits
            newValue = constrain(newValue, 30.0f, 90.0f);
            climateController->setHumiditySetpoint(newValue);
            Serial.print("Interface: Humidity setpoint adjusted to ");
            Serial.println(newValue);
            break;
        }
        
        case MENU_TEMP_CONTROL_ENABLE: {
            bool current = climateController->isTemperatureControlEnabled();
            climateController->setTemperatureControlEnabled(!current);
            Serial.print("Interface: Temperature control ");
            Serial.println(!current ? "enabled" : "disabled");
            break;
        }
        
        case MENU_HUMIDITY_CONTROL_ENABLE: {
            bool current = climateController->isHumidityControlEnabled();
            climateController->setHumidityControlEnabled(!current);
            Serial.print("Interface: Humidity control ");
            Serial.println(!current ? "enabled" : "disabled");
            break;
        }
        
        default:
            break;
    }
}

String Interface::formatTemperature(float temp) {
    if (isnan(temp)) return "---";
    return String(temp, 1);
}

String Interface::formatHumidity(float humidity) {
    if (isnan(humidity)) return "---";
    return String(humidity, 1);
}

String Interface::formatOnOff(bool state) {
    return state ? "ON " : "OFF";
}

void Interface::updateActivity() {
    lastActivityTime = millis();
}

bool Interface::validateDevices() {
    return (display && display->isConnected() && 
            encoder && encoder->isConnected());
}

// Device factory registration
bool registerInterfaceDevice() {
    return DeviceRegistry::registerDeviceType("Interface", "",
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) -> Device* {
            return new Interface(wire, address, tcaPort, channels, deviceIndex);
        });
}
