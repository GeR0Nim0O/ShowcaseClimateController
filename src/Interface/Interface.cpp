#include "Interface.h"
#include "DeviceRegistry/DeviceRegistry.h"
#include "DFR0554Display.h"
#include <Arduino.h>

Interface::Interface()
    : climateController(nullptr),
      display(nullptr),
      dfr0554DisplayPtr(nullptr),
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
    DeviceRegistry& registry = DeviceRegistry::getInstance();    // Find display device
    if (!display) {
        Device* displayDevice = registry.getDeviceByType("Display");
        if (displayDevice) {
            // Check device type to determine which display type it is
            String deviceType = displayDevice->getType();
            if (deviceType == "Display") {            display = static_cast<Display*>(displayDevice);
            } else if (deviceType == "DFR0554Display") {
                dfr0554DisplayPtr = static_cast<DFR0554Display*>(displayDevice);
            }
        }
        
        if (!display && !dfr0554DisplayPtr) {
            return false;
        }
    }
      // Find rotary encoder device
    if (!encoder) {
        Serial.println("Interface: Looking for RotaryEncoder device...");
        encoder = static_cast<RotaryEncoder*>(registry.getDeviceByType("RotaryEncoder"));
        if (!encoder) {
            Serial.println("Interface: No RotaryEncoder device found");
            return false;
        }
        Serial.println("Interface: RotaryEncoder device found!");
    }
    
    // Validate all devices are connected
    if (!validateDevices()) {
        Serial.println("Interface: Device validation failed");
        return false;
    }    // Initialize encoder state    if (encoder->isConnected()) {
        Serial.println("Interface: Encoder is connected, initializing state...");
        
        // Reset encoder to center position (100) to avoid hitting limits
        encoder->setEncoderValue(100);
        delay(50); // Allow time for the write to complete
        
        lastEncoderValue = encoder->getPosition();
        lastButtonState = encoder->isButtonPressed();
        Serial.printf("Interface: Encoder reset to center position: %d, button: %s\n", 
                     lastEncoderValue, lastButtonState ? "pressed" : "released");
    } else {
        Serial.println("Interface: WARNING - Encoder is not connected!");
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
    if (encoder) {
        encoder->update();
    }
    displayUpdate();
    
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
        Serial.println("Interface: Button pressed!");
        updateActivity();
        nextMenu();
    }
    
    lastButtonState = currentButtonState;
}

void Interface::handleEncoderRotation() {
    if (!encoder || !encoder->isConnected()) return;
    
    int currentValue = encoder->getPosition();    int delta = currentValue - lastEncoderValue;
    
    if (delta != 0) {
        Serial.printf("Interface: Encoder moved! Position: %d, Delta: %d\n", currentValue, delta);
        updateActivity();
        
        // Only adjust settings when in a menu (not default screen)
        if (menuActive && currentMenu != MENU_DEFAULT) {
            adjustCurrentSetting(delta > 0 ? 1 : -1);
        }
        
        lastEncoderValue = currentValue;
    }
}

void Interface::updateDisplay() {
    if (!displayIsConnected()) return;
    
    // Static variable to track last menu state
    static MenuState lastDisplayedMenu = MENU_COUNT; // Invalid initial value to force first update
    
    // Only update if menu changed or we're in default mode (which has its own change detection)
    if (currentMenu != lastDisplayedMenu || currentMenu == MENU_DEFAULT) {
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
        
        // Update cached menu state only if we actually switched menus
        if (currentMenu != lastDisplayedMenu) {
            lastDisplayedMenu = currentMenu;
            Serial.printf("Interface: Display updated for menu %d\n", static_cast<int>(currentMenu));
        }
    }
}

void Interface::resetToDefault() {
    currentMenu = MENU_DEFAULT;
    menuActive = false;
    updateActivity();
}

void Interface::displayDefault() {
    if (!climateController) {
        // Static variable to track if error message was already shown
        static bool errorShown = false;
        if (!errorShown) {
            displayClear();
            displaySetCursor(0, 0);
            displayPrint("No Climate Ctrl");
            displaySetCursor(0, 1);
            displayPrint("Available");
            errorShown = true;
        }
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
    // Static variables to cache last displayed content
    static float lastDisplayedTemp = -999.0;
    static float lastDisplayedHum = -999.0;
    static bool lastAutoTuneActive = false;
    static bool lastShowingComplete = false;
    static String lastTempStatus = "";
    static String lastHumStatus = "";
    static bool firstDefaultDisplay = true;
    
    // Check if content has changed
    const float tolerance = 0.01;
    bool tempChanged = abs(currentTemp - lastDisplayedTemp) > tolerance;
    bool humChanged = abs(currentHum - lastDisplayedHum) > tolerance;
    bool autoTuneStatusChanged = (currentAutoTuneActive != lastAutoTuneActive);
    bool completeMsgChanged = (showingAutoTuneComplete != lastShowingComplete);
    
    // Get status strings for comparison
    String tempStatus = formatTemperatureStatus();
    String humStatus = formatHumidityStatus();
    bool statusChanged = (tempStatus != lastTempStatus || humStatus != lastHumStatus);
    
    // Only update display if something actually changed or first time
    if (firstDefaultDisplay || tempChanged || humChanged || autoTuneStatusChanged || 
        completeMsgChanged || statusChanged) {
        
        // Line 1: Current temperature and humidity
        displayClear();
        displaySetCursor(0, 0);
        displayPrint("T:" + formatTemperature(currentTemp));
        displaySetCursor(8, 0);
        displayPrint("RH:" + formatHumidity(currentHum));    
        // Line 2: Show AutoTune status, completion message, or normal control status
        displaySetCursor(0, 1);    if (showingAutoTuneComplete) {
            // Show completion message for 3 seconds
            displayPrint("AutoTune");
            if (millis() - autoTuneCompleteTime > 3000) {
                showingAutoTuneComplete = false;
            }
        } else if (currentAutoTuneActive) {
            // Show AutoTune in progress
            displayPrint(formatAutoTuneStatus());    } else {
            // Show normal control status
            displayPrint("T:" + tempStatus + " RH:" + humStatus);
        }
        
        // Update cached values
        lastDisplayedTemp = currentTemp;
        lastDisplayedHum = currentHum;
        lastAutoTuneActive = currentAutoTuneActive;
        lastShowingComplete = showingAutoTuneComplete;
        lastTempStatus = tempStatus;
        lastHumStatus = humStatus;
        firstDefaultDisplay = false;
        
        Serial.printf("Interface: Default display updated - T=%.1f°C, RH=%.0f%%\n", 
                      currentTemp, currentHum);
    }
}

void Interface::displayTempSetpoint() {
    if (!climateController) return;
    
    float setpoint = climateController->getTemperatureSetpoint();
    
    displayClear();
    displaySetCursor(0, 0);
    displayPrint("Temp Setpoint:");
    displaySetCursor(0, 1);
    displayPrint(formatTemperature(setpoint) + " C");
    displaySetCursor(10, 1);
    displayPrint("ADJUST");
}

void Interface::displayHumiditySetpoint() {
    if (!climateController) return;
    
    float setpoint = climateController->getHumiditySetpoint();
    
    displayClear();
    displaySetCursor(0, 0);
    displayPrint("RH Setpoint:");
    displaySetCursor(0, 1);
    displayPrint(formatHumidity(setpoint) + "%");
    displaySetCursor(10, 1);
    displayPrint("ADJUST");
}

void Interface::displayTempControlEnable() {
    if (!climateController) return;
    
    bool enabled = climateController->isTemperatureControlEnabled();
    
    displayClear();
    displaySetCursor(0, 0);
    displayPrint("Temp Control:");
    displaySetCursor(0, 1);
    displayPrint(formatOnOff(enabled));
    displaySetCursor(10, 1);
    displayPrint("TOGGLE");
}

void Interface::displayHumidityControlEnable() {    if (!climateController) return;
    
    bool enabled = climateController->isHumidityControlEnabled();
    
    displayClear();
    displaySetCursor(0, 0);
    displayPrint("RH Control:");
    displaySetCursor(0, 1);
    displayPrint(formatOnOff(enabled));
    displaySetCursor(10, 1);
    displayPrint("TOGGLE");
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
    return (displayIsConnected() && 
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

// Display wrapper methods to handle both Display and DFR0554Display
void Interface::displayClear() {
    if (display) {
        display->clear();
    } else if (dfr0554DisplayPtr) {
        dfr0554DisplayPtr->clear();
    }
}

void Interface::displaySetCursor(int col, int row) {
    if (display) {
        display->setCursor(col, row);
    } else if (dfr0554DisplayPtr) {
        dfr0554DisplayPtr->setCursor(col, row);
    }
}

void Interface::displayPrint(const String& text) {
    if (display) {
        display->print(text);
    } else if (dfr0554DisplayPtr) {
        dfr0554DisplayPtr->print(text);
    }
}

bool Interface::displayIsConnected() {
    if (display) {
        return display->isConnected();
    } else if (dfr0554DisplayPtr) {
        return dfr0554DisplayPtr->isConnected();
    }
    return false;
}

void Interface::displayUpdate() {
    if (display) {
        display->update();
    } else if (dfr0554DisplayPtr) {
        dfr0554DisplayPtr->update();
    }
}

// Public display methods for external use
void Interface::updateClimateDisplay() {
    if (!displayIsConnected() || !climateController) {
        return;
    }
    
    // Get current climate data
    float currentTemp = climateController->getCurrentTemperature();
    float currentHum = climateController->getCurrentHumidity();
    float tempSetpoint = climateController->getTemperatureSetpoint();
    float humSetpoint = climateController->getHumiditySetpoint();
    
    // Static variables to track last displayed values
    static float lastDisplayedTemp = -999.0;
    static float lastDisplayedHum = -999.0;
    static float lastDisplayedTempSetpoint = -999.0;
    static float lastDisplayedHumSetpoint = -999.0;
    static bool firstUpdate = true;
    
    // Check if any values have changed (with small tolerance for floating point comparison)
    const float tolerance = 0.01; // 0.01 degree/percent tolerance
    bool tempChanged = abs(currentTemp - lastDisplayedTemp) > tolerance;
    bool humChanged = abs(currentHum - lastDisplayedHum) > tolerance;
    bool tempSetpointChanged = abs(tempSetpoint - lastDisplayedTempSetpoint) > tolerance;
    bool humSetpointChanged = abs(humSetpoint - lastDisplayedHumSetpoint) > tolerance;
    
    // Only update display if values have changed or this is the first update
    if (firstUpdate || tempChanged || humChanged || tempSetpointChanged || humSetpointChanged) {
        // Create formatted display strings
        String line1 = String(currentTemp, 1) + "C/" + String(tempSetpoint, 1) + "C";
        String line2 = String(currentHum, 0) + "%/" + String(humSetpoint, 0) + "%";
        
        // Update display
        displayClear();
        displaySetCursor(0, 0);
        displayPrint(line1);
        displaySetCursor(0, 1);
        displayPrint(line2);
        
        // Update cached values
        lastDisplayedTemp = currentTemp;
        lastDisplayedHum = currentHum;
        lastDisplayedTempSetpoint = tempSetpoint;
        lastDisplayedHumSetpoint = humSetpoint;
        firstUpdate = false;
        
        // Debug output for display updates
        Serial.printf("Interface: Climate display updated: T=%.1f°C/%.1f°C, H=%.0f%%/%.0f%%\n", 
                      currentTemp, tempSetpoint, currentHum, humSetpoint);
    }
}

void Interface::showStartupMessage() {
    if (!displayIsConnected()) {
        return;
    }
    
    displayClear();
    displaySetCursor(0, 0);
    displayPrint("Climate Control");
    displaySetCursor(0, 1);
    displayPrint("Initializing...");
}

bool Interface::isDisplayAvailable() {
    return displayIsConnected();
}
