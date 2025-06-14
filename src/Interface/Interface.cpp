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
      lastButtonPressTime(0),
      previousAutoTuneActive(false),
      autoTuneCompleteTime(0),
      showingAutoTuneComplete(false),
      autoTuneSelection(0), // Default to Normal AutoTune
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
        Device* displayDevice = registry.getDeviceByType("Display");
        if (displayDevice) {
            // Check device type to determine which display type it is
            String deviceType = displayDevice->getType();
            if (deviceType == "Display") {
                display = static_cast<Display*>(displayDevice);
            } else if (deviceType == "DFR0554Display") {
                dfr0554DisplayPtr = static_cast<DFR0554Display*>(displayDevice);
            }
        }
        
        if (!display && !dfr0554DisplayPtr) {
            return false;
        }    }
      // Find rotary encoder device
    if (!encoder) {
        Serial.println("Interface: Looking for RotaryEncoder device...");
        encoder = static_cast<RotaryEncoder*>(registry.getDeviceByType("RotaryEncoder"));
        if (!encoder) {
            Serial.println("Interface: No RotaryEncoder device found - continuing without encoder");
            // Don't return false - allow interface to work without encoder for display purposes
        } else {
            Serial.println("Interface: RotaryEncoder device found!");
        }
    }
    
    // Validate all devices are connected
    if (!validateDevices()) {
        Serial.println("Interface: Device validation failed");
        return false;
    }    // Initialize encoder state
    if (encoder && encoder->isConnected()) {
        Serial.println("Interface: Encoder is connected, initializing state...");
        
        // Set much lower gain for stability (1 is the lowest)
        encoder->setGainCoefficient(1); // Lowest sensitivity for stability
        delay(100);
        
        // Reset encoder to a safe middle position
        encoder->setEncoderValue(200);
        delay(100); // Allow more time for the write to complete
        
        // Read back the actual values to verify
        lastEncoderValue = encoder->getPosition();
        lastButtonState = encoder->isButtonPressed();
        int actualGain = encoder->getGainCoefficient();
        
        Serial.printf("Interface: Encoder initialized - Position: %d, Button: %s, Gain: %d\n", 
                     lastEncoderValue, lastButtonState ? "pressed" : "released", actualGain);
                     
        // If position is still unstable, try a different approach
        if (lastEncoderValue > 1000 || lastEncoderValue < 0) {
            Serial.println("Interface: Encoder position unstable, trying reset...");
            encoder->setEncoderValue(50);
            delay(100);
            lastEncoderValue = encoder->getPosition();
            Serial.printf("Interface: After reset - Position: %d\n", lastEncoderValue);
        }
    } else {
        Serial.println("Interface: WARNING - Encoder is not connected or not available!");
    }
    
    // Initialize display
    resetToDefault();
    
    Serial.println("Interface: Initialized successfully");
    return true;
}

void Interface::update() {
    if (!validateDevices()) {
        return;
    }      // Update encoder and display devices
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
    unsigned long currentTime = millis();
    
    // Detect button press (rising edge) with debouncing
    if (currentButtonState && !lastButtonState) {
        // Check if enough time has passed since last button press (debouncing)
        if (currentTime - lastButtonPressTime >= BUTTON_DEBOUNCE_MS) {
            Serial.printf("Interface: Button pressed! Menu active: %s, Current menu: %d\n", 
                          menuActive ? "true" : "false", static_cast<int>(currentMenu));
            updateActivity(); // Restart timeout on button interaction
            nextMenu();
            lastButtonPressTime = currentTime; // Update last press time
            Serial.printf("Interface: After nextMenu() - Menu active: %s, New menu: %d\n", 
                          menuActive ? "true" : "false", static_cast<int>(currentMenu));
        } else {
            Serial.printf("Interface: Button press ignored (debouncing) - %lu ms since last press\n", 
                         currentTime - lastButtonPressTime);
        }
    }
    
    lastButtonState = currentButtonState;
}

void Interface::handleEncoderRotation() {
    if (!encoder || !encoder->isConnected()) return;
    
    int currentValue = encoder->getPosition();
    int rawDelta = currentValue - lastEncoderValue;
      // Check for invalid/extreme values that indicate encoder malfunction
    if (currentValue > 32000 || currentValue < -100) {
        // Rate limit malfunction messages to avoid flooding
        static unsigned long lastMalfunctionMessageTime = 0;
        static unsigned long lastResetTime = 0;
        unsigned long currentTime = millis();
        
        // Only print malfunction message once every 10 seconds
        if (currentTime - lastMalfunctionMessageTime > 10000) {
            Serial.printf("Interface: Encoder malfunction detected (%d), attempting recovery\n", currentValue);
            lastMalfunctionMessageTime = currentTime;
        }
        
        // Don't constantly reset - only reset if we haven't reset recently
        if (currentTime - lastResetTime > 1000) { // Only reset once per second
            encoder->setEncoderValue(200);
            delay(50);
            lastResetTime = currentTime;
        }
        lastEncoderValue = encoder->getPosition();
        return; // Skip this update cycle
    }
    
    // Normal range check for typical encoder limits
    if (currentValue > 900 || currentValue < 50) {
        Serial.printf("Interface: Encoder near normal limit (%d), gentle reset\n", currentValue);
        encoder->setEncoderValue(200);
        delay(10);
        lastEncoderValue = encoder->getPosition();
        return;
    }
      if (rawDelta != 0) {
        // Restart timeout on any encoder movement
        updateActivity();
          // Apply scaling - only count every N encoder ticks as one step
        const int ENCODER_SCALE = 4; // Reduced from 8 to 4 for better responsiveness
        static int accumulatedDelta = 0;
        static MenuState lastMenuForAccumulation = MENU_DEFAULT; // Track menu changes
        
        // Reset accumulation if menu changed
        if (currentMenu != lastMenuForAccumulation) {
            accumulatedDelta = 0;
            lastMenuForAccumulation = currentMenu;
            Serial.printf("Interface: Menu changed, resetting encoder accumulation\n");
        }
        
        accumulatedDelta += rawDelta;
        
        // Only trigger when accumulated delta exceeds threshold
        if (abs(accumulatedDelta) >= ENCODER_SCALE) {
            int scaledDelta = accumulatedDelta / ENCODER_SCALE;
            accumulatedDelta = accumulatedDelta % ENCODER_SCALE; // Keep remainder
            
            Serial.printf("Interface: Encoder step! Raw delta: %d, Scaled delta: %d\n", rawDelta, scaledDelta);
            
            // Only adjust settings when in a menu (not default screen)
            if (menuActive && currentMenu != MENU_DEFAULT) {
                adjustCurrentSetting(scaledDelta > 0 ? 1 : -1);
            }
        } else {
            Serial.printf("Interface: Encoder moved (accumulating): Raw delta: %d, Accumulated: %d\n", rawDelta, accumulatedDelta);
        }
        
        lastEncoderValue = currentValue;
    }
}

void Interface::updateDisplay() {
    if (!displayIsConnected()) return;
    
    // Static variables to track last displayed state
    static MenuState lastDisplayedMenu = MENU_COUNT; // Invalid initial value to force first update
    static float lastDisplayedTempSetpoint = -999.0;
    static float lastDisplayedHumSetpoint = -999.0;
    static bool lastTempControlEnabled = false;
    static bool lastHumControlEnabled = false;
    
    // Get current values for comparison
    float currentTempSetpoint = climateController ? climateController->getTemperatureSetpoint() : 0.0;
    float currentHumSetpoint = climateController ? climateController->getHumiditySetpoint() : 0.0;
    bool currentTempControlEnabled = climateController ? climateController->isTemperatureControlEnabled() : false;
    bool currentHumControlEnabled = climateController ? climateController->isHumidityControlEnabled() : false;
    
    // Check if we need to update the display
    bool menuChanged = (currentMenu != lastDisplayedMenu);
    bool tempSetpointChanged = (abs(currentTempSetpoint - lastDisplayedTempSetpoint) > 0.01);
    bool humSetpointChanged = (abs(currentHumSetpoint - lastDisplayedHumSetpoint) > 0.01);
    bool tempControlChanged = (currentTempControlEnabled != lastTempControlEnabled);
    bool humControlChanged = (currentHumControlEnabled != lastHumControlEnabled);
      // Update display if menu changed, default mode (which has its own change detection), or setpoint values changed
    bool shouldUpdate = menuChanged || 
                       (currentMenu == MENU_DEFAULT) ||
                       (currentMenu == MENU_TEMP_SETPOINT && tempSetpointChanged) ||
                       (currentMenu == MENU_HUMIDITY_SETPOINT && humSetpointChanged) ||
                       (currentMenu == MENU_TEMP_CONTROL_ENABLE && tempControlChanged) ||
                       (currentMenu == MENU_HUMIDITY_CONTROL_ENABLE && humControlChanged) ||
                       (currentMenu == MENU_AUTOTUNE);
    
    if (shouldUpdate) {
        switch (currentMenu) {
            case MENU_DEFAULT:
                displayDefault();
                break;
            case MENU_TEMP_SETPOINT:
                displayTempSetpoint();
                if (tempSetpointChanged) {
                    Serial.printf("Interface: Temperature setpoint display updated to %.1f°C\n", currentTempSetpoint);
                }
                break;
            case MENU_HUMIDITY_SETPOINT:
                displayHumiditySetpoint();
                if (humSetpointChanged) {
                    Serial.printf("Interface: Humidity setpoint display updated to %.0f%%\n", currentHumSetpoint);
                }
                break;
            case MENU_TEMP_CONTROL_ENABLE:
                displayTempControlEnable();
                break;            case MENU_HUMIDITY_CONTROL_ENABLE:
                displayHumidityControlEnable();
                break;
            case MENU_AUTOTUNE:
                displayAutoTune();
                break;}
        
        // Update cached values
        if (menuChanged) {
            lastDisplayedMenu = currentMenu;
            Serial.printf("Interface: Display updated for menu %d\n", static_cast<int>(currentMenu));
        }
        if (tempSetpointChanged) {
            lastDisplayedTempSetpoint = currentTempSetpoint;
        }
        if (humSetpointChanged) {
            lastDisplayedHumSetpoint = currentHumSetpoint;
        }
        if (tempControlChanged) {
            lastTempControlEnabled = currentTempControlEnabled;
        }
        if (humControlChanged) {
            lastHumControlEnabled = currentHumControlEnabled;
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
        
        displayClear();
        
        if (showingAutoTuneComplete) {
            // Show completion message for 3 seconds
            displaySetCursor(0, 0);
            displayPrint("AutoTune");
            displaySetCursor(0, 1);
            displayPrint("Complete");
            if (millis() - autoTuneCompleteTime > 3000) {
                showingAutoTuneComplete = false;
            }
        } else if (currentAutoTuneActive) {
            // Show AutoTune in progress
            displaySetCursor(0, 0);
            displayPrint("AutoTune");
            displaySetCursor(0, 1);
            displayPrint("Running...");
        } else {
            // Show normal display format: Temperature on line 1, Humidity on line 2
            // Line 1: T:21.2C Cooling
            displaySetCursor(0, 0);
            displayPrint("T:" + formatTemperature(currentTemp) + " " + tempStatus);
            
            // Line 2: H:55% Humidifying  
            displaySetCursor(0, 1);
            displayPrint("H:" + formatHumidity(currentHum) + " " + humStatus);
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

void Interface::displayAutoTune() {
    if (!climateController) return;
    
    displayClear();
    displaySetCursor(0, 0);
    displayPrint("AutoTune Mode:");
    displaySetCursor(0, 1);
    
    if (climateController->isAutoTuning()) {
        displayPrint("RUNNING...  STOP");
    } else {
        String options[] = {"Normal", "Fast", "Skip"};
        displayPrint(options[autoTuneSelection] + "    START");
    }
}

void Interface::nextMenu() {
    // Cycle through menus
    int nextMenuIndex = (static_cast<int>(currentMenu) + 1) % MENU_COUNT;
    currentMenu = static_cast<MenuState>(nextMenuIndex);
    
    // Set menu active flag (except for default)
    menuActive = (currentMenu != MENU_DEFAULT);
      Serial.printf("Interface: Switched to menu %d (%s), menuActive: %s\n", 
                  static_cast<int>(currentMenu),
                  currentMenu == MENU_DEFAULT ? "DEFAULT" :
                  currentMenu == MENU_TEMP_SETPOINT ? "TEMP_SETPOINT" :
                  currentMenu == MENU_HUMIDITY_SETPOINT ? "HUMIDITY_SETPOINT" :
                  currentMenu == MENU_TEMP_CONTROL_ENABLE ? "TEMP_CONTROL_ENABLE" :
                  currentMenu == MENU_HUMIDITY_CONTROL_ENABLE ? "HUMIDITY_CONTROL_ENABLE" :
                  currentMenu == MENU_AUTOTUNE ? "AUTOTUNE" : "UNKNOWN",
                  menuActive ? "true" : "false");
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
          case MENU_AUTOTUNE: {
            // Handle AutoTune mode selection: Normal (N), Fast (F), or Skip (S)
            if (climateController->isAutoTuning()) {
                // AutoTune is running, allow stop with any direction
                climateController->stopAutoTune();
                Serial.println("Interface: AutoTune stopped by user");
                settingChanged = true;
            } else {
                // Not running, cycle through options or start AutoTune
                if (direction != 0) {
                    // Rotate through options: Normal -> Fast -> Skip -> Normal
                    autoTuneSelection = (autoTuneSelection + (direction > 0 ? 1 : -1) + 3) % 3;
                    Serial.print("Interface: AutoTune selection: ");
                    Serial.println(autoTuneSelection == 0 ? "Normal" : 
                                  autoTuneSelection == 1 ? "Fast" : "Skip");
                } else {
                    // Direction == 0 means button press (start AutoTune)
                    if (autoTuneSelection == 0) {
                        // Normal AutoTune
                        if (climateController->startTemperatureAutoTune()) {
                            Serial.println("Interface: Normal AutoTune started");
                        } else {
                            Serial.println("Interface: Failed to start Normal AutoTune");
                        }
                    } else if (autoTuneSelection == 1) {
                        // Fast AutoTune
                        if (climateController->startTemperatureAutoTuneFast()) {
                            Serial.println("Interface: Fast AutoTune started");
                        } else {
                            Serial.println("Interface: Failed to start Fast AutoTune");
                        }
                    }
                    // Skip selection (2) does nothing
                    settingChanged = true;
                }
            }
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
    return String(temp, 1) + "C";
}

String Interface::formatHumidity(float humidity) {
    if (isnan(humidity)) return "---";
    return String((int)round(humidity)) + "%";
}

String Interface::formatOnOff(bool state) {
    return state ? "ON " : "OFF";
}

String Interface::formatTemperatureStatus() {
    if (!climateController) {
        return "---";
    }
    
    if (!climateController->isTemperatureControlEnabled()) {
        return "Off";
    }
    
    // Check current heating/cooling status
    if (climateController->isHeating()) {
        return "Heating";
    } else if (climateController->isCooling()) {
        return "Cooling";
    } else {
        // Temperature control is enabled but not actively heating or cooling
        return "OK";
    }
}

String Interface::formatHumidityStatus() {
    if (!climateController) {
        return "---";
    }
    
    if (!climateController->isHumidityControlEnabled()) {
        return "Off";
    }
      // Check current humidifying/dehumidifying status
    if (climateController->isHumidifying()) {
        return "Humidify";
    } else if (climateController->isDehumidifying()) {
        return "Dehumidify";
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
    // Only require display to be connected, encoder is optional
    return displayIsConnected();
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
