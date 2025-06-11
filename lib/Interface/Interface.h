#ifndef INTERFACE_H
#define INTERFACE_H

#include <Arduino.h>
#include "Display.h"
#include "DFR0554Display.h"
#include "RotaryEncoder.h"
#include "ClimateController.h"

class Interface {
public:
    // Interface coordination methods
    Interface();
    ~Interface();
    
    bool begin();
    void update();
    
    // Interface-specific methods
    void setClimateController(ClimateController* controller);
    void setDisplay(Display* display);
    void setEncoder(RotaryEncoder* encoder);
    
    // Menu system
    enum MenuState {
        MENU_DEFAULT = 0,
        MENU_TEMP_SETPOINT,
        MENU_HUMIDITY_SETPOINT,
        MENU_TEMP_CONTROL_ENABLE,
        MENU_HUMIDITY_CONTROL_ENABLE,
        MENU_COUNT // Keep this last for menu cycling
    };
      void handleEncoderButton();
    void handleEncoderRotation();
    void updateDisplay();
    void resetToDefault();
    
    // Public display methods
    void updateClimateDisplay();
    void showStartupMessage();
    bool isDisplayAvailable();
    
    // Configuration
    void setTimeoutMs(unsigned long timeout) { timeoutMs = timeout; }
    void setAdjustmentStep(float step) { adjustmentStep = step; }    
private:
    // Device references
    ClimateController* climateController;
    Display* display;
    DFR0554Display* dfr0554DisplayPtr;
    RotaryEncoder* encoder;
    
    // Menu system state
    MenuState currentMenu;
    unsigned long lastActivityTime;
    unsigned long timeoutMs;
    bool menuActive;
    
    // Encoder state tracking
    int lastEncoderValue;
    bool lastButtonState;
    
    // AutoTune state tracking
    bool previousAutoTuneActive;
    unsigned long autoTuneCompleteTime;
    bool showingAutoTuneComplete;
    
    // Adjustment parameters
    float adjustmentStep;
      // Display helpers
    void displayDefault();
    void displayTempSetpoint();
    void displayHumiditySetpoint();
    void displayTempControlEnable();
    void displayHumidityControlEnable();
    
    // Display wrapper methods
    void displayClear();
    void displaySetCursor(int col, int row);
    void displayPrint(const String& text);
    bool displayIsConnected();
    void displayUpdate();
      // Menu navigation
    void nextMenu();
    void adjustCurrentSetting(int direction);
    
    // Utility methods
    String formatTemperature(float temp);
    String formatHumidity(float humidity);
    String formatOnOff(bool state);
    String formatTemperatureStatus();
    String formatHumidityStatus();
    String formatAutoTuneStatus();
    void updateActivity();
    
    // Device validation
    bool validateDevices();
    
    // Configuration persistence
    void saveSettingsToConfig();
};

#endif // INTERFACE_H
