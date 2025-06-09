# Interface System Documentation

## Overview

The Interface system provides a user-friendly menu-driven interface for the Climate Controller using a combination of an LCD display and rotary encoder. It follows the existing device architecture pattern and integrates seamlessly with the DeviceRegistry system.

## Components

### Interface Class
- **Location**: `lib/Interface/Interface.h`, `src/Interface/Interface.cpp`
- **Type**: Menu coordination class (NOT a device)
- **Purpose**: Coordinates Display and RotaryEncoder devices to provide menu navigation

### Dependencies
- **Display**: 2x16 LCD with I2C backpack (PCF8574)
- **RotaryEncoder**: i2cEncoderLibV2 compatible rotary encoder
- **ClimateController**: For accessing temperature/humidity data and setpoints

## Features

### Menu System
The interface provides a cyclical menu system with the following screens:

1. **Default Screen** (Menu 0)
   - Shows current temperature and humidity readings
   - Shows temperature control status (HEAT/COOL/OK/OFF) and humidity control status (HUM/DEHUM/OK/OFF)   - Shows AutoTune status when active ("PIDtune")
   - Shows completion message ("Tuning Done") for 3 seconds when AutoTune finishes
   - Format: `T:22.5 RH:65` / `T:HEAT RH:OFF` or `PIDtune`

2. **Temperature Setpoint** (Menu 1)
   - Allows adjustment of temperature setpoint (10-40°C)
   - Encoder rotation adjusts value in 0.1°C steps
   - Format: `Temp Setpoint:` / `22.5 C ADJUST`

3. **Humidity Setpoint** (Menu 2)
   - Allows adjustment of humidity setpoint (30-90%)
   - Encoder rotation adjusts value in 1% steps
   - Format: `RH Setpoint:` / `65% ADJUST`

4. **Temperature Control Enable** (Menu 3)
   - Toggle temperature control on/off
   - Format: `Temp Control:` / `ON  TOGGLE`

5. **Humidity Control Enable** (Menu 4)
   - Toggle humidity control on/off
   - Format: `Humidity Ctrl:` / `OFF TOGGLE`

### Navigation
- **Button Press**: Cycles through menus (0→1→2→3→4→0...)
- **Encoder Rotation**: Adjusts values when in setting menus (1-4)
- **Timeout**: Returns to default screen after 10 seconds of inactivity

### Configuration
- **Timeout Duration**: Configurable (default 10 seconds)
- **Adjustment Steps**: Temperature 0.1°C, Humidity 1% (hardcoded)
- **Value Limits**: Temperature (10-40°C), Humidity (30-90%)

### Display Layout Flowchart (2x16 LCD)

The following flowchart shows the actual display content as it appears on a 2x16 character LCD:

```
Display Format (2 rows × 16 characters):
┌────────────────┐
│1234567890123456│ ← Row 1
│1234567890123456│ ← Row 2  
└────────────────┘

Menu Navigation Flow:    
    ┌────────────────┐
    │T:22.5  RH:65   │ ← DEFAULT SCREEN (Menu 0)
    │T:HEAT  RH:OFF  │   Live sensor readings & control status    └────────┬───────┘   Or during AutoTune:
             │           ┌────────────────┐
             │           │T:22.5  RH:65   │ ← AutoTune Active
             │           │PIDtune Active  │
             │           └────────────────┘
             │           Or after AutoTune completion (3 seconds):
             │           ┌────────────────┐
             │           │T:22.5  RH:65   │ ← AutoTune Complete
             │           │Tuning Done     │
             │           └────────────────┘
             │ [BUTTON PRESS]
    ┌────────▼───────┐
    │Temp Setpoint:  │ ← TEMPERATURE SETPOINT (Menu 1)
    │22.5 C    ADJUST│   [ROTATE = ±0.1°C, Range: 10-40°C]
    └────────┬───────┘
             │ [BUTTON PRESS]    
    ┌────────▼───────┐
    │RH Setpoint:    │ ← HUMIDITY SETPOINT (Menu 2)
    │65%       ADJUST│   [ROTATE = ±1%, Range: 30-90%]
    └────────┬───────┘
             │ [BUTTON PRESS]
    ┌────────▼───────┐
    │Temp Control:   │ ← TEMPERATURE CONTROL (Menu 3)
    │ON        TOGGLE│   [ROTATE = Toggle ON/OFF]
    └────────┬───────┘
             │ [BUTTON PRESS]
    ┌────────▼───────┐
    │RH Control:     │ ← HUMIDITY CONTROL (Menu 4)
    │OFF       TOGGLE│   [ROTATE = Toggle ON/OFF]
    └────────┬───────┘
             │ [BUTTON PRESS]
             └────────────► (Return to DEFAULT SCREEN)

Error/Fallback Display:
    ┌────────────────┐
    │No Climate Ctrl │ ← Shown when ClimateController unavailable
    │Available       │
    └────────────────┘

Key Features:
- Automatic timeout: Returns to DEFAULT after 10 seconds of inactivity
- Real-time updates: Sensor values refresh continuously on DEFAULT screen
- Auto-save: Settings changes are immediately saved to configuration
- Character optimization: Abbreviations used to fit 16-character width
  * T: = Temperature, RH: = Relative Humidity
  * Temperature status: HEAT (heating), COOL (cooling), OK (standby), OFF (disabled)
  * Humidity status: HUM (humidifying), DEHUM (dehumidifying), OK (standby), OFF (disabled)
  * ADJUST/TOGGLE = Action indicators
```

## Device Registration

The Interface system uses two device types from the DeviceRegistry:

```cpp
// Display device
DeviceRegistry::registerDeviceType("Display", "LCD2x16", ...);

// Rotary encoder device  
DeviceRegistry::registerDeviceType("RotaryEncoder", "I2C", ...);

// Note: Interface is NOT a device - it's a coordination class that uses these devices
```

## Usage Example

### Basic Setup
```cpp
#include "DeviceRegistry/DeviceRegistry.h"
#include "Interface.h"
#include "ClimateController.h"

void setup() {
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    
    // Create devices (see InterfaceExample.cpp for full example)
    // ...device creation code...
      // Initialize all devices
    registry.initializeAllDevices();
    
    // Create interface coordination class
    Interface* interface = new Interface();
    ClimateController* controller = ClimateController::createFromDeviceRegistry();
    
    interface->setClimateController(controller);
    interface->begin();  // Initialize interface with devices from registry
    interface->setTimeoutMs(10000);      // 10 second timeout
    // Note: Temperature adjusts in 0.1°C steps, Humidity in 1% steps (hardcoded)
}

void loop() {
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    registry.updateAllDevices();  // Updates Display and RotaryEncoder devices
    interface->update();          // Updates Interface coordination logic
    delay(50);
}
```

### Configuration File Integration
The Interface can be configured via JSON configuration files:

```json
{
  "devices": [
    {
      "type": "Display",
      "model": "LCD2x16", 
      "address": "0x27",
      "label": "Main Display"
    },
    {
      "type": "RotaryEncoder",
      "model": "I2C",
      "address": "0x30", 
      "label": "Main Encoder"
    }
  ]
}
```

## Hardware Requirements

### Display
- **Type**: 2x16 character LCD
- **Interface**: I2C via PCF8574 backpack
- **Address**: Configurable (typically 0x27 or 0x3F)
- **Wiring**: SDA, SCL, VCC (5V or 3.3V), GND

### Rotary Encoder  
- **Type**: i2cEncoderLibV2 compatible encoder
- **Interface**: I2C
- **Address**: Configurable (default 0x61)
- **Features**: Position sensing, button press detection
- **Wiring**: SDA, SCL, VCC (3.3V), GND

### Microcontroller
- **Platform**: ESP32 (or compatible)
- **I2C**: Hardware I2C (Wire library)
- **Optional**: TCA9548A I2C multiplexer for multiple devices

## Implementation Details

### Device Architecture Integration
The Interface coordinates with the established device architecture:
- Uses dependency injection pattern for Display and RotaryEncoder device references
- Accesses devices through DeviceRegistry for automatic discovery and management
- Does NOT inherit from Device base class - it's a coordination layer
- Provides menu navigation by coordinating between multiple devices

### Error Handling
- Validates device connections before operation
- Gracefully handles missing ClimateController
- Provides fallback displays for sensor reading errors
- Includes bounds checking for setpoint adjustments

### Memory Management
- Devices managed by DeviceRegistry (no manual memory management)
- Uses references rather than ownership for device coordination
- Minimal memory footprint with efficient string handling

## Extensibility

### Adding New Menu Items
To add new menu items, extend the MenuState enum and add corresponding cases:

```cpp
enum MenuState {
    MENU_DEFAULT = 0,
    MENU_TEMP_SETPOINT,
    MENU_HUMIDITY_SETPOINT, 
    MENU_TEMP_CONTROL_ENABLE,
    MENU_HUMIDITY_CONTROL_ENABLE,
    MENU_NEW_SETTING,  // Add new menu item
    MENU_COUNT        // Keep this last
};
```

Then add corresponding display and adjustment methods:
- `displayNewSetting()`
- Add case in `adjustCurrentSetting()`
- Add case in `updateDisplay()`

### Custom Display Formats
Override display methods to customize the appearance:
- `formatTemperature()` - Temperature display format
- `formatHumidity()` - Humidity display format  
- `formatOnOff()` - Boolean status format

### Alternative Input Devices
The system can be extended to support additional input devices by:
- Creating new device types that implement button/rotation interfaces
- Modifying Interface to accept different input device types
- Maintaining the same logical navigation structure

## Troubleshooting

### Common Issues

1. **Interface not responding**
   - Check I2C connections and addresses
   - Verify devices are properly registered and initialized
   - Check serial output for device status messages

2. **Display shows "No Climate Ctrl Available"**
   - Ensure ClimateController is created and passed to Interface
   - Check that climate sensors are properly connected
   - Verify ClimateController initialization

3. **Encoder not detecting rotation/button presses**
   - Check RotaryEncoder device I2C address and wiring
   - Verify encoder initialization in serial output
   - Test encoder independently with RotaryEncoder device methods

4. **Menu timeout not working**
   - Check that Interface update() is being called regularly
   - Verify timeout value is set correctly
   - Ensure system millis() is functioning properly

### Debug Information
Enable debug output by checking DeviceRegistry and Interface serial messages:
- Device registration confirmations
- Device initialization status
- Menu state changes
- Encoder activity detection
- Error conditions and fallbacks

## Future Enhancements

### Possible Extensions
- **Multi-language support**: Configurable display languages
- **Custom themes**: Different display layouts and styles  
- **Advanced navigation**: Sub-menus and hierarchical structure
- **Remote control**: MQTT or web-based menu control
- **Data logging**: Historical setpoint and usage tracking
- **Alarm management**: Visual and audible alarm handling
- **Trend display**: Graphical temperature/humidity trends
- **Calibration menus**: Sensor calibration interfaces

### Integration Opportunities
- **Configuration management**: Runtime configuration changes
- **Network settings**: WiFi and MQTT configuration menus
- **System diagnostics**: Device health and status displays
- **Energy monitoring**: Power consumption and efficiency metrics
