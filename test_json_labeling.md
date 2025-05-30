# JSON-Based Device Labeling Test

## Overview
This test demonstrates the new JSON-based device labeling system that replaces the previous hardcoded TCA port-based labeling.

## Test Configuration (config.json)

The following devices are now configured with labels in the JSON, including **multiple SHT sensors**:

```json
{
  "Devices": {
    "DS3231": {
      "Type": "RTC",
      "TypeNumber": "DS3231",
      "Address": "0x68",
      "Label": "System",
      "Channels": { "Time": { "Name": "Time", "Threshold": 0.0 } }
    },
    "SHT_Interior": {
      "Type": "Sensor",
      "TypeNumber": "SHT",
      "Address": "0x44",
      "TCAPort": 1,
      "Label": "Interior",
      "Channels": {
        "T": { "Name": "Temperature", "Threshold": 0.3 },
        "H": { "Name": "Humidity", "Threshold": 1.0 }
      }
    },
    "SHT_Exterior": {
      "Type": "Sensor",
      "TypeNumber": "SHT",
      "Address": "0x44",
      "TCAPort": 2,
      "Label": "Exterior",
      "Channels": {
        "T": { "Name": "Temperature", "Threshold": 0.3 },
        "H": { "Name": "Humidity", "Threshold": 1.0 }
      }
    },    "BH1705": {
      "Type": "Sensor",
      "TypeNumber": "BH1705",
      "Address": "0x23",
      "TCAPort": 1,
      "Label": "Interior",
      "Channels": { "L": { "Name": "Lux", "Threshold": 1.0 } }
    },
    "SCALE": {
      "Type": "Sensor",
      "TypeNumber": "SCALES",
      "Address": "0x26",
      "TCAPort": 1,
      "Label": "Interior",
      "Channels": { "W": { "Name": "Weight", "Threshold": 1.0 } }
    },
    "GMX02B": {
      "Type": "Sensor",
      "TypeNumber": "GMx02B",
      "Address": "0x08",
      "TCAPort": 1,
      "Label": "Interior",
      "Channels": { "NH3": { "Name": "Ammonia", "Threshold": 1.0 } }
    },
    "PCF8574": {
      "Type": "GPIO",
      "TypeNumber": "PCF8574",
      "Address": "0x20",
      "TCAPort": 1,
      "Mode": "OUTPUT",
      "Label": "Controller",
      "Channels": {
        "IO0": { "Name": "FanExterior", "Threshold": 1.0 },
        "IO1": { "Name": "FanInterior", "Threshold": 1.0 }
      }
    },
    "GP8403": {
      "Type": "DAC",
      "TypeNumber": "GP8403",
      "Address": "0x5F",
      "TCAPort": 1,
      "Label": "Controller",
      "Channels": { "DAC_A": { "Name": "TemperaturePower" } }
    }
  }
}
```

## Key Changes

### 1. JSON Configuration Structure
- Added `"Label"` field to each device configuration
- Labels can be any custom string (e.g., "Interior", "Exterior", "Controller", "System")
- Labels are completely configurable and not tied to hardware ports

### 2. Code Implementation
- **`Configuration::initializeDevicesFromJSON()`**: New method that creates devices from JSON config
- **Label Assignment**: Devices are labeled directly from JSON configuration
- **Fallback Support**: Original I2C scan method available as fallback (without hardcoded labels)

### 3. Climate Controller Compatibility
The existing climate controller code using `DeviceRegistry::getDeviceByTypeAndLabel("Sensor", "Interior")` will work seamlessly with the new system.

## Expected Output

When the system starts, you should see:

```
Using JSON configuration for device initialization
Creating device from JSON: SHT (Sensor/SHT) at address 0x44 on TCA port 1 with label: Interior
Device labeled as: Interior
Successfully created device: SHT

Creating device from JSON: BH1705 (Sensor/BH1705) at address 0x23 on TCA port 1 with label: Interior  
Device labeled as: Interior
Successfully created device: BH1705

Creating device from JSON: PCF8574 (GPIO/PCF8574) at address 0x20 on TCA port 0 with label: Controller
Device labeled as: Controller
Successfully created device: PCF8574

Successfully created 7 devices from JSON configuration
```

## Test Cases

1. **Device Creation**: All devices should be created from JSON configuration
2. **Label Assignment**: Each device should have the correct label from JSON
3. **Climate Controller**: Should find Interior sensor using `getDeviceByTypeAndLabel()`
4. **Display Output**: Device labels should appear in sensor readings display
5. **Fallback Mode**: If JSON is invalid, system falls back to I2C scan (without labels)

## Benefits

1. **Flexibility**: Labels can be changed without code modification
2. **Configuration**: All device settings centralized in JSON
3. **Scalability**: Easy to add new devices or change existing configurations
4. **Maintainability**: No hardcoded TCA port mappings in code
5. **Compatibility**: Existing climate control logic unchanged
