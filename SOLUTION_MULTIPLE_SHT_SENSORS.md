# Solution: Multiple SHT Sensors with JSON Labeling

## Problem Addressed

The original issue was that the system had only one SHT device configuration in `config.json`, but there are actually multiple SHT sensors connected to different TCA ports (Interior and Exterior). The hardcoded TCA port-based labeling couldn't distinguish between multiple sensors of the same type.

## Solution Implemented

### 1. Updated JSON Configuration Structure

The `config.json` now supports multiple instances of the same device type by using unique device keys and explicit TCA port specification:

```json
{
  "Devices": {
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
    }
  }
}
```

### 2. Key Features

- **Unique Device Keys**: `SHT_Interior` and `SHT_Exterior` instead of just `SHT`
- **Explicit TCA Port**: `"TCAPort": 1` and `"TCAPort": 2` specify exact ports
- **Custom Labels**: `"Label": "Interior"` and `"Label": "Exterior"` for identification
- **Same I2C Address**: Both sensors can have address `0x44` since they're on different TCA ports

### 3. Code Implementation

#### New Method: `initializeDevicesFromJSON()`

This method replaces the hardcoded TCA port-based labeling:

```cpp
std::vector<Device*> Configuration::initializeDevicesFromJSON(
    std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, 
    DS3231rtc*& rtc
) {
    // Creates devices from JSON configuration
    // Uses TCAPort field or falls back to I2C scan mapping
    // Applies labels directly from JSON
}
```

#### Enhanced TCA Port Handling

The system now:
1. **Prefers JSON TCAPort**: Uses explicit port from JSON configuration
2. **Validates against scan results**: Ensures device actually exists on specified port
3. **Falls back gracefully**: Uses I2C scan if no TCAPort specified
4. **Provides detailed logging**: Shows exactly what's being created and why

### 4. Climate Controller Compatibility

The existing climate controller code continues to work unchanged:

```cpp
// This still works perfectly
Device* interiorSensor = DeviceRegistry::getInstance()
    .getDeviceByTypeAndLabel("Sensor", "Interior");

Device* exteriorSensor = DeviceRegistry::getInstance()
    .getDeviceByTypeAndLabel("Sensor", "Exterior");
```

### 5. Complete Device Mapping

The updated configuration supports all device types with labels:

| Device | TCA Port | Label | Purpose |
|--------|----------|-------|---------|
| SHT_Interior | 1 | Interior | Indoor climate sensing |
| SHT_Exterior | 2 | Exterior | Outdoor climate sensing |
| BH1705 | 1 | Interior | Light sensor |
| SCALE | 1 | Interior | Weight sensor |
| GMX02B | 1 | Interior | Air quality sensor |
| PCF8574 | 1 | Controller | GPIO control |
| GP8403 | 1 | Controller | DAC output |
| DS3231 | 0 | System | Real-time clock |

## Benefits

1. **Solves Multiple Sensor Problem**: Can now differentiate between Interior/Exterior SHT sensors
2. **Configuration Flexibility**: Labels can be changed without code modification
3. **Explicit Port Assignment**: No ambiguity about which port devices should use
4. **Backward Compatibility**: Fallback to I2C scan if JSON is incomplete
5. **Maintainable**: All device configuration centralized in JSON

## Testing

To test the new system:

1. **Deploy the updated config.json** with multiple SHT entries
2. **Monitor serial output** for device creation messages
3. **Verify labeling** by checking sensor readings display
4. **Test climate controller** to ensure it finds Interior sensor correctly

Expected output:
```
Using JSON configuration for device initialization
Creating device from JSON: SHT_Interior (Sensor/SHT) at address 0x44 on TCA port 1 with label: Interior
Device labeled as: Interior
Creating device from JSON: SHT_Exterior (Sensor/SHT) at address 0x44 on TCA port 2 with label: Exterior  
Device labeled as: Exterior
Successfully created 8 devices from JSON configuration
```

## Migration Notes

- **No breaking changes**: Existing code continues to work
- **Gradual migration**: Can add TCAPort fields incrementally
- **Fallback support**: Missing TCAPort falls back to I2C scan behavior
- **Error handling**: Clear messages when devices can't be found/created

This solution completely addresses the multiple SHT sensor labeling issue while maintaining system flexibility and reliability.
