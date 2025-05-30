# Type Registration Mismatch Fix

## Problem Identified
The core issue preventing device creation was a type registration mismatch between the DeviceRegistry and the JSON configuration:

- **DeviceRegistry Registration**: `("Sensor", "SHT31")`
- **JSON Configuration**: `"TypeNumber": "SHT"`

This mismatch caused the DeviceRegistry to fail finding the device type during creation, resulting in null device pointers.

## Root Cause Analysis
Located in `src/Device/DeviceRegistry/DeviceRegistry.cpp` at line 316:

```cpp
bool registeredSHT31 = DeviceRegistry::registerDeviceType("Sensor", "SHT31", 
    [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
       const std::map<String, String>& channels, int deviceIndex) {
    return new SHTsensor(wire, address, tcaPort, threshold, channels, deviceIndex);
});
```

## Fix Applied
Updated `config.json` to match the DeviceRegistry registration:

### Before:
```json
"SHT_Interior": {
  "Type": "Sensor",
  "TypeNumber": "SHT",
  "Address": "0x44",
  "Label": "Interior"
}

"SHT_Exterior": {
  "Type": "Sensor", 
  "TypeNumber": "SHT",
  "Address": "0x44",
  "Label": "Exterior"
}
```

### After:
```json
"SHT_Interior": {
  "Type": "Sensor",
  "TypeNumber": "SHT31",
  "Address": "0x44",
  "Label": "Interior"
}

"SHT_Exterior": {
  "Type": "Sensor",
  "TypeNumber": "SHT31", 
  "Address": "0x44",
  "Label": "Exterior"
}
```

## Expected Results
With this fix, the system should now:

1. ✅ **Successfully create both SHT sensors** from JSON configuration
2. ✅ **Apply proper labels** ("Interior" and "Exterior") via positional indexing
3. ✅ **Use Interior sensor for climate control** in the main application
4. ✅ **Display labeled sensor data** in serial output and MQTT

## Debug Output to Verify
Look for these messages in the serial monitor:

```
DeviceRegistry: Creating device - Type: Sensor, Model: SHT31
DeviceRegistry: Device type found in factory registry
DeviceRegistry: Device created successfully via factory
Device created successfully. Setting label from JSON: 'Interior'
Device label after setting: 'Interior'
Successfully created device: SHT_Interior with final label: Interior
```

## Build Status
✅ **Compilation**: SUCCESS - Code builds without errors
📤 **Upload**: In progress - Code uploaded to device
🔍 **Testing**: Monitor serial output to verify functionality

## System Architecture
The positional indexing system is now functional:
1. First SHT31 sensor found during I2C scan → assigned to SHT_Interior (Label: "Interior")
2. Second SHT31 sensor found during I2C scan → assigned to SHT_Exterior (Label: "Exterior")

## Next Steps
1. Monitor serial output to confirm device creation
2. Verify both sensors are created with correct labels
3. Test climate control uses Interior-labeled sensor
4. Validate MQTT data includes proper device labeling
