# Sensor Labeling Implementation Test Plan

## What Was Implemented

### 1. Device Labeling System
- Added `deviceLabel` field to Device base class
- Added `getDeviceLabel()` and `setDeviceLabel()` methods
- All devices can now have labels assigned

### 2. DeviceRegistry Label-Based Lookup
- Added `getDeviceByTypeAndLabel(String deviceType, String label)` method
- Added `getDevicesByLabel(String label)` method  
- Case-insensitive label matching

### 3. Automatic Sensor Labeling (Configuration.cpp)
```cpp
// TCA port 1 → "Interior" label
// TCA port 2 → "Exterior" label
// Other ports → "Unknown" label
```

### 4. Climate Controller Sensor Selection (main.cpp)
```cpp
// Primary: Use Interior labeled sensor for climate control
Device* interiorSensor = deviceRegistry.getDeviceByTypeAndLabel("TemperatureHumidity", "Interior");

// Fallback: Use first available sensor if no Interior sensor found
if (interiorSensor == nullptr) {
    // Use first available sensor
}
```

### 5. Enhanced Display and Logging
- All sensor data now shows device labels
- MQTT data includes labels
- SD card logging includes labels
- Debug output shows which sensor is used for climate control

## Expected Behavior

### On System Startup:
1. **Sensor Creation**: SHT sensors should be created with labels based on TCA port
   - TCA port 1 sensor → labeled "Interior"
   - TCA port 2 sensor → labeled "Exterior"

2. **Climate Controller Initialization**: 
   - Should find and use the "Interior" labeled sensor for control
   - Should log which sensor is being used
   - Should fallback to first available sensor if no Interior sensor exists

3. **Data Display**:
   - Sensor readings should show with labels (e.g., "SHTSensor_2 (Interior)")
   - MQTT messages should include labels
   - SD card logs should include labels

### During Operation:
1. **Climate Control**: Only the Interior sensor affects heating/cooling decisions
2. **Data Monitoring**: All sensors (Interior and Exterior) continue to be read and logged
3. **Fallback Safety**: System continues working even if Interior sensor fails

## Testing Checklist

- [ ] Build completes without errors ✓
- [ ] Sensors are labeled correctly based on TCA port assignment
- [ ] Climate controller uses Interior sensor for control decisions
- [ ] All sensors continue to be monitored and logged
- [ ] Display shows sensor labels in output
- [ ] MQTT data includes sensor labels
- [ ] SD card logging includes sensor labels
- [ ] Fallback mechanism works if no Interior sensor is found

## Hardware Configuration Verification

Verify that the TCA port assignments match your actual hardware:
- **TCA Port 1**: Should be connected to Interior sensor
- **TCA Port 2**: Should be connected to Exterior sensor

If the port assignments are different, update the labeling logic in `Configuration.cpp`:

```cpp
// In createSHTSensor function, modify the labeling logic:
if (tcaPort == YOUR_INTERIOR_PORT) {
    sensor->setDeviceLabel("Interior");
} else if (tcaPort == YOUR_EXTERIOR_PORT) {
    sensor->setDeviceLabel("Exterior");
} else {
    sensor->setDeviceLabel("Unknown");
}
```

## Next Steps

1. **Deploy and Test**: Upload the firmware and monitor serial output
2. **Verify Labeling**: Check that sensors are labeled correctly
3. **Confirm Climate Control**: Verify only Interior sensor affects climate decisions
4. **Monitor All Data**: Ensure all sensors continue to be logged
5. **Validate Fallback**: Test behavior if Interior sensor is disconnected

The implementation maintains full backward compatibility while adding the new labeling system for improved sensor management and control.
