# Device Registry Fix - TypeNumber Alignment

## Problem Resolved
The DeviceRegistry was registering SHT sensors as "SHT31" but the config.json files were using "SHT", causing device creation failures.

## Changes Made

### 1. DeviceRegistry.cpp Update
**File**: `src/Device/DeviceRegistry/DeviceRegistry.cpp`
**Change**: Line 315 - Updated device registration from "SHT31" to "SHT"

```cpp
// BEFORE:
bool registeredSHT31 = DeviceRegistry::registerDeviceType("Sensor", "SHT31", ...

// AFTER:
bool registeredSHT = DeviceRegistry::registerDeviceType("Sensor", "SHT", ...
```

### 2. Configuration Files Updated
**Files Updated**:
- `config.json` ✅ (already had "SHT")
- `config_for_sd_card.json` ✅ (fixed from "SHT31" to "SHT")

Both configuration files now consistently use:
```json
"TypeNumber": "SHT"
```

## Expected Behavior After Fix

1. **Device Registration**: DeviceRegistry now registers SHT sensors as ("Sensor", "SHT")
2. **Configuration Match**: Both config files use "TypeNumber": "SHT" 
3. **Device Creation**: Should successfully create both SHT_Interior and SHT_Exterior devices
4. **Label Assignment**: Interior and Exterior labels should be properly applied via positional indexing

## Next Steps

### 1. Deploy Updated Config to SD Card
Copy the corrected `config_for_sd_card.json` to your SD card as `config.json`

### 2. Upload and Test
```bash
python -m platformio run --target upload --upload-port COM11
python -m platformio device monitor --port COM11 --baud 115200
```

### 3. Expected Serial Output
You should now see:
```
DeviceRegistry: Registering device type: Sensor/SHT
Configuration: Processing device: SHT_Interior
Configuration: Found TypeNumber: SHT
Configuration: Extracted label: Interior
Configuration: Device created successfully with label: Interior

Configuration: Processing device: SHT_Exterior  
Configuration: Found TypeNumber: SHT
Configuration: Extracted label: Exterior
Configuration: Device created successfully with label: Exterior
```

### 4. Verification Points
- [ ] Two SHT devices created (Interior and Exterior)
- [ ] Labels properly extracted and assigned
- [ ] Climate control uses Interior-labeled sensor
- [ ] MQTT data includes proper device identification

## Root Cause Analysis
The issue was a simple naming inconsistency between:
- DeviceRegistry registration: "SHT31" (hardware-specific)
- Configuration files: "SHT" (user preference for generic naming)

By aligning both to use "SHT", the positional indexing system can now properly match configurations to created devices.

## Code Quality Impact
- ✅ No breaking changes to existing device functionality
- ✅ Maintains backward compatibility for other device types
- ✅ Improves consistency between registry and configuration
- ✅ Enables the enhanced labeling system to work as designed
