# Positional Indexing Device Labeling System

## Overview

The device labeling system has been enhanced to support configurable labels through the `config.json` file using positional indexing. This allows for flexible device labeling without hardcoding TCA port mappings.

## How Positional Indexing Works

### Concept
Instead of mapping devices to specific TCA ports, the system now uses **positional indexing**:
- The first sensor of a specific type found during I2C scanning gets assigned to the first configuration entry of that type in the JSON
- The second sensor of the same type gets assigned to the second configuration entry, and so on

### Implementation Details

1. **Scanning Phase**: All I2C devices are scanned and stored with their addresses and TCA ports
2. **Sorting**: Scanned devices are sorted by TCA port first, then by address for consistent ordering
3. **Type Counting**: The system maintains counters for each device type (e.g., "SHT", "DS3231", etc.)
4. **Assignment**: When processing JSON configurations, devices are assigned based on their position in the scan results

### Example

**Config.json Configuration:**
```json
{
  "SHT_Interior": {
    "Type": "Sensor",
    "TypeNumber": "SHT",
    "Address": "0x44",
    "Label": "Interior"
  },
  "SHT_Exterior": {
    "Type": "Sensor", 
    "TypeNumber": "SHT",
    "Address": "0x44",
    "Label": "Exterior"
  }
}
```

**Scan Results:**
- TCA Port 1: SHT sensor at 0x44
- TCA Port 2: SHT sensor at 0x44

**Assignment:**
- First SHT sensor found (TCA Port 1) → `SHT_Interior` → Label: "Interior"
- Second SHT sensor found (TCA Port 2) → `SHT_Exterior` → Label: "Exterior"

## Benefits

1. **Flexible Configuration**: Labels can be changed in JSON without code modifications
2. **Port Independence**: No need to specify TCA ports in configuration
3. **Consistent Ordering**: Deterministic assignment based on scan order
4. **Backward Compatibility**: Existing climate control logic using "Interior" labeled sensors continues to work

## Usage

### Device Labels
The following device labels are supported in the current configuration:

- **"Interior"**: For sensors inside the controlled environment
- **"Exterior"**: For sensors outside the controlled environment  
- **"Controller"**: For control devices (DACs, GPIOs)
- **"System"**: For system devices (RTC)

### Climate Control Integration
The climate control system specifically looks for devices labeled as "Interior" to make control decisions:

```cpp
// Climate control uses Interior-labeled sensors
Device* interiorSensor = DeviceRegistry::getInstance().findDeviceByLabel("Interior");
```

### Adding New Device Types
To add support for new device types:

1. Add the device configuration to `config.json` with appropriate `Type`, `TypeNumber`, `Address`, and `Label`
2. Ensure the device type is registered in `DeviceRegistry`
3. The positional indexing will automatically handle assignment during initialization

## File Changes

### Modified Files:
- `config.json`: Added separate entries for multiple SHT sensors with labels
- `src/Configuration/Configuration.cpp`: Implemented positional indexing in `initializeDevicesFromJSON()`

### Key Changes:
1. **Removed TCA Port Dependencies**: No longer requires `TCAPort` fields in JSON
2. **Added Type-Based Counting**: Tracks how many devices of each type have been processed
3. **Consistent Ordering**: Sorts scanned devices for predictable assignment
4. **Enhanced Logging**: Provides detailed debug information about device assignment

## Testing

The system provides extensive logging during device initialization:
- Shows scan results for each TCA port
- Indicates which device configuration is being processed
- Reports successful device creation with labels
- Warns about missing or insufficient devices

Monitor the serial output during initialization to verify correct device assignment.
