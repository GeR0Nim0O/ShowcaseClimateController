#ifndef DEVICE_FACTORY_H
#define DEVICE_FACTORY_H

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <map>
#include "Device.h"

/**
 * DeviceFactory - A clean factory class for creating specific device instances
 * 
 * This factory pattern separates device creation logic from the DeviceRegistry,
 * making the system more modular and easier to extend with new device types.
 * 
 * To add a new device type:
 * 1. Add the device-specific include in the .cpp file
 * 2. Add creation logic in createDevice() method
 * 3. Optionally add helper methods for complex device types
 */
class DeviceFactory {
public:
    /**
     * Create a device instance based on type and configuration
     * 
     * @param wire I2C bus to use
     * @param type Device category (e.g., "Sensor", "GPIO", "DAC", "RTC")
     * @param typeNumber Specific device model (e.g., "SHT31", "PCF8574", "GP8403")
     * @param address I2C address
     * @param tcaPort TCA9548A multiplexer port
     * @param channelThresholds Map of channel names to threshold values
     * @param channelNames Map of channel IDs to descriptive names
     * @param deviceIndex Device instance index
     * @param mode Device-specific mode (e.g., "INPUT"/"OUTPUT" for GPIO)
     * @return Pointer to created device or nullptr if creation failed
     */
    static Device* createDevice(
        TwoWire* wire,
        const String& type, 
        const String& typeNumber, 
        uint8_t address, 
        uint8_t tcaPort, 
        const std::map<String, float>& channelThresholds, 
        const std::map<String, String>& channelNames, 
        int deviceIndex,
        const String& mode = ""
    );
    
    /**
     * Get list of supported device types
     * @return Vector of supported type/typeNumber combinations
     */
    static std::vector<std::pair<String, String>> getSupportedDevices();
    
    /**
     * Check if a device type is supported
     * @param type Device category
     * @param typeNumber Specific device model
     * @return true if the device type is supported
     */
    static bool isDeviceSupported(const String& type, const String& typeNumber);

private:
    // Helper methods for creating specific device categories
    static Device* createSensorDevice(
        TwoWire* wire, const String& typeNumber, uint8_t address, uint8_t tcaPort,
        float threshold, const std::map<String, String>& channels, int deviceIndex,
        const String& deviceName, const std::map<String, float>& channelThresholds
    );
    
    static Device* createGPIODevice(
        TwoWire* wire, const String& typeNumber, uint8_t address, uint8_t tcaPort,
        float threshold, const std::map<String, String>& channels, int deviceIndex,
        const String& deviceName, const std::map<String, float>& channelThresholds,
        const String& mode
    );
    
    static Device* createDACDevice(
        TwoWire* wire, const String& typeNumber, uint8_t address, uint8_t tcaPort,
        float threshold, const std::map<String, String>& channels, int deviceIndex,
        const String& deviceName, const std::map<String, float>& channelThresholds
    );
    
    static Device* createRTCDevice(
        TwoWire* wire, const String& typeNumber, uint8_t address, uint8_t tcaPort,
        float threshold, const std::map<String, String>& channels, int deviceIndex,
        const String& deviceName, const std::map<String, float>& channelThresholds
    );
    
    // Utility methods
    static String generateDeviceName(const String& typeNumber, uint8_t tcaPort, int deviceIndex);
    static bool validateDeviceParameters(const String& type, const String& typeNumber, uint8_t address);
    static void logDeviceCreation(const String& type, const String& typeNumber, 
                                 uint8_t address, uint8_t tcaPort, int deviceIndex);
    static bool validateCreatedDevice(Device* device);
};

#endif // DEVICE_FACTORY_H
