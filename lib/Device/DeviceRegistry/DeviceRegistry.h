#ifndef DEVICE_REGISTRY_H
#define DEVICE_REGISTRY_H

#include <Arduino.h>
#include <vector>
#include <map>
#include <functional>
#include "Device.h"

class DeviceRegistry {
public:
    static DeviceRegistry& getInstance();
    
    // Hardware-agnostic device factory registration
    static bool registerDeviceType(const String& type, const String& typeNumber, 
        std::function<Device*(TwoWire*, uint8_t, uint8_t, float, const std::map<String, String>&, int)> creator);
    
    // Hardware-agnostic device creation
    static Device* createDevice(const String& type, const String& typeNumber, 
        TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
        const std::map<String, String>& channels, int deviceIndex);
    
    // Device creation with extended parameters (for backward compatibility)
    Device* createDeviceWithThresholds(
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
    
    // Core device management - generic and hardware-agnostic
    bool registerDevice(Device* device);
    bool initializeAllDevices();
    void updateAllDevices();
      // Generic device getters - no hardware-specific types
    Device* getDevice(const String& deviceName);
    Device* getDeviceByIndex(int index);
    std::vector<Device*> getDevicesByType(const String& type);
    Device* getDeviceByType(const String& type, int index = 0);
    Device* getDeviceByTypeAndLabel(const String& type, const String& label);
    std::vector<Device*> getDevicesByLabel(const String& label);
    
    // Status and utility methods
    int getDeviceCount() const { return devices.size(); }
    int getDeviceCountByType(const String& type) const;
    bool allDevicesConnected();
    void printDeviceStatus();
    
    // Iterator support for range-based loops
    std::vector<Device*>::iterator begin() { return devices.begin(); }
    std::vector<Device*>::iterator end() { return devices.end(); }
    std::vector<Device*>::const_iterator begin() const { return devices.begin(); }
    std::vector<Device*>::const_iterator end() const { return devices.end(); }

private:
    DeviceRegistry() = default;
    ~DeviceRegistry() = default;
    DeviceRegistry(const DeviceRegistry&) = delete;
    DeviceRegistry& operator=(const DeviceRegistry&) = delete;
    
    std::vector<Device*> devices;
    
    // Static factory registry for hardware-agnostic device creation
    static std::map<std::pair<String, String>, 
        std::function<Device*(TwoWire*, uint8_t, uint8_t, float, const std::map<String, String>&, int)>> deviceFactory;
    
    // Internal helper for type-based operations
    bool isTypeMatch(const String& deviceType, const String& searchType) const;
};

#endif // DEVICE_REGISTRY_H
