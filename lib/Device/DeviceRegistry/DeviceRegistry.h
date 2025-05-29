#ifndef DEVICE_REGISTRY_H
#define DEVICE_REGISTRY_H

#include <Arduino.h>
#include <vector>
#include <map>
#include "Device.h"

class DeviceRegistry {
public:
    static DeviceRegistry& getInstance();
    
    // Core device management - generic and hardware-agnostic
    bool registerDevice(Device* device);
    bool initializeAllDevices();
    void updateAllDevices();
    
    // Generic device getters - no hardware-specific types
    Device* getDevice(const String& deviceName);
    Device* getDeviceByIndex(int index);
    std::vector<Device*> getDevicesByType(const String& type);
    Device* getDeviceByType(const String& type, int index = 0);
    
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
    
    // Internal helper for type-based operations
    bool isTypeMatch(const String& deviceType, const String& searchType) const;
};

#endif // DEVICE_REGISTRY_H
