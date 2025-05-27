#ifndef DEVICE_REGISTRY_H
#define DEVICE_REGISTRY_H

#include <Arduino.h>
#include <vector>
#include "Device.h"
#include "PCF8574gpio.h"
#include "SHT31sensor.h"

class DeviceRegistry {
public:
    static DeviceRegistry& getInstance();
    
    bool registerDevice(Device* device);
    bool initializeAllDevices();
    void updateAllDevices();
    
    // Device getters
    PCF8574_GPIO* getGPIOExpander(int index = 0);
    SHT31_Sensor* getTemperatureHumiditySensor(int index = 0);
    Device* getDevice(const String& deviceName);
    Device* getDeviceByIndex(int index);
    
    // Status
    int getDeviceCount() const { return devices.size(); }
    bool allDevicesConnected();
    void printDeviceStatus();

private:
    DeviceRegistry() = default;
    ~DeviceRegistry() = default;
    DeviceRegistry(const DeviceRegistry&) = delete;
    DeviceRegistry& operator=(const DeviceRegistry&) = delete;
    
    std::vector<Device*> devices;
    std::vector<PCF8574_GPIO*> gpioExpanders;
    std::vector<SHT31_Sensor*> temperatureHumiditySensors;
};

#endif // DEVICE_REGISTRY_H
