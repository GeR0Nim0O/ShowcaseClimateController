#ifndef DEVICE_REGISTRY_H
#define DEVICE_REGISTRY_H

#include <Arduino.h>
#include <vector>
#include "Device.h"
#include "PCF8574gpio.h"
#include "SHTsensor.h"

class DeviceRegistry {
public:
    static DeviceRegistry& getInstance();
      bool registerDevice(Device* device);
    bool initializeAllDevices();
    void updateAllDevices();    // Device creation
    static Device* createDeviceWithThresholds(
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
      // Device getters
    PCF8574gpio* getGPIOExpander(int index = 0);
    SHTsensor* getTemperatureHumiditySensor(int index = 0);
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
    std::vector<PCF8574gpio*> gpioExpanders;
    std::vector<SHTsensor*> temperatureHumiditySensors;
};

#endif // DEVICE_REGISTRY_H
