#include "DeviceRegistry.h"

DeviceRegistry& DeviceRegistry::getInstance() {
    static DeviceRegistry instance;
    return instance;
}

bool DeviceRegistry::registerDevice(Device* device) {
    if (!device) {
        Serial.println("Cannot register null device");
        return false;
    }
    
    devices.push_back(device);
      // Add to specific type vectors for easy access
    PCF8574gpio* gpioDevice = dynamic_cast<PCF8574gpio*>(device);
    if (gpioDevice) {
        gpioExpanders.push_back(gpioDevice);
    }
    
    SHT31sensor* tempHumDevice = dynamic_cast<SHT31sensor*>(device);
    if (tempHumDevice) {
        temperatureHumiditySensors.push_back(tempHumDevice);
    }
    
    Serial.print("Registered device: ");
    Serial.println(device->getDeviceName());
    
    return true;
}

bool DeviceRegistry::initializeAllDevices() {
    Serial.println("Initializing all devices...");
    bool allSuccess = true;
    
    for (Device* device : devices) {
        Serial.print("Initializing ");
        Serial.print(device->getDeviceName());
        Serial.print("... ");
        
        if (device->begin()) {
            Serial.println("OK");
        } else {
            Serial.println("FAILED");
            allSuccess = false;
        }
    }
    
    if (allSuccess) {
        Serial.println("All devices initialized successfully");
    } else {
        Serial.println("Some devices failed to initialize");
    }
    
    return allSuccess;
}

void DeviceRegistry::updateAllDevices() {
    for (Device* device : devices) {
        if (device->isInitialized()) {
            device->update();
        }
    }
}

PCF8574_GPIO* DeviceRegistry::getGPIOExpander(int index) {
    if (index < 0 || index >= gpioExpanders.size()) {
        return nullptr;
    }
    return gpioExpanders[index];
}

SHT31_Sensor* DeviceRegistry::getTemperatureHumiditySensor(int index) {
    if (index < 0 || index >= temperatureHumiditySensors.size()) {
        return nullptr;
    }
    return temperatureHumiditySensors[index];
}

Device* DeviceRegistry::getDevice(const String& deviceName) {
    for (Device* device : devices) {
        if (device->getDeviceName() == deviceName) {
            return device;
        }
    }
    return nullptr;
}

Device* DeviceRegistry::getDeviceByIndex(int index) {
    if (index < 0 || index >= devices.size()) {
        return nullptr;
    }
    return devices[index];
}

bool DeviceRegistry::allDevicesConnected() {
    for (Device* device : devices) {
        if (!device->isConnected()) {
            return false;
        }
    }
    return true;
}

void DeviceRegistry::printDeviceStatus() {
    Serial.println("\n=== Device Status ===");
    Serial.print("Total devices: ");
    Serial.println(devices.size());
    
    for (int i = 0; i < devices.size(); i++) {
        Device* device = devices[i];
        Serial.print(i);
        Serial.print(": ");
        Serial.print(device->getDeviceName());
        Serial.print(" (");
        Serial.print(device->getType());
        Serial.print(") - ");
        Serial.print("Address: 0x");
        Serial.print(device->getI2CAddress(), HEX);
        Serial.print(", Channel: ");
        Serial.print(device->getTCAChannel());
        Serial.print(", Status: ");
        Serial.println(device->isConnected() ? "Connected" : "Disconnected");
    }
    Serial.println("==================\n");
}
