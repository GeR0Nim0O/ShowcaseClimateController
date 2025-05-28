#include "DeviceRegistry.h"
#include "BH1705sensor.h"
#include "SCALESsensor.h"
#include "DS3231rtc.h"
#include "GP8403dac.h"

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
    // Removed dynamic_cast and type-specific vectors due to -fno-rtti
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

PCF8574gpio* DeviceRegistry::getGPIOExpander(int index) {
    if (index < 0 || index >= gpioExpanders.size()) {
        return nullptr;
    }
    return gpioExpanders[index];
}

SHT31sensor* DeviceRegistry::getTemperatureHumiditySensor(int index) {
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

Device* DeviceRegistry::createDeviceWithThresholds(
    const String& type, 
    const String& typeNumber, 
    TwoWire* wire, 
    uint8_t address, 
    uint8_t tcaPort, 
    const std::map<String, float>& channelThresholds, 
    const std::map<String, String>& channelNames, 
    int deviceIndex,
    const String& mode
) {
) {
    Device* device = nullptr;
    
    // Convert channelThresholds to a single threshold (use first threshold found)
    float threshold = 1.0f; // default
    if (!channelThresholds.empty()) {
        threshold = channelThresholds.begin()->second;
    }
    
    // Convert channelNames to std::map<String, String>
    std::map<String, String> channels = channelNames;
    
    // Create a meaningful device name based on type, typeNumber, and location
    String deviceName = typeNumber + "_TCA" + String(tcaPort) + "_" + String(deviceIndex);
    
    Serial.print("Creating device: ");
    Serial.print(type);
    Serial.print(" ");
    Serial.println(typeNumber);
    Serial.print("Device Name: ");
    Serial.println(deviceName);
    
    if (type.equalsIgnoreCase("Sensor")) {
        if (typeNumber.equalsIgnoreCase("SHT31")) {
            device = new SHT31sensor(wire, address, tcaPort, threshold, channels, deviceIndex);
            if (device) device->deviceName = deviceName; // Set device name
        } else if (typeNumber.equalsIgnoreCase("BH1705")) {
            device = new BH1705sensor(wire, address, tcaPort, threshold, channels, deviceIndex);
            if (device) device->deviceName = deviceName; // Set device name
        } else if (typeNumber.equalsIgnoreCase("SCALES")) {
            device = new SCALESsensor(wire, address, tcaPort, threshold, channels, deviceIndex);
            if (device) device->deviceName = deviceName; // Set device name
        }    } else if (type.equalsIgnoreCase("GPIO")) {        if (typeNumber.equalsIgnoreCase("PCF8574")) {
            // Parse mode from string
            PCF8574Mode pcfMode = PCF8574Mode::OUTPUT_MODE; // Default
            if (mode.equalsIgnoreCase("INPUT") || mode.equalsIgnoreCase("INPUT_MODE")) {
                pcfMode = PCF8574Mode::INPUT_MODE;
            } else if (mode.equalsIgnoreCase("OUTPUT") || mode.equalsIgnoreCase("OUTPUT_MODE")) {
                pcfMode = PCF8574Mode::OUTPUT_MODE;
            }
            
            Serial.print("Creating PCF8574 with mode: ");
            Serial.println(mode.length() > 0 ? mode : "OUTPUT_MODE (default)");
            
            device = new PCF8574gpio(wire, address, tcaPort, threshold, channels, deviceIndex, pcfMode);
            if (device) device->deviceName = deviceName; // Set device name
        }
    } else if (type.equalsIgnoreCase("RTC")) {
        if (typeNumber.equalsIgnoreCase("DS3231")) {
            device = new DS3231rtc(wire, address, tcaPort, threshold, channels, deviceIndex);
            if (device) device->deviceName = deviceName; // Set device name
        }
    } else if (type.equalsIgnoreCase("DAC")) {
        if (typeNumber.equalsIgnoreCase("MCP4725")) {
            device = new GP8403dac(wire, address, tcaPort, threshold, channels, deviceIndex);
            if (device) device->deviceName = deviceName; // Set device name
        }
    }
    if (device) {
        Serial.println("Device created successfully");
        // Register the device in the registry
        DeviceRegistry& registry = DeviceRegistry::getInstance();
        if (registry.registerDevice(device)) {
            Serial.println("Device registered successfully in registry");
        } else {
            Serial.println("Failed to register device in registry");
        }
    } else {
        Serial.println("Failed to create device - unknown type/typeNumber combination");
    }
    
    return device;
}
