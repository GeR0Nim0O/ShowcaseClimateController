#include "DeviceRegistry/DeviceRegistry.h"
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

SHTsensor* DeviceRegistry::getTemperatureHumiditySensor(int index) {
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
    TwoWire* wire,
    const String& type, 
    const String& typeNumber, 
    uint8_t address, 
    uint8_t tcaPort, 
    const std::map<String, float>& channelThresholds, 
    const std::map<String, String>& channelNames, 
    int deviceIndex,
    const String& mode
) {
    Device* device = nullptr;
    
    // Add debugging for all device creation attempts
    Serial.print("Creating device with type: ");
    Serial.print(type);
    Serial.print(", type number: ");
    Serial.print(typeNumber);
    Serial.print(", address: 0x");
    Serial.print(address, HEX);
    Serial.print(", TCA port: ");
    Serial.println(tcaPort);
    
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
      if (type.equalsIgnoreCase("Sensor")) {        if (typeNumber.equalsIgnoreCase("SHT31")) {
            device = new SHTsensor(wire, address, tcaPort, threshold, channels, deviceIndex);
            if (device) {
                device->deviceName = deviceName; // Set device name
                device->setChannelThresholds(channelThresholds); // Set channel-specific thresholds
            }} else if (typeNumber.equalsIgnoreCase("BH1705")) {
            device = new BH1705sensor(address, tcaPort, threshold, channels, deviceIndex);
            if (device) {
                device->deviceName = deviceName; // Set device name
                device->setChannelThresholds(channelThresholds); // Set channel-specific thresholds
            }        } else if (typeNumber.equalsIgnoreCase("SCALES")) {
            device = new SCALESsensor(address, tcaPort, threshold, channels, deviceIndex);
            if (device) {
                device->deviceName = deviceName; // Set device name
                device->setChannelThresholds(channelThresholds); // Set channel-specific thresholds
            }
        }
    } else if (type.equalsIgnoreCase("GPIO")) {
        if (typeNumber.equalsIgnoreCase("PCF8574")) {
            // Parse mode from string
            PCF8574Mode pcfMode = PCF8574Mode::OUTPUT_MODE; // Default
            if (mode.equalsIgnoreCase("INPUT") || mode.equalsIgnoreCase("INPUT_MODE")) {
                pcfMode = PCF8574Mode::INPUT_MODE;
            } else if (mode.equalsIgnoreCase("OUTPUT") || mode.equalsIgnoreCase("OUTPUT_MODE")) {
                pcfMode = PCF8574Mode::OUTPUT_MODE;
            }
            
            Serial.print("Creating PCF8574 with mode: ");
            Serial.println(mode.length() > 0 ? mode : "OUTPUT_MODE (default)");
              device = new PCF8574gpio(address, tcaPort, threshold, channels, deviceIndex, pcfMode);
            if (device) {                device->deviceName = deviceName; // Set device name
                device->setChannelThresholds(channelThresholds); // Set channel-specific thresholds
            }
        }
    } else if (type.equalsIgnoreCase("RTC")) {
        if (typeNumber.equalsIgnoreCase("DS3231")) {
            device = new DS3231rtc(address, tcaPort, threshold, channels, deviceIndex);
            if (device) {
                device->deviceName = deviceName; // Set device name
                device->setChannelThresholds(channelThresholds); // Set channel-specific thresholds
            }
        }    } else if (type.equalsIgnoreCase("DAC")) {
        Serial.println("Attempting to create DAC device:");
        Serial.print("  Type: ");
        Serial.println(type);
        Serial.print("  TypeNumber: ");
        Serial.println(typeNumber);
        Serial.print("  Address: 0x");
        Serial.println(address, HEX);
        Serial.print("  TCA Port: ");
        Serial.println(tcaPort);
        Serial.print("  Device Index: ");
        Serial.println(deviceIndex);
        
        // Print channel information
        Serial.println("  Channels:");
        for (const auto& channel : channelNames) {
            Serial.print("    ");
            Serial.print(channel.first);
            Serial.print(": ");
            Serial.println(channel.second);
        }
          if (typeNumber.equalsIgnoreCase("GP8403") || typeNumber.equalsIgnoreCase("MCP4725")) {
            Serial.println("Creating GP8403dac...");
            device = new GP8403dac(address, tcaPort, threshold, channels, deviceIndex);
            if (device) {
                device->deviceName = deviceName; // Set device name
                device->setChannelThresholds(channelThresholds); 
                Serial.println("DAC device created successfully!");
            } else {
                Serial.println("ERROR: Failed to create DAC device - out of memory?");
            }
        } else {
            Serial.print("ERROR: Unknown DAC type: ");
            Serial.println(typeNumber);
        }
    }
    
    if (device) {
        Serial.println("Device created successfully");
        // Register the device in the registry
        DeviceRegistry& registry = DeviceRegistry::getInstance();
        if (registry.registerDevice(device)) {
            Serial.println("Device registered successfully in registry");
            
            // Special debug for DAC device
            if (type.equalsIgnoreCase("DAC")) {
                Serial.print("DAC created with index ");
                Serial.println(deviceIndex);
            }
        } else {
            Serial.println("Failed to register device in registry");
        }
    } else {
        Serial.println("Failed to create device - unknown type/typeNumber combination");
    }
    
    return device;
}
