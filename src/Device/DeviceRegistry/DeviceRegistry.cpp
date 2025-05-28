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
    
    // Additional safety checks
    if ((uint32_t)device < 0x3F800000 || (uint32_t)device > 0x3FFFFFFF) {
        Serial.println("Cannot register device - pointer outside valid memory range");
        return false;
    }
    
    // Test virtual method calls before registration
    try {
        String testName = device->getDeviceName();
        String testType = device->getType();
        
        Serial.print("Registering device: ");
        Serial.print(testName);
        Serial.print(" (");
        Serial.print(testType);
        Serial.println(")");
        
        devices.push_back(device);
        
        Serial.println("Device successfully registered");
        return true;
        
    } catch (...) {
        Serial.println("Cannot register device - virtual method call failed");
        return false;
    }
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
    
    // Check available heap memory before creating device
    size_t freeHeap = ESP.getFreeHeap();
    size_t minFreeHeap = ESP.getMinFreeHeap();
    Serial.print("Free heap before device creation: ");
    Serial.print(freeHeap);
    Serial.print(" bytes, Min free heap: ");
    Serial.print(minFreeHeap);
    Serial.println(" bytes");
    
    if (freeHeap < 10000) {  // Less than 10KB free
        Serial.println("WARNING: Low memory detected before device creation");
    }
    
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
            }        } else if (typeNumber.equalsIgnoreCase("BH1705")) {
            device = new BH1705sensor(wire, address, tcaPort, threshold, channels, deviceIndex);
            if (device) {
                device->deviceName = deviceName; // Set device name
                device->setChannelThresholds(channelThresholds); // Set channel-specific thresholds
            }        } else if (typeNumber.equalsIgnoreCase("SCALES")) {
            device = new SCALESsensor(wire, address, tcaPort, threshold, channels, deviceIndex);
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
            Serial.println(mode.length() > 0 ? mode : "OUTPUT_MODE (default)");              device = new PCF8574gpio(wire, address, tcaPort, threshold, channels, deviceIndex, pcfMode);
            if (device) {device->deviceName = deviceName; // Set device name
                device->setChannelThresholds(channelThresholds); // Set channel-specific thresholds
            }
        }
    } else if (type.equalsIgnoreCase("RTC")) {
        if (typeNumber.equalsIgnoreCase("DS3231")) {            device = new DS3231rtc(wire, address, tcaPort, threshold, channels, deviceIndex);
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
            Serial.println("Creating GP8403dac...");            device = new GP8403dac(wire, address, tcaPort, threshold, channels, deviceIndex);
            if (device) {
                device->deviceName = deviceName; // Set device name
                device->setChannelThresholds(channelThresholds); 
                Serial.println("DAC device created successfully!");
            } else {
                Serial.println("ERROR: Failed to create DAC device - out of memory?");
            }
        } else {
            Serial.print("ERROR: Unknown DAC type: ");
            Serial.println(typeNumber);        }
    }
    
    // Check heap after device creation attempt
    size_t freeHeapAfter = ESP.getFreeHeap();
    Serial.print("Free heap after device creation attempt: ");
    Serial.print(freeHeapAfter);
    Serial.println(" bytes");
    
    if (device) {
        Serial.print("Device object created, memory used: ");
        Serial.print(freeHeap - freeHeapAfter);
        Serial.println(" bytes");
    }
    
      if (device) {
        Serial.println("Device created successfully");
        
        // Add device validation before registration
        Serial.println("Validating device object integrity...");
        
        // Check if device pointer is in valid memory range
        if ((uint32_t)device < 0x3F800000 || (uint32_t)device > 0x3FFFFFFF) {
            Serial.println("ERROR: Device pointer is outside valid ESP32 memory range");
            delete device;
            return nullptr;
        }
        
        // Test basic virtual method calls
        try {
            String testType = device->getType();
            uint8_t testAddress = device->getI2CAddress();
            uint8_t testChannel = device->getTCAChannel();
            
            Serial.print("Device validation successful - Type: ");
            Serial.print(testType);
            Serial.print(", Address: 0x");
            Serial.print(testAddress, HEX);
            Serial.print(", Channel: ");
            Serial.println(testChannel);
        } catch (...) {
            Serial.println("ERROR: Device validation failed - virtual method call exception");
            delete device;
            return nullptr;
        }
        
        // Add a small delay to ensure memory is stable
        delay(10);
        yield();
        
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
            delete device;
            return nullptr;
        }
    } else {
        Serial.println("Failed to create device - unknown type/typeNumber combination");
    }
    
    return device;
}
