#include "DeviceRegistry/DeviceRegistry.h"
// Hardware-specific includes - contained within DeviceRegistry.cpp
#include "SHTsensor.h"
#include "BH1705sensor.h" 
#include "SCALESsensor.h"
#include "PCF8574gpio.h"
#include "GP8403dac.h"
#include "DS3231rtc.h"
#include "Display.h"
#include "Relay4Ch.h"
#include "RotaryEncoder.h"
// Note: Interface.h removed - Interface is no longer a Device

// Initialize the static factory registry
std::map<std::pair<String, String>, 
    std::function<Device*(TwoWire*, uint8_t, uint8_t, float, const std::map<String, String>&, int)>> 
    DeviceRegistry::deviceFactory;

DeviceRegistry& DeviceRegistry::getInstance() {
    static DeviceRegistry instance;
    return instance;
}

// Hardware-agnostic device type registration
bool DeviceRegistry::registerDeviceType(const String& type, const String& typeNumber, 
    std::function<Device*(TwoWire*, uint8_t, uint8_t, float, const std::map<String, String>&, int)> creator) {
    
    Serial.print("DeviceRegistry: Registering device type: ");
    Serial.print(type);
    Serial.print("/");
    Serial.println(typeNumber);
    
    deviceFactory[{type, typeNumber}] = creator;
    return true;
}

// Hardware-agnostic device creation
Device* DeviceRegistry::createDevice(const String& type, const String& typeNumber, 
    TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
    const std::map<String, String>& channels, int deviceIndex) {
    
    Serial.print("DeviceRegistry: Creating device - Type: ");
    Serial.print(type);
    Serial.print(", Model: ");
    Serial.println(typeNumber);

    auto it = deviceFactory.find({type, typeNumber});
    if (it != deviceFactory.end()) {
        Serial.println("DeviceRegistry: Device type found in factory registry");
        Device* device = it->second(wire, address, tcaPort, threshold, channels, deviceIndex);
        if (device) {
            Serial.println("DeviceRegistry: Device created successfully via factory");
        } else {
            Serial.println("DeviceRegistry: Device creation failed");
        }
        return device;
    } else {
        Serial.print("DeviceRegistry: ERROR - Device type not registered: ");
        Serial.print(type);
        Serial.print("/");
        Serial.println(typeNumber);
    }
    return nullptr;
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
    // Input validation
    if (type.isEmpty() || typeNumber.isEmpty() || address == 0) {
        Serial.println("DeviceRegistry: Invalid parameters for device creation");
        return nullptr;
    }
    
    // Memory check
    size_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 10000) {
        Serial.println("DeviceRegistry: WARNING - Low memory detected");
    }
    
    // Convert channelThresholds to single threshold (use first or default)
    float threshold = 1.0f;
    if (!channelThresholds.empty()) {
        threshold = channelThresholds.begin()->second;
    }
    
    // Create device using the factory pattern
    Device* device = createDevice(type, typeNumber, wire, address, tcaPort, threshold, channelNames, deviceIndex);
    
    if (device) {
        // Set device name
        String deviceName = typeNumber + "_TCA" + String(tcaPort) + "_" + String(deviceIndex);
        device->deviceName = deviceName;
        
        // Set channel-specific thresholds
        device->setChannelThresholds(channelThresholds);
        
        // Handle special mode parameter (e.g., for GPIO devices)
        // This could be extended to support mode setting in the future
        
        // Register the device
        if (registerDevice(device)) {
            Serial.println("DeviceRegistry: Device registered successfully");
            return device;
        } else {
            Serial.println("DeviceRegistry: Failed to register device");
            delete device;
            return nullptr;
        }
    }
    
    return nullptr;
}

bool DeviceRegistry::registerDevice(Device* device) {
    if (!device) {
        Serial.println("DeviceRegistry: Cannot register null device");
        return false;
    }
    
    // Additional safety checks
    if ((uint32_t)device < 0x3F800000 || (uint32_t)device > 0x3FFFFFFF) {
        Serial.println("DeviceRegistry: Device pointer outside valid memory range");
        return false;
    }
    
    // Test virtual method calls before registration
    try {
        String testName = device->getDeviceName();
        String testType = device->getType();
        
        Serial.print("DeviceRegistry: Registering device: ");
        Serial.print(testName);
        Serial.print(" (");
        Serial.print(testType);
        Serial.println(")");
        
        // Add to main devices vector - simple and generic
        devices.push_back(device);
        
        Serial.println("DeviceRegistry: Device successfully registered");
        return true;
        
    } catch (...) {
        Serial.println("DeviceRegistry: Registration failed - virtual method call exception");
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

// Generic device getters - type-agnostic and easily extensible
std::vector<Device*> DeviceRegistry::getDevicesByType(const String& type) {
    std::vector<Device*> result;
    for (Device* device : devices) {
        if (device && isTypeMatch(device->getType(), type)) {
            result.push_back(device);
        }
    }
    return result;
}

Device* DeviceRegistry::getDeviceByType(const String& type, int index) {
    int currentIndex = 0;
    for (Device* device : devices) {
        if (device && isTypeMatch(device->getType(), type)) {
            if (currentIndex == index) {
                return device;
            }
            currentIndex++;
        }
    }
    return nullptr;
}

Device* DeviceRegistry::getDeviceByTypeAndLabel(const String& type, const String& label) {
    for (Device* device : devices) {
        if (device && isTypeMatch(device->getType(), type) && device->getDeviceLabel().equalsIgnoreCase(label)) {
            return device;
        }
    }
    return nullptr;
}

std::vector<Device*> DeviceRegistry::getDevicesByLabel(const String& label) {
    std::vector<Device*> result;
    for (Device* device : devices) {
        if (device && device->getDeviceLabel().equalsIgnoreCase(label)) {
            result.push_back(device);
        }
    }
    return result;
}

int DeviceRegistry::getDeviceCountByType(const String& type) const {
    int count = 0;
    for (Device* device : devices) {
        if (device && isTypeMatch(device->getType(), type)) {
            count++;
        }
    }
    return count;
}

bool DeviceRegistry::isTypeMatch(const String& deviceType, const String& searchType) const {
    // Case-insensitive type matching with common aliases
    if (deviceType.equalsIgnoreCase(searchType)) {
        return true;
    }
    
    // Handle common type aliases for backward compatibility
    if (searchType.equalsIgnoreCase("GPIO") && 
        (deviceType.equalsIgnoreCase("PCF8574gpio") || deviceType.equalsIgnoreCase("PCF8574GPIO"))) {
        return true;
    }
    
    if (searchType.equalsIgnoreCase("DAC") && 
        deviceType.equalsIgnoreCase("GP8403dac")) {
        return true;
    }
    
    if (searchType.equalsIgnoreCase("TemperatureHumidity") && 
        deviceType.equalsIgnoreCase("SHTSensor")) {
        return true;
    }
    
    return false;
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

// Hardware-specific device registrations using lambda functions
// This keeps all hardware-specific code contained within DeviceRegistry.cpp
namespace {    bool registeredSHT = DeviceRegistry::registerDeviceType("Sensor", "SHT", 
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) {
        return new SHTsensor(wire, address, tcaPort, threshold, channels, deviceIndex);
    });

    bool registeredBH1705 = DeviceRegistry::registerDeviceType("Sensor", "BH1705", 
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) {
        return new BH1705sensor(wire, address, tcaPort, threshold, channels, deviceIndex);
    });

    bool registeredSCALES = DeviceRegistry::registerDeviceType("Sensor", "SCALES", 
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) {
        return new SCALESsensor(wire, address, tcaPort, threshold, channels, deviceIndex);
    });    bool registeredPCF8574 = DeviceRegistry::registerDeviceType("GPIO", "PCF8574", 
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) {
        // Note: This basic registration doesn't handle mode parameter
        // For full mode support, the factory pattern would need extension
        return new PCF8574gpio(wire, address, tcaPort, threshold, channels, deviceIndex, PCF8574Mode::OUTPUT_MODE);
    });

    bool registeredRelay4Ch = DeviceRegistry::registerDeviceType("Relay", "Relay4Ch", 
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) {
        // Default to SYNC mode for relay control
        return new Relay4Ch(wire, address, tcaPort, threshold, channels, deviceIndex, Relay4ChMode::SYNC);
    });

    bool registeredGP8403 = DeviceRegistry::registerDeviceType("DAC", "GP8403", 
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) {
        return new GP8403dac(wire, address, tcaPort, threshold, channels, deviceIndex);
    });

    bool registeredMCP4725 = DeviceRegistry::registerDeviceType("DAC", "MCP4725", 
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) {
        return new GP8403dac(wire, address, tcaPort, threshold, channels, deviceIndex);
    });    bool registeredDS3231 = DeviceRegistry::registerDeviceType("RTC", "DS3231", 
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) {
        return new DS3231rtc(wire, address, tcaPort, threshold, channels, deviceIndex);
    });    bool registeredDisplay = DeviceRegistry::registerDeviceType("Display", "LCD2x16", 
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) {
        String deviceName = "LCD_Display_" + String(deviceIndex);
        return new Display(wire, address, tcaPort, deviceName, deviceIndex);
    });    bool registeredRotaryEncoder = DeviceRegistry::registerDeviceType("RotaryEncoder", "I2C", 
        [](TwoWire* wire, uint8_t address, uint8_t tcaPort, float threshold, 
           const std::map<String, String>& channels, int deviceIndex) {
        String deviceName = "RotaryEncoder_" + String(deviceIndex);
        return new RotaryEncoder(wire, address, tcaPort, deviceName, deviceIndex);
    });

    // Note: Interface is no longer a Device - it's a coordination class
    // Interface is created manually and coordinates the registered Display and RotaryEncoder devices
}
