#include "DeviceFactory/DeviceFactory.h"

// Hardware-specific includes - only in the factory
#include "Sensor/SHTsensor/SHTsensor.h"
#include "Sensor/BH1705sensor/BH1705sensor.h"
#include "Sensor/SCALESsensor/SCALESsensor.h"
#include "GPIO/PCF8574gpio/PCF8574gpio.h"
#include "DAC/GP8403dac/GP8403dac.h"
#include "RTC/DS3231rtc/DS3231rtc.h"

Device* DeviceFactory::createDevice(
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
    // Validate input parameters
    if (!validateDeviceParameters(type, typeNumber, address)) {
        return nullptr;
    }
    
    // Check available memory
    size_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 10000) {  // Less than 10KB free
        Serial.println("DeviceFactory: WARNING - Low memory detected before device creation");
    }
    
    // Log creation attempt
    logDeviceCreation(type, typeNumber, address, tcaPort, deviceIndex);
    
    // Generate device name
    String deviceName = generateDeviceName(typeNumber, tcaPort, deviceIndex);
    
    // Extract primary threshold (use first threshold found or default)
    float threshold = 1.0f;
    if (!channelThresholds.empty()) {
        threshold = channelThresholds.begin()->second;
    }
    
    Device* device = nullptr;
    
    // Create device based on type
    if (type.equalsIgnoreCase("Sensor")) {
        device = createSensorDevice(wire, typeNumber, address, tcaPort, threshold, 
                                  channelNames, deviceIndex, deviceName, channelThresholds);
    } else if (type.equalsIgnoreCase("GPIO")) {
        device = createGPIODevice(wire, typeNumber, address, tcaPort, threshold, 
                                channelNames, deviceIndex, deviceName, channelThresholds, mode);
    } else if (type.equalsIgnoreCase("DAC")) {
        device = createDACDevice(wire, typeNumber, address, tcaPort, threshold, 
                               channelNames, deviceIndex, deviceName, channelThresholds);
    } else if (type.equalsIgnoreCase("RTC")) {
        device = createRTCDevice(wire, typeNumber, address, tcaPort, threshold, 
                               channelNames, deviceIndex, deviceName, channelThresholds);
    } else {
        Serial.print("DeviceFactory: Unsupported device type: ");
        Serial.println(type);
        return nullptr;
    }
    
    // Validate created device
    if (device && !validateCreatedDevice(device)) {
        delete device;
        return nullptr;
    }
    
    // Log memory usage
    size_t freeHeapAfter = ESP.getFreeHeap();
    if (device) {
        Serial.print("DeviceFactory: Device created, memory used: ");
        Serial.print(freeHeap - freeHeapAfter);
        Serial.println(" bytes");
    }
    
    return device;
}

Device* DeviceFactory::createSensorDevice(
    TwoWire* wire, const String& typeNumber, uint8_t address, uint8_t tcaPort,
    float threshold, const std::map<String, String>& channels, int deviceIndex,
    const String& deviceName, const std::map<String, float>& channelThresholds
) {
    Device* device = nullptr;
    
    if (typeNumber.equalsIgnoreCase("SHT31")) {
        device = new SHTsensor(wire, address, tcaPort, threshold, channels, deviceIndex);
    } else if (typeNumber.equalsIgnoreCase("BH1705")) {
        device = new BH1705sensor(wire, address, tcaPort, threshold, channels, deviceIndex);
    } else if (typeNumber.equalsIgnoreCase("SCALES")) {
        device = new SCALESsensor(wire, address, tcaPort, threshold, channels, deviceIndex);
    } else {
        Serial.print("DeviceFactory: Unsupported sensor type: ");
        Serial.println(typeNumber);
        return nullptr;
    }
    
    if (device) {
        device->deviceName = deviceName;
        device->setChannelThresholds(channelThresholds);
    }
    
    return device;
}

Device* DeviceFactory::createGPIODevice(
    TwoWire* wire, const String& typeNumber, uint8_t address, uint8_t tcaPort,
    float threshold, const std::map<String, String>& channels, int deviceIndex,
    const String& deviceName, const std::map<String, float>& channelThresholds,
    const String& mode
) {
    Device* device = nullptr;
    
    if (typeNumber.equalsIgnoreCase("PCF8574")) {
        // Parse mode from string
        PCF8574Mode pcfMode = PCF8574Mode::OUTPUT_MODE; // Default
        if (mode.equalsIgnoreCase("INPUT") || mode.equalsIgnoreCase("INPUT_MODE")) {
            pcfMode = PCF8574Mode::INPUT_MODE;
        } else if (mode.equalsIgnoreCase("OUTPUT") || mode.equalsIgnoreCase("OUTPUT_MODE")) {
            pcfMode = PCF8574Mode::OUTPUT_MODE;
        }
        
        Serial.print("DeviceFactory: Creating PCF8574 with mode: ");
        Serial.println(mode.length() > 0 ? mode : "OUTPUT_MODE (default)");
        
        device = new PCF8574gpio(wire, address, tcaPort, threshold, channels, deviceIndex, pcfMode);
    } else {
        Serial.print("DeviceFactory: Unsupported GPIO type: ");
        Serial.println(typeNumber);
        return nullptr;
    }
    
    if (device) {
        device->deviceName = deviceName;
        device->setChannelThresholds(channelThresholds);
    }
    
    return device;
}

Device* DeviceFactory::createDACDevice(
    TwoWire* wire, const String& typeNumber, uint8_t address, uint8_t tcaPort,
    float threshold, const std::map<String, String>& channels, int deviceIndex,
    const String& deviceName, const std::map<String, float>& channelThresholds
) {
    Device* device = nullptr;
    
    Serial.println("DeviceFactory: Creating DAC device");
    Serial.print("  TypeNumber: ");
    Serial.println(typeNumber);
    Serial.print("  Address: 0x");
    Serial.println(address, HEX);
    Serial.print("  TCA Port: ");
    Serial.println(tcaPort);
    
    if (typeNumber.equalsIgnoreCase("GP8403") || typeNumber.equalsIgnoreCase("MCP4725")) {
        device = new GP8403dac(wire, address, tcaPort, threshold, channels, deviceIndex);
        
        if (device) {
            Serial.println("DeviceFactory: DAC device created successfully");
        } else {
            Serial.println("DeviceFactory: ERROR - Failed to create DAC device");
        }
    } else {
        Serial.print("DeviceFactory: Unsupported DAC type: ");
        Serial.println(typeNumber);
        return nullptr;
    }
    
    if (device) {
        device->deviceName = deviceName;
        device->setChannelThresholds(channelThresholds);
    }
    
    return device;
}

Device* DeviceFactory::createRTCDevice(
    TwoWire* wire, const String& typeNumber, uint8_t address, uint8_t tcaPort,
    float threshold, const std::map<String, String>& channels, int deviceIndex,
    const String& deviceName, const std::map<String, float>& channelThresholds
) {
    Device* device = nullptr;
    
    if (typeNumber.equalsIgnoreCase("DS3231")) {
        device = new DS3231rtc(wire, address, tcaPort, threshold, channels, deviceIndex);
    } else {
        Serial.print("DeviceFactory: Unsupported RTC type: ");
        Serial.println(typeNumber);
        return nullptr;
    }
    
    if (device) {
        device->deviceName = deviceName;
        device->setChannelThresholds(channelThresholds);
    }
    
    return device;
}

std::vector<std::pair<String, String>> DeviceFactory::getSupportedDevices() {
    return {
        {"Sensor", "SHT31"},
        {"Sensor", "BH1705"},
        {"Sensor", "SCALES"},
        {"GPIO", "PCF8574"},
        {"DAC", "GP8403"},
        {"DAC", "MCP4725"},
        {"RTC", "DS3231"}
    };
}

bool DeviceFactory::isDeviceSupported(const String& type, const String& typeNumber) {
    auto supportedDevices = getSupportedDevices();
    for (const auto& device : supportedDevices) {
        if (device.first.equalsIgnoreCase(type) && device.second.equalsIgnoreCase(typeNumber)) {
            return true;
        }
    }
    return false;
}

String DeviceFactory::generateDeviceName(const String& typeNumber, uint8_t tcaPort, int deviceIndex) {
    return typeNumber + "_TCA" + String(tcaPort) + "_" + String(deviceIndex);
}

bool DeviceFactory::validateDeviceParameters(const String& type, const String& typeNumber, uint8_t address) {
    // Check for null or empty parameters
    if (type.isEmpty() || type.equalsIgnoreCase("null") || 
        typeNumber.isEmpty() || typeNumber.equalsIgnoreCase("null")) {
        Serial.println("DeviceFactory: ERROR - Cannot create device with null or empty type/typeNumber");
        return false;
    }
    
    // Check for invalid address
    if (address == 0) {
        Serial.println("DeviceFactory: ERROR - Cannot create device with invalid address 0x00");
        return false;
    }
    
    // Check if device type is supported
    if (!isDeviceSupported(type, typeNumber)) {
        Serial.print("DeviceFactory: ERROR - Unsupported device: ");
        Serial.print(type);
        Serial.print("/");
        Serial.println(typeNumber);
        return false;
    }
    
    return true;
}

void DeviceFactory::logDeviceCreation(const String& type, const String& typeNumber, 
                                     uint8_t address, uint8_t tcaPort, int deviceIndex) {
    Serial.print("DeviceFactory: Creating device - Type: ");
    Serial.print(type);
    Serial.print(", Model: ");
    Serial.print(typeNumber);
    Serial.print(", Address: 0x");
    Serial.print(address, HEX);
    Serial.print(", TCA Port: ");
    Serial.print(tcaPort);
    Serial.print(", Index: ");
    Serial.println(deviceIndex);
}

bool DeviceFactory::validateCreatedDevice(Device* device) {
    if (!device) {
        Serial.println("DeviceFactory: ERROR - Device creation returned null");
        return false;
    }
    
    // Check if device pointer is in valid memory range (ESP32 specific)
    if ((uint32_t)device < 0x3F800000 || (uint32_t)device > 0x3FFFFFFF) {
        Serial.println("DeviceFactory: ERROR - Device pointer outside valid ESP32 memory range");
        return false;
    }
    
    // Test basic virtual method calls
    try {
        String testType = device->getType();
        uint8_t testAddress = device->getI2CAddress();
        uint8_t testChannel = device->getTCAChannel();
        String testName = device->getDeviceName();
        
        Serial.print("DeviceFactory: Device validation successful - Type: ");
        Serial.print(testType);
        Serial.print(", Name: ");
        Serial.print(testName);
        Serial.print(", Address: 0x");
        Serial.print(testAddress, HEX);
        Serial.print(", Channel: ");
        Serial.println(testChannel);
        
        return true;
    } catch (...) {
        Serial.println("DeviceFactory: ERROR - Device validation failed - virtual method call exception");
        return false;
    }
}
