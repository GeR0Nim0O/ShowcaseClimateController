/*
 * Interface Usage Example
 * 
 * This example demonstrates how to use the Interface class to create a 
 * menu-driven display system for the climate controller.
 * 
 * The Interface coordinates a Display device and RotaryEncoder device to provide:
 * - Default screen showing current temperature/humidity and control status
 * - Menu navigation via encoder button press
 * - Setpoint adjustment via encoder rotation
 * - Automatic timeout back to default screen after 10 seconds of inactivity
 * 
 * Menu structure:
 * 0. Default Screen (T: 22.5°C H: 65.0% / Status)
 * 1. Temperature Setpoint (adjustable 10-40°C)
 * 2. Humidity Setpoint (adjustable 30-90%)
 * 3. Temperature Control Enable/Disable
 * 4. Humidity Control Enable/Disable
 * 
 * Hardware requirements:
 * - ESP32 with I2C
 * - LCD display (2x16) with I2C backpack (e.g., PCF8574)
 * - I2C Rotary Encoder (i2cEncoderLibV2 compatible)
 * - Climate controller system with SHT sensor
 */

#include <Arduino.h>
#include <Wire.h>
#include "DeviceRegistry/DeviceRegistry.h"
#include "ClimateController.h"
#include "Interface.h"
#include "Display.h"
#include "RotaryEncoder.h"

// I2C device addresses (adjust based on your hardware)
#define DISPLAY_I2C_ADDRESS     0x27  // PCF8574 I2C backpack for LCD
#define ENCODER_I2C_ADDRESS     0x61  // i2cEncoderLibV2 default address
#define SHT_SENSOR_ADDRESS      0x44  // SHT40/SHT45 sensor address

// TCA9548A multiplexer channels (if using multiplexer)
#define DISPLAY_TCA_CHANNEL     0
#define ENCODER_TCA_CHANNEL     1
#define SENSOR_TCA_CHANNEL      2

void setupInterfaceExample() {
    Serial.begin(115200);
    Serial.println("Interface Example Starting...");
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(100000); // 100kHz for reliable communication
    
    // Get device registry instance
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    
    // Create and register Display device
    std::map<String, String> displayChannels;
    displayChannels["display"] = "main";
    
    Device* displayDevice = registry.createDevice(
        "Display", "LCD2x16", 
        &Wire, DISPLAY_I2C_ADDRESS, DISPLAY_TCA_CHANNEL, 
        0.0, displayChannels, 0
    );
    
    if (displayDevice) {
        displayDevice->setDeviceLabel("Main Display");
        registry.registerDevice(displayDevice);
        Serial.println("Display device registered");
    } else {
        Serial.println("Failed to create Display device");
        return;
    }
    
    // Create and register RotaryEncoder device
    std::map<String, String> encoderChannels;
    encoderChannels["encoder"] = "main";
    encoderChannels["button"] = "select";
    
    Device* encoderDevice = registry.createDevice(
        "RotaryEncoder", "I2C",
        &Wire, ENCODER_I2C_ADDRESS, ENCODER_TCA_CHANNEL,
        0.0, encoderChannels, 0
    );
    
    if (encoderDevice) {
        encoderDevice->setDeviceLabel("Main Encoder");
        registry.registerDevice(encoderDevice);
        Serial.println("Encoder device registered");
    } else {
        Serial.println("Failed to create RotaryEncoder device");
        return;
    }
    
    // Create and register Interface device (coordinates display and encoder)
    std::map<String, String> interfaceChannels;
    interfaceChannels["interface"] = "main";
    
    Device* interfaceDevice = registry.createDevice(
        "Interface", "",
        &Wire, 0x00, 0, // Interface doesn't need I2C address
        0.0, interfaceChannels, 0
    );
    
    if (interfaceDevice) {
        interfaceDevice->setDeviceLabel("Main Interface");
        registry.registerDevice(interfaceDevice);
        Serial.println("Interface device registered");
    } else {
        Serial.println("Failed to create Interface device");
        return;
    }
    
    // Initialize all devices
    bool initSuccess = registry.initializeAllDevices();
    if (!initSuccess) {
        Serial.println("Some devices failed to initialize");
        return;
    }
    
    // Create climate controller with device discovery
    ClimateController* climateController = ClimateController::createFromDeviceRegistry();
    if (!climateController) {
        Serial.println("Failed to create ClimateController");
        return;
    }
    
    // Set up the interface with climate controller
    Interface* interface = static_cast<Interface*>(registry.getDeviceByType("Interface"));
    if (interface) {
        interface->setClimateController(climateController);
        
        // Configure interface settings
        interface->setTimeoutMs(10000);      // 10 second timeout
        interface->setAdjustmentStep(0.5);   // 0.5 degree/percent steps
        
        Serial.println("Interface setup complete!");
        Serial.println("\nUsage:");
        Serial.println("- Press encoder button to cycle through menus");
        Serial.println("- Rotate encoder to adjust values in setting menus");
        Serial.println("- Interface returns to default screen after 10 seconds");
    } else {
        Serial.println("Failed to get Interface device");
    }
    
    // Print device status
    registry.printDeviceStatus();
}

void loopInterfaceExample() {
    // Get device registry and update all devices
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    
    // Update all devices (including Interface)
    registry.updateAllDevices();
    
    // Update climate controller if it exists
    static ClimateController* climateController = nullptr;
    if (!climateController) {
        // Try to get climate controller on first loop
        Device* device = registry.getDeviceByType("ClimateController");
        if (device) {
            climateController = static_cast<ClimateController*>(device);
        }
    }
    
    if (climateController) {
        ClimateController::updateControllerWithTiming(climateController);
    }
    
    // Small delay to prevent overwhelming the system
    delay(50);
}

/*
 * Integration with main.cpp:
 * 
 * In your main.cpp, you would typically call:
 * 
 * void setup() {
 *     setupInterfaceExample();
 * }
 * 
 * void loop() {
 *     loopInterfaceExample();
 * }
 * 
 * Or integrate the device creation into your existing setup routine.
 */

/*
 * Configuration file example (config.json):
 * 
 * {
 *   "devices": [
 *     {
 *       "type": "Display",
 *       "model": "LCD2x16",
 *       "address": "0x27",
 *       "tcaPort": 0,
 *       "channels": {
 *         "display": "main"
 *       },
 *       "label": "Main Display"
 *     },
 *     {
 *       "type": "RotaryEncoder", 
 *       "model": "I2C",
 *       "address": "0x61",
 *       "tcaPort": 1,
 *       "channels": {
 *         "encoder": "main",
 *         "button": "select"
 *       },
 *       "label": "Main Encoder"
 *     },
 *     {
 *       "type": "Interface",
 *       "model": "",
 *       "address": "0x00",
 *       "tcaPort": 0,
 *       "channels": {
 *         "interface": "main"
 *       },
 *       "label": "Main Interface"
 *     }
 *   ]
 * }
 */
