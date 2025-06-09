/*
 * Interface Usage Example
 * 
 * This example demonstrates how to use the Interface class as a coordination
 * class to create a menu-driven display system for the climate controller.
 * 
 * The Interface coordinates registered Display and RotaryEncoder devices to provide:
 * - Default screen showing current temperature/humidity and control status
 * - Menu navigation via encoder button press
 * - Setpoint adjustment via encoder rotation (0.1°C steps, 1% humidity steps)
 * - Automatic timeout back to default screen after 10 seconds of inactivity
 * - AutoTune status display when PID tuning is active
 * 
 * Menu structure:
 * 0. Default Screen (T: 22.5°C RH: 65% / Status)
 * 1. Temperature Setpoint (adjustable 10-40°C in 0.1°C steps)
 * 2. Humidity Setpoint (adjustable 30-90% in 1% steps)
 * 3. Temperature Control Enable/Disable
 * 4. Humidity Control Enable/Disable
 * 
 * Status displays:
 * - Temperature: HEAT/COOL/OK/OFF
 * - Humidity: HUM/DEHUM/OK/OFF
 * - AutoTune: "PIDtune" when active, "Tuning Done" on completion
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
#define ENCODER_I2C_ADDRESS     0x30  // i2cEncoderLibV2 address
#define SHT_SENSOR_ADDRESS      0x44  // SHT40/SHT45 sensor address

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
        &Wire, DISPLAY_I2C_ADDRESS, 0, 
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
        &Wire, ENCODER_I2C_ADDRESS, 0,
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
    
    // Create Interface coordination class (not a Device)
    Interface* interface = new Interface();
    if (interface) {
        // Set up the interface with climate controller
        interface->setClimateController(climateController);
        
        // Configure interface settings
        interface->setTimeoutMs(10000);      // 10 second timeout
        
        Serial.println("Interface coordination class created!");
        Serial.println("\nUsage:");
        Serial.println("- Press encoder button to cycle through menus");
        Serial.println("- Rotate encoder to adjust values in setting menus");
        Serial.println("- Temperature: 0.1°C steps, Humidity: 1% steps");
        Serial.println("- Interface returns to default screen after 10 seconds");
        Serial.println("- AutoTune status shows when PID tuning is active");
    } else {
        Serial.println("Failed to create Interface coordination class");
    }
    
    // Print device status
    registry.printDeviceStatus();
}

void loopInterfaceExample() {
    // Get device registry and update all devices
    DeviceRegistry& registry = DeviceRegistry::getInstance();
    
    // Update all devices (Display and RotaryEncoder)
    registry.updateAllDevices();
      // Update climate controller if it exists
    static ClimateController* climateController = nullptr;
    if (!climateController) {
        // The climate controller was created in setup, not from DeviceRegistry
        // In a real application, you'd store this reference from setup
        Serial.println("Warning: ClimateController instance not available in loop");
    }
    
    if (climateController) {
        ClimateController::updateControllerWithTiming(climateController);
    }
    
    // Update interface coordination class
    static Interface* interface = nullptr;
    if (!interface) {
        // This would be stored globally in a real application
        // For this example, you'd need to pass it from setup or store globally
        Serial.println("Warning: Interface instance not available in loop");
    } else {
        interface->update();
    }
    
    // Small delay to prevent overwhelming the system
    delay(50);
}

/*
 * Integration with main.cpp:
 * 
 * Since Interface is now a coordination class (not a Device), you need to 
 * manage its lifecycle in your main application:
 * 
 * Interface* interface = nullptr;
 * 
 * void setup() {
 *     setupInterfaceExample();
 *     
 *     // Get the interface instance (you'd need to store it globally)
 *     // interface = getGlobalInterfaceInstance();
 * }
 * 
 * void loop() {
 *     loopInterfaceExample();
 *     
 *     // Update interface manually
 *     if (interface) {
 *         interface->update();
 *     }
 * }
 */

/* * Configuration file example (config.json):
 * 
 * Note: Interface is no longer registered as a Device in the DeviceRegistry.
 * Only the hardware devices (Display and RotaryEncoder) are registered.
 * 
 * {
 *   "devices": [
 *     {
 *       "type": "Display",
 *       "model": "LCD2x16",
 *       "address": "0x27",
 *       "channels": {
 *         "display": "main"
 *       },
 *       "label": "Main Display"
 *     },
 *     {
 *       "type": "RotaryEncoder", 
 *       "model": "I2C",
 *       "address": "0x30",
 *       "channels": {
 *         "encoder": "main",
 *         "button": "select"
 *       },
 *       "label": "Main Encoder"
 *     }
 *   ]
 * }
 * 
 * The Interface coordination class is created separately and coordinates
 * the registered Display and RotaryEncoder devices.
 */
