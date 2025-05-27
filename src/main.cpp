#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <stdlib.h> // Added for system()

// Include our custom classes
#include "DeviceRegistry.h"
#include "PCF8574_GPIO.h"
#include "SHT31_Sensor.h"
#include "Display.h"
#include "DAC_Module.h"
#include "RotaryEncoder.h"
#include "ClimateController.h"
#include "ClimateConfig.h"

// Define the I2C address of the PCA9548A multiplexer
#define PCA9548A_ADDRESS 0x70

// Device I2C addresses and TCA channels
#define GPIO_EXPANDER_ADDR    0x20
#define GPIO_EXPANDER_CHANNEL 0
#define SHT31_ADDR           0x44
#define SHT31_CHANNEL        1
#define DISPLAY_ADDR         0x3C
#define DISPLAY_CHANNEL      2
#define DAC_ADDR             0x62
#define DAC_CHANNEL          3
#define ENCODER_ADDR         0x61
#define ENCODER_CHANNEL      4

// Global objects
DeviceRegistry& deviceRegistry = DeviceRegistry::getInstance();
ClimateConfig& config = ClimateConfig::getInstance();

// Device instances
PCF8574_GPIO* gpioExpander = nullptr;
SHT31_Sensor* tempHumSensor = nullptr;
Display* display = nullptr;
DAC_Module* dacModule = nullptr;
RotaryEncoder* encoder = nullptr;
ClimateController* climateController = nullptr;

void gitAutoCommitAndPush() {
    // Commit and push all changes with a timestamped message
    system("git add .");
    system("git commit -m \"Auto-commit from device\"");
    system("git push");
}

void initializeDevices() {
    Serial.println("=== Initializing Climate Controller Devices ===");
    
    // Create device instances
    gpioExpander = new PCF8574_GPIO(GPIO_EXPANDER_ADDR, GPIO_EXPANDER_CHANNEL, "GPIO_Expander", 0);
    tempHumSensor = new SHT31_Sensor(SHT31_ADDR, SHT31_CHANNEL, "Temperature_Humidity_Sensor", 0);
    display = new Display(DISPLAY_ADDR, DISPLAY_CHANNEL, "OLED_Display", 0);
    dacModule = new DAC_Module(DAC_ADDR, DAC_CHANNEL, "DAC_Module", 0);
    encoder = new RotaryEncoder(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_BUTTON_PIN, "Rotary_Encoder", 0);
    
    // Register devices with the device registry
    deviceRegistry.registerDevice(gpioExpander);
    deviceRegistry.registerDevice(tempHumSensor);
    deviceRegistry.registerDevice(display);
    deviceRegistry.registerDevice(dacModule);
    deviceRegistry.registerDevice(encoder);
    
    // Initialize all devices
    bool allInitialized = deviceRegistry.initializeAllDevices();
    
    if (allInitialized) {
        Serial.println("All devices initialized successfully!");
        
        // Create climate controller
        climateController = new ClimateController(gpioExpander, tempHumSensor);
        if (climateController->begin()) {
            Serial.println("Climate Controller initialized successfully!");
            
            // Configure climate controller with saved settings
            climateController->setTemperatureSetpoint(config.getTemperatureSetpoint());
            climateController->setHumiditySetpoint(config.getHumiditySetpoint());
            climateController->setTemperaturePID(config.getTemperatureKp(), config.getTemperatureKi(), config.getTemperatureKd());
            climateController->setHumidityPID(config.getHumidityKp(), config.getHumidityKi(), config.getHumidityKd());
            climateController->setFanInterior(config.getFanInteriorEnabled());
            climateController->setFanExterior(config.getFanExteriorEnabled());
            
            // Configure encoder for menu navigation
            encoder->setMinMax(0, 100);
            encoder->setStepSize(1);
            
            // Show status on display
            if (display->isConnected()) {
                display->displaySystemStatus("System Ready");
                delay(2000);
            }
        } else {
            Serial.println("Failed to initialize Climate Controller!");
        }
    } else {
        Serial.println("Some devices failed to initialize!");
    }
    
    deviceRegistry.printDeviceStatus();
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("=== Climate Controller Starting ===");
    
    // Initialize I2C
    Wire.begin(17, 16); // SDA, SCL pins for ESP32-S3-DevKitC-1
    
    // Initialize configuration
    if (!config.begin()) {
        Serial.println("Failed to initialize configuration!");
    }
    
    // Scan I2C devices for debugging
    Serial.println("Scanning I2C bus directly...");
    scanI2CDevices();
    
    delay(1000);
    Serial.println("Scanning I2C bus with PCA9548A multiplexer...");
    scanAllChannels();
    
    // Initialize all devices
    initializeDevices();
    
    // Automatically commit and push changes to git
    gitAutoCommitAndPush();
    
    Serial.println("=== Setup Complete ===");
}

void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastDisplayUpdate = 0;
    static unsigned long lastEncoderCheck = 0;
    
    unsigned long currentTime = millis();
    
    // Update climate controller (every 1 second)
    if (currentTime - lastUpdate >= 1000) {
        if (climateController) {
            climateController->update();
        }
        
        // Update all devices
        deviceRegistry.updateAllDevices();
        
        lastUpdate = currentTime;
    }
    
    // Update display (every 2 seconds)
    if (currentTime - lastDisplayUpdate >= 2000) {
        if (display && display->isConnected() && tempHumSensor) {
            display->displayClimateStatus(
                tempHumSensor->getTemperature(),
                tempHumSensor->getHumidity(),
                config.getTemperatureSetpoint(),
                config.getHumiditySetpoint()
            );
        }
        lastDisplayUpdate = currentTime;
    }
    
    // Check encoder input (every 50ms for responsiveness)
    if (currentTime - lastEncoderCheck >= 50) {
        if (encoder) {
            encoder->update();
            
            // Handle encoder rotation for setpoint adjustment
            long positionChange = encoder->getPositionChange();
            if (positionChange != 0) {
                // Adjust temperature setpoint (example)
                float newSetpoint = config.getTemperatureSetpoint() + (positionChange * 0.5);
                newSetpoint = constrain(newSetpoint, 10.0, 35.0);
                config.setTemperatureSetpoint(newSetpoint);
                
                if (climateController) {
                    climateController->setTemperatureSetpoint(newSetpoint);
                }
                
                Serial.print("New temperature setpoint: ");
                Serial.println(newSetpoint);
            }
            
            // Handle button press for saving settings
            if (encoder->wasButtonPressed()) {
                config.saveSettings();
                Serial.println("Settings saved!");
                
                if (display && display->isConnected()) {
                    display->displaySystemStatus("Settings Saved!");
                    delay(1000);
                }
            }
        }
        lastEncoderCheck = currentTime;
    }
    
    // Small delay to prevent overwhelming the system
    delay(10);
}

// Function to select a channel on the PCA9548A
void selectI2CChannel(uint8_t channel)
{
    if (channel > 7)
        return; // PCA9548A has 8 channels (0-7)
    Wire.beginTransmission(PCA9548A_ADDRESS);
    Wire.write(1 << channel); // Select the channel
    Wire.endTransmission();
}

// Function to scan I2C devices on the current channel
void scanI2CDevices()
{
    Serial.println("Scanning I2C bus...");
    for (uint8_t address = 1; address < 127; ++address)
    {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0)
        {
            Serial.print("Found I2C device at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
}

// Function to scan all channels of the PCA9548A
void scanAllChannels()
{
    for (uint8_t channel = 0; channel < 8; ++channel)
    {
        Serial.print("Scanning channel ");
        Serial.println(channel);
        selectI2CChannel(channel);
        scanI2CDevices();
    }
}