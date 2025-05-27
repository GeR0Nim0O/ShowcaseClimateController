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

// Encoder pins (direct GPIO)
#define ENCODER_PIN_A        4
#define ENCODER_PIN_B        5
#define ENCODER_BUTTON_PIN   6

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

void setup()
{
    Serial.begin(115200);

    delay(3000);
    Serial.println("Starting I2C scanner...");

    Wire.begin(17, 16); // SDA, SCL pins for ESP32-S3-DevKitC-1

    Serial.println("Scanning I2C bus directly...");
    scanI2CDevices(); // First scan the bus directly

    delay(3000);
    Serial.println("Starting I2C scanner with PCA9548A multiplexer...");
    scanAllChannels(); // Then scan using the multiplexer

    Serial.println("Selecting channel 0 on PCA9548A...");
    selectI2CChannel(0); // Select channel 0 on the PCA9548A

    Serial.println("Initializing PCF8574...");
    pcf8574.begin(); // Initialize the PCF8574 library

    // int x = pcf8574.read8();
    // Serial.print("Read ");
    // Serial.println(x, HEX);
    pcf8574.write8(0x00); // Set all outputs to LOW initially
    pcf8574.write(FAN_INTERIOR, HIGH);  // Set FAN_INTERIOR to LOW
    pcf8574.write(FAN_EXTERIOR, HIGH);   // Set FAN_EXTERIOR to HIGH6
    pcf8574.write(TEMP_ENABLE, HIGH);       // Set HUMIDIFY to HIGH
    pcf8574.write(COOL, HIGH);         // Set TEMP_ENABLE to HIGH

    // Automatically commit and push changes to git
    gitAutoCommitAndPush();
}

void loop()
{

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