#include "Device.h"
#include "SHT31sensor.h"
#include "I2CHandler.h"

SHT31sensor::SHT31sensor(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, i2cChannel, tcaPort, threshold, channels, deviceIndex), wire(wire), _address(SHT31_ADDRESS), _temperature(NAN), _humidity(NAN) {
    
    // Set type - inherited from Device base class
    type = "SHT31Sensor";

    Serial.println("SHT31sensor created:");
    Serial.print("Address: ");
    Serial.println(_address, HEX);
    Serial.print("Threshold: ");
    Serial.println(threshold);
    Serial.print("Number of Channels: ");
    Serial.println(channels.size());
    Serial.print("Type: ");
    Serial.println(type);
    Serial.print("Device Index: ");
    Serial.println(deviceIndex);
}

bool SHT31sensor::begin()
{
    // Serial.println("DEBUG: SHT31sensor::begin() START");
    I2CHandler::selectTCA(tcaChannel); // Use tcaChannel from base Device class
    
    // First test I2C connection
    wire->beginTransmission(_address);
    uint8_t error = wire->endTransmission();
    if (error != 0) {
        Serial.print("SHT31 not found at address 0x");
        Serial.print(_address, HEX);
        Serial.print(" on TCA port ");
        Serial.print(tcaChannel);
        Serial.print(", I2C error: ");
        Serial.println(error);
        return false;
    }
    
    // Soft reset with retry
    bool resetSuccess = false;
    for (int i = 0; i < 3 && !resetSuccess; i++) {
        if (writeCommand(0x30A2)) { // Soft reset command
            resetSuccess = true;
        } else {
            Serial.print("SHT31 soft reset attempt ");
            Serial.print(i + 1);
            Serial.println(" failed, retrying...");
            delay(10);
        }
    }
    
    if (!resetSuccess) {
        Serial.println("SHT31 begin error (soft reset failed after retries)");
        return false;
    }
    
    delay(10); // Wait for reset to complete

    // Read the status register to ensure the sensor is ready
    uint16_t status = readStatus();
    if (status == 0xFFFF)
    {
        Serial.println("SHT31 begin error (read status)");
        return false;
    }

    Serial.print("SHT31 status: ");
    Serial.println(status, HEX);

    // Set measurement mode to high repeatability
    if (!setMeasurementMode(0x2400))
    {
        Serial.println("Failed to set measurement mode");
        return false;
    }

    // Disable heater
    if (!setHeater(false))
    {
        Serial.println("Failed to disable heater");
        return false;
    }    // Print SHT31 serial number
    Serial.print("SHT31 Serial Number: ");
    Serial.println(getSerialNumber());    // Read initial values using readData
    auto sht31Data = readData();
    if (sht31Data.find("T") != sht31Data.end() && sht31Data.find("H") != sht31Data.end()) {
        float temperature = sht31Data["T"].toFloat();
        float humidity = sht31Data["H"].toFloat();
        Serial.print("Initial Temperature: ");
        Serial.println(temperature);
        Serial.print("Initial Humidity: ");
        Serial.println(humidity);
    }    initialized = true; // Set initialized flag to true
    Serial.println("DEBUG: SHT31 initialized flag set to true");
    Serial.print("DEBUG: SHT31 initialized flag is now: ");
    Serial.println(initialized);
    Serial.println("SHT31 sensor initialized successfully");
    Serial.println("DEBUG: SHT31sensor::begin() returning true");
    return true;
}

bool SHT31sensor::readRawData(uint16_t &rawTemperature, uint16_t &rawHumidity)
{
    I2CHandler::selectTCA(getTCAChannel());
    if (!writeCommand(0x2C06)) // Command to read data
    {
        Serial.println("SHT31 readRawData: writeCommand error");
        return false;
    }

    delay(500); // Wait for data to be available

    uint8_t data[6];
    if (!readBytes(data, 6))
    {
        Serial.println("SHT31 readRawData: readBytes error");
        return false;
    }

    rawTemperature = (data[0] << 8) | data[1];
    rawHumidity = (data[3] << 8) | data[4];

    return true;
}

std::map<String, String> SHT31sensor::readData()
{
    uint16_t rawTemperature, rawHumidity;
    if (!readRawData(rawTemperature, rawHumidity))
    {
        return {{"T", "NAN"}, {"H", "NAN"}};
    }

    _temperature = -45 + 175 * (rawTemperature / 65535.0);
    _humidity = 100 * (rawHumidity / 65535.0);

    std::map<String, String> data;
    for (const auto& channel : channels) {
        if (channel.second == "T") {
            data[channel.first] = String(_temperature);
        } else if (channel.second == "H") {
            data[channel.first] = String(_humidity);
        }
    }

    return data;
}

uint32_t SHT31sensor::getSerialNumber()
{
    I2CHandler::selectTCA(getTCAChannel()); 
    if (!writeCommand(0x3780)) // Command to read serial number
    {
        Serial.println("SHT31 getSerialNumber: writeCommand error");
        return 0;
    }

    delay(500); // Wait for data to be available

    uint8_t data[4];
    if (!readBytes(data, 4))
    {
        Serial.println("SHT31 getSerialNumber: readBytes error");
        return 0;
    }

    uint32_t serialNumber = 0;
    for (int i = 0; i < 4; i++) {
        serialNumber = (serialNumber << 8) | data[i];
    }

    return serialNumber;
}

uint16_t SHT31sensor::readStatus()
{
    I2CHandler::selectTCA(getTCAChannel()); 
    if (!writeCommand(0xF32D)) // Command to read statusregister
    {
        Serial.println("SHT31 readStatus: writeCommand error");
        return 0xFFFF;
    }

    delay(20); // Wait for data to be available

    uint8_t data[3];
    if (!readBytes(data, 3))
    {
        Serial.println("SHT31 readStatus: readBytes error");
        return 0xFFFF;
    }

    return (data[0] << 8) | data[1];
}

bool SHT31sensor::clearStatus()
{
    I2CHandler::selectTCA(getTCAChannel()); 
    if (!writeCommand(0x3041)) // Command to clear statusregister
    {
        Serial.println("SHT31 clearStatus: writeCommand error");
        return false;
    }

    delay(20); // Wait for command to complete

    return true;
}

bool SHT31sensor::setMeasurementMode(uint16_t mode)
{
    I2CHandler::selectTCA(getTCAChannel()); 
    if (!writeCommand(mode))
    {
        Serial.println("SHT31 setMeasurementMode: writeCommand error");
        return false;
    }

    delay(20); // Wait for command to complete

    return true;
}

bool SHT31sensor::setHeater(bool enable)
{
    I2CHandler::selectTCA(getTCAChannel()); 
    if (!writeCommand(enable ? 0x306D : 0x3066)) // Command to enable/disable heater
    {
        Serial.println("SHT31 setHeater: writeCommand error");
        return false;
    }

    delay(20); // Wait for command to complete

    return true;
}

bool SHT31sensor::writeCommand(uint16_t command)
{
    wire->beginTransmission(_address);
    wire->write(command >> 8); // Send MSB
    wire->write(command & 0xFF); // Send LSB
    return wire->endTransmission() == 0;
}

bool SHT31sensor::readBytes(uint8_t *data, uint8_t length)
{
    if (wire->requestFrom(_address, length) != length)
    {
        return false;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = wire->read();
    }

    return true;
}

float SHT31sensor::getTemperature() const {
    return _temperature;
}

float SHT31sensor::getHumidity() const {
    return _humidity;
}

// Implementation of pure virtual methods from Device base class
bool SHT31sensor::isConnected() {
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(_address);
    return (wire->endTransmission() == 0);
}

void SHT31sensor::update() {
    // Update sensor readings
    readData();
}
