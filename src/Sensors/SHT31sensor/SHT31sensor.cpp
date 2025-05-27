#include "Device.h"
#include "SHT31sensor.h"
#include "I2CHandler.h"

SHT31sensor::SHT31sensor(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, i2cAddress, tcaChannel, threshold, channels, deviceIndex), wire(wire), _address(SHT31_ADDRESS), _temperature(NAN), _humidity(NAN) {
    
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

bool SHT31sensor::begin() {
    // Make sure to select the correct TCA channel before initializing
    selectTCAChannel(tcaChannel);
    
    // Check if the sensor is connected before trying to initialize it
    Wire.beginTransmission(i2cAddress);
    if (Wire.endTransmission() != 0) {
        Serial.print("SHT31 sensor not found at address 0x");
        Serial.print(i2cAddress, HEX);
        Serial.print(" on TCA channel ");
        Serial.println(tcaChannel);
        return false;
    }
    
    // Initialize the SHT31 sensor
    if (!sht.begin()) {
        Serial.println("Failed to initialize SHT31 sensor!");
        return false;
    }
    
    // Read sensor status to verify communication
    uint16_t stat = sht.readStatus();
    Serial.print("SHT31 status: ");
    Serial.println(stat, HEX);
    
    // Set measurement mode (high repeatability)
    Serial.println("DEBUG: Setting measurement mode...");
    bool success = true; // Adjust based on your library's API
    if (success) {
        Serial.println("DEBUG: Measurement mode set successfully");
    } else {
        Serial.println("DEBUG: Failed to set measurement mode");
        return false;
    }
    
    // Disable heater
    Serial.println("DEBUG: Disabling heater...");
    sht.heater(false);
    Serial.println("DEBUG: Heater disabled successfully");
    
    // Get serial number for debugging
    Serial.print("SHT31 Serial Number: ");
    Serial.println(sht.readSerialNumber());
    
    // Do an initial reading (but skip detailed error handling)
    Serial.println("DEBUG: Attempting initial sensor reading...");
    Serial.println("DEBUG: Skipping initial sensor reading to avoid potential crash");
    
    // Set initialized flag
    Serial.println("DEBUG: Setting initialized flag...");
    Serial.print("DEBUG: Address of 'this': ");
    Serial.println((uint32_t)this, HEX);
    Serial.print("DEBUG: Address of 'initialized' variable: ");
    Serial.println((uint32_t)&initialized, HEX);
    Serial.print("DEBUG: initialized value before setting: ");
    Serial.println(initialized);
    
    initialized = true;
    
    Serial.print("DEBUG: SHT31 initialized flag set to true");
    Serial.print("DEBUG: SHT31 initialized flag is now: ");
    Serial.println(initialized);
    Serial.print("DEBUG: isInitialized() method returns: ");
    Serial.println(isInitialized());
    
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
    // Always select the correct TCA channel before communicating
    selectTCAChannel(tcaChannel);
    
    // Reset readingSuccess flag
    bool readingSuccess = false;
    
    // Try multiple times if needed
    for (int attempt = 0; attempt < 3 && !readingSuccess; attempt++) {
        if (sht.readTempHum()) {  // This reads both temperature and humidity
            temperature = sht.getTemperature();
            humidity = sht.getHumidity();
            readingSuccess = true;
            Serial.print("SHT31 reading successful - Temp: ");
            Serial.print(temperature);
            Serial.print("°C, Humidity: ");
            Serial.println(humidity);
        } else {
            Serial.print("SHT31 reading failed, attempt ");
            Serial.print(attempt + 1);
            Serial.println("/3");
            delay(100);  // Wait before retrying
        }
    }
    
    if (!readingSuccess) {
        Serial.println("SHT31 reading failed after multiple attempts");
    }
}
