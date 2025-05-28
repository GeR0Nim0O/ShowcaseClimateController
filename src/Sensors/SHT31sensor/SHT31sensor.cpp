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
    
    // Read sensor status to verify communication
    uint16_t stat = readStatus();
    Serial.print("SHT31 status: ");
    Serial.println(stat, HEX);
    
    // Set measurement mode (high repeatability)
    Serial.println("DEBUG: Setting measurement mode...");
    bool success = setMeasurementMode(0x2C06); // High repeatability, clock stretching disabled
    if (success) {
        Serial.println("DEBUG: Measurement mode set successfully");
    } else {
        Serial.println("DEBUG: Failed to set measurement mode");
        return false;
    }
    
    // Disable heater
    Serial.println("DEBUG: Disabling heater...");
    setHeater(false);
    Serial.println("DEBUG: Heater disabled successfully");
    
    // Get serial number for debugging
    Serial.print("SHT31 Serial Number: ");
    Serial.println(getSerialNumber());
    
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
    // Make sure to select the TCA channel before communication
    I2CHandler::selectTCA(getTCAChannel());
    
    // Send measurement command - high repeatability, clock stretching disabled
    if (!writeCommand(0x2C06))
    {
        Serial.println("SHT31 readRawData: writeCommand error");
        return false;
    }

    // Wait for measurement to complete (datasheet specifies 15ms for high repeatability)
    delay(20);

    // Read the 6 bytes of data: Temp MSB, Temp LSB, Temp CRC, Humidity MSB, Humidity LSB, Humidity CRC
    uint8_t data[6];
    if (!readBytes(data, 6))
    {
        Serial.println("SHT31 readRawData: readBytes error");
        return false;
    }    // Extract raw temperature and humidity values from received bytes
    rawTemperature = (data[0] << 8) | data[1];
    rawHumidity = (data[3] << 8) | data[4];

    return true;
}

std::map<String, String> SHT31sensor::readData()
{
    std::map<String, String> result;
    uint16_t rawTemperature, rawHumidity;
    
    if (readRawData(rawTemperature, rawHumidity))
    {
        // Convert raw values to actual temperature and humidity values
        // T = -45 + 175 * (rawTemperature / 65535.0)
        // RH = 100 * (rawHumidity / 65535.0)
        _temperature = -45.0f + 175.0f * (float(rawTemperature) / 65535.0f);
        _humidity = 100.0f * (float(rawHumidity) / 65535.0f);
        
        Serial.print("SHT31 converted values - Temp: ");
        Serial.print(_temperature);
        Serial.print("°C, Humidity: ");
        Serial.println(_humidity);
    }
    else
    {
        Serial.println("SHT31 readData: Failed to read raw data, using previous values");
        // Keep previous values if read fails
    }

    // Map our readings to the expected channel keys
    for (const auto& channel : channels) {
        if (channel.second == "Temperature" || channel.second == "T") {
            result[channel.first] = String(_temperature, 2); // 2 decimal places
        } 
        else if (channel.second == "Humidity" || channel.second == "H") {
            result[channel.first] = String(_humidity, 2); // 2 decimal places
        }
    }
    
    return result;
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
        uint16_t rawTemperature, rawHumidity;
        if (readRawData(rawTemperature, rawHumidity)) {
            _temperature = -45 + 175 * (rawTemperature / 65535.0);
            _humidity = 100 * (rawHumidity / 65535.0);
            readingSuccess = true;
            Serial.print("SHT31 reading successful - Temp: ");
            Serial.print(_temperature);
            Serial.print("°C, Humidity: ");
            Serial.println(_humidity);
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
