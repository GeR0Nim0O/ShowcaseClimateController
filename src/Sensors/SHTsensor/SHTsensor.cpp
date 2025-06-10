#include "Device.h"
#include "SHTsensor.h"
#include "I2CHandler.h"

SHTsensor::SHTsensor(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, threshold, channels, i2cAddress, tcaChannel, deviceIndex), _address(SHT_ADDRESS), _temperature(NAN), _humidity(NAN) {
    
    // Set type - inherited from Device base class
    type = "SHTSensor";

    Serial.println("SHTsensor created:");
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

bool SHTsensor::begin() {
    // Make sure to select the correct TCA channel before initializing    selectTCAChannel(tcaChannel);
    
    // Check if the sensor is connected before trying to initialize it
    wire->beginTransmission(i2cAddress);
    if (wire->endTransmission() != 0) {
        Serial.print("SHT sensor not found at address 0x");
        Serial.print(i2cAddress, HEX);
        Serial.print(" on TCA channel ");
        Serial.println(tcaChannel);
        return false;
    }
    
    // Read sensor status to verify communication
    uint16_t stat = readStatus();
    Serial.print("SHT status: ");
    Serial.println(stat, HEX);
      // Set measurement mode (high repeatability)
    bool success = setMeasurementMode(0x2C06); // High repeatability, clock stretching disabled
    if (!success) {
        Serial.println("Failed to set measurement mode");
        return false;    }
    
    // Disable heater
    setHeater(false);
    
    // Set initialized flag
    initialized = true;
    
    return true;
}

bool SHTsensor::readRawData(uint16_t &rawTemperature, uint16_t &rawHumidity)
{
    // Make sure to select the TCA channel before communication
    I2CHandler::selectTCA(getTCAChannel());
      // Send measurement command - high repeatability, clock stretching disabled
    if (!writeCommand(0x2C06))
    {
        Serial.print("SHT readRawData: writeCommand error on TCA");
        Serial.println(getTCAChannel());
        return false;
    }

    // Wait for measurement to complete (datasheet specifies 15ms for high repeatability)
    delay(20);

    // Read the 6 bytes of data: Temp MSB, Temp LSB, Temp CRC, Humidity MSB, Humidity LSB, Humidity CRC
    uint8_t data[6];
    if (!readBytes(data, 6))
    {
        Serial.print("SHT readRawData: readBytes error on TCA");
        Serial.println(getTCAChannel());
        return false;
    }    // Extract raw temperature and humidity values from received bytes
    rawTemperature = (data[0] << 8) | data[1];
    rawHumidity = (data[3] << 8) | data[4];

    // Debug output for TCA port 4 (radiator sensor)
    if (getTCAChannel() == 4) {
        Serial.print("Radiator sensor TCA4 - Raw temp: ");
        Serial.print(rawTemperature);
        Serial.print(", Raw hum: ");
        Serial.print(rawHumidity);
        Serial.print(" -> Temp: ");
        Serial.print(-45.0f + 175.0f * (float(rawTemperature) / 65535.0f));
        Serial.println("°C");
    }

    return true;
}

std::map<String, String> SHTsensor::readData()
{
    std::map<String, String> result;
    uint16_t rawTemperature, rawHumidity;
    
    if (readRawData(rawTemperature, rawHumidity))
    {        // Convert raw values to actual temperature and humidity values
        // T = -45 + 175 * (rawTemperature / 65535.0)
        // RH = 100 * (rawHumidity / 65535.0)
        _temperature = -45.0f + 175.0f * (float(rawTemperature) / 65535.0f);
        _humidity = 100.0f * (float(rawHumidity) / 65535.0f);
    }
    else
    {
        Serial.println("SHT readData: Failed to read raw data, using previous values");
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

uint32_t SHTsensor::getSerialNumber()
{
    I2CHandler::selectTCA(getTCAChannel()); 
    if (!writeCommand(0x3780)) // Command to read serial number
    {
        Serial.println("SHT getSerialNumber: writeCommand error");
        return 0;
    }

    delay(500); // Wait for data to be available

    uint8_t data[4];
    if (!readBytes(data, 4))
    {
        Serial.println("SHT getSerialNumber: readBytes error");
        return 0;
    }

    uint32_t serialNumber = 0;
    for (int i = 0; i < 4; i++) {
        serialNumber = (serialNumber << 8) | data[i];
    }

    return serialNumber;
}

uint16_t SHTsensor::readStatus()
{
    I2CHandler::selectTCA(getTCAChannel()); 
    if (!writeCommand(0xF32D)) // Command to read statusregister
    {
        Serial.println("SHT readStatus: writeCommand error");
        return 0xFFFF;
    }

    delay(20); // Wait for data to be available

    uint8_t data[3];
    if (!readBytes(data, 3))
    {
        Serial.println("SHT readStatus: readBytes error");
        return 0xFFFF;
    }

    return (data[0] << 8) | data[1];
}

bool SHTsensor::clearStatus()
{
    I2CHandler::selectTCA(getTCAChannel()); 
    if (!writeCommand(0x3041)) // Command to clear statusregister
    {
        Serial.println("SHT clearStatus: writeCommand error");
        return false;
    }

    delay(20); // Wait for command to complete

    return true;
}

bool SHTsensor::setMeasurementMode(uint16_t mode)
{
    I2CHandler::selectTCA(getTCAChannel()); 
    if (!writeCommand(mode))
    {
        Serial.println("SHT setMeasurementMode: writeCommand error");
        return false;
    }

    delay(20); // Wait for command to complete

    return true;
}

bool SHTsensor::setHeater(bool enable)
{
    I2CHandler::selectTCA(getTCAChannel()); 
    if (!writeCommand(enable ? 0x306D : 0x3066)) // Command to enable/disable heater
    {
        Serial.println("SHT setHeater: writeCommand error");
        return false;
    }

    delay(20); // Wait for command to complete

    return true;
}

bool SHTsensor::writeCommand(uint16_t command)
{
    wire->beginTransmission(_address);
    wire->write(command >> 8); // Send MSB
    wire->write(command & 0xFF); // Send LSB
    return wire->endTransmission() == 0;
}

bool SHTsensor::readBytes(uint8_t *data, uint8_t length)
{
    if (wire->requestFrom(_address, length) != length)
    {
        return false;
    }    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = wire->read();
    }

    return true;
}

float SHTsensor::getTemperature() const {
    return _temperature;
}

float SHTsensor::getHumidity() const {
    return _humidity;
}

// Implementation of pure virtual methods from Device base class
bool SHTsensor::isConnected() {
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(_address);
    return (wire->endTransmission() == 0);
}

void SHTsensor::update() {
    // Always select the correct TCA channel before communicating
    I2CHandler::selectTCA(getTCAChannel());
    
    // Reset readingSuccess flag
    bool readingSuccess = false;
    
    // Try multiple times if needed
    for (int attempt = 0; attempt < 3 && !readingSuccess; attempt++) {
        uint16_t rawTemperature, rawHumidity;        if (readRawData(rawTemperature, rawHumidity)) {
            _temperature = -45 + 175 * (rawTemperature / 65535.0);
            _humidity = 100 * (rawHumidity / 65535.0);
            readingSuccess = true;
        } else {
            Serial.print("SHT reading failed, attempt ");
            Serial.print(attempt + 1);
            Serial.println("/3");
            delay(100);  // Wait before retrying
        }
    }
    
    if (!readingSuccess) {
        Serial.println("SHT reading failed after multiple attempts");
    }
}
