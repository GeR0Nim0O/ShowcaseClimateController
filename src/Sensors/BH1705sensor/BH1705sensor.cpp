#include "Device.h"
#include "BH1705sensor.h"
#include "I2CHandler.h"

BH1705sensor::BH1705sensor(uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(i2cChannel, tcaPort, threshold, channels, deviceIndex), _address(BH1705_ADDRESS), _lux(NAN) {
    type = "Sensor"; // Fixed type
    this->typeNumber = "BH1705"; // Fixed type number
    Serial.println("BH1705sensor created:");    Serial.print("Address: ");
    Serial.println(_address, HEX);
    Serial.print("Threshold: ");
    Serial.println(threshold);
    Serial.print("Number of Channels: ");
    Serial.println(channels.size());
    Serial.print("Type: ");
    Serial.println(type);
    Serial.print("TypeNumber: ");
    Serial.println(this->typeNumber);
    Serial.print("Device Index: ");
    Serial.println(deviceIndex);
}

bool BH1705sensor::begin() {
    I2CHandler::selectTCA(getTCAChannel()); // Use getTCAChannel() method from Device base class
    if (!writeCommand(0x01)) // Power on command
    {
        Serial.println("BH1705 begin error (power on)");
        return false;
    }
    delay(10); // Wait for power on to complete    // Set measurement mode
    if (!writeCommand(0x10)) // Continuously H-Resolution Mode
    {
        Serial.println("Failed to set measurement mode");
        return false;
    }

    initialized = true; // Set initialized flag to true
    return true;
}

std::map<String, String> BH1705sensor::readData() {
    uint8_t data[2];
    if (!readBytes(data, 2))
    {
        return {{"L", "NaN"}};
    }

    _lux = (data[0] << 8) | data[1];
    _lux /= 1.2; // Convert to lux

    std::map<String, String> dataMap;
    for (const auto& channel : channels) {
        if (channel.second == "L") {
            dataMap[channel.first] = String(_lux);
        }
    }

    return dataMap;
}

bool BH1705sensor::writeCommand(uint8_t command) {
    Wire.beginTransmission(_address);
    Wire.write(command);
    return Wire.endTransmission() == 0;
}

bool BH1705sensor::readBytes(uint8_t *data, uint8_t length) {
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

float BH1705sensor::getLux() const {
    return _lux;
}

// Implementation of pure virtual methods from Device base class
bool BH1705sensor::isConnected() {
    I2CHandler::selectTCA(getTCAChannel());
    wire->beginTransmission(_address);
    return (wire->endTransmission() == 0);
}

void BH1705sensor::update() {
    // Update sensor readings
    readData();
}