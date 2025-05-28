#include "Device.h"
#include "SCALESsensor.h"
#include "I2CHandler.h"

SCALESsensor::SCALESsensor(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, threshold, channels, i2cChannel, tcaPort, deviceIndex), _address(SCALES_ADDRESS), _weight(NAN) {
}

bool SCALESsensor::begin() {
    I2CHandler::selectTCA(getTCAChannel()); // Use getTCAChannel() method from Device base class
    if (!writeCommand(0x01)) // Power on command
    {
        Serial.println("SCALES begin error (power on)");
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

std::map<String, String> SCALESsensor::readData() {
    uint8_t dataArr[2];
    if (!readBytes(dataArr, 2))
    {
        return {{"W", "NaN"}};
    }

    _weight = (dataArr[0] << 8) | dataArr[1];
    _weight /= 1.2; // Convert to weight
    
    std::map<String, String> data;
    for (const auto& channel : channels) {
        if (channel.second == "W") {
            data[channel.first] = String(_weight);
        }
    }

    return data;
}

bool SCALESsensor::writeCommand(uint8_t command) {
    wire->beginTransmission(_address);
    wire->write(command);
    return wire->endTransmission() == 0;
}

bool SCALESsensor::readBytes(uint8_t *data, uint8_t length) {
    if (wire->requestFrom(_address, length) != length)
    {
        return false;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = Wire.read();
    }

    return true;
}

float SCALESsensor::getWeight() const {
    return _weight;
}

// Implementation of pure virtual methods from Device base class
bool SCALESsensor::isConnected() {
    I2CHandler::selectTCA(getTCAChannel());
    Wire.beginTransmission(_address);
    return (Wire.endTransmission() == 0);
}

void SCALESsensor::update() {
    // Update sensor readings
    readData();
}
