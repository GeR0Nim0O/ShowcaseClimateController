#include "SHT31_Sensor.h"

// SHT31 Commands
#define SHT31_MEAS_HIGHREP_STRETCH 0x2C06
#define SHT31_MEAS_MEDREP_STRETCH  0x2C0D
#define SHT31_MEAS_LOWREP_STRETCH  0x2C10
#define SHT31_READSTATUS           0xF32D
#define SHT31_CLEARSTATUS          0x3041
#define SHT31_SOFTRESET            0x30A2
#define SHT31_HEATEREN             0x306D
#define SHT31_HEATERDIS            0x3066

SHT31_Sensor::SHT31_Sensor(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex)
    : Device(i2cAddress, tcaChannel, deviceName, deviceIndex), 
      temperature(0.0), humidity(0.0), heaterEnabled(false) {
    type = "Temperature_Humidity_Sensor";
}

bool SHT31_Sensor::begin() {
    selectTCAChannel(tcaChannel);
    
    if (!testI2CConnection()) {
        Serial.println("SHT31 sensor not found!");
        return false;
    }
    
    // Soft reset the sensor
    if (!softReset()) {
        Serial.println("SHT31 soft reset failed!");
        return false;
    }
    
    delay(10);
    
    initialized = true;
    Serial.println("SHT31 sensor initialized successfully");
    return true;
}

bool SHT31_Sensor::isConnected() {
    return testI2CConnection();
}

void SHT31_Sensor::update() {
    readSensor();
}

bool SHT31_Sensor::readSensor() {
    selectTCAChannel(tcaChannel);
    
    if (!writeCommand(SHT31_MEAS_HIGHREP_STRETCH)) {
        return false;
    }
    
    delay(20); // Wait for measurement
    
    uint8_t data[6];
    if (!readData(data, 6)) {
        return false;
    }
    
    // Check CRC for temperature
    if (!checkCRC(&data[0], data[2])) {
        Serial.println("Temperature CRC check failed");
        return false;
    }
    
    // Check CRC for humidity
    if (!checkCRC(&data[3], data[5])) {
        Serial.println("Humidity CRC check failed");
        return false;
    }
    
    uint16_t rawTemp = (data[0] << 8) | data[1];
    uint16_t rawHum = (data[3] << 8) | data[4];
    
    temperature = calculateTemperature(rawTemp);
    humidity = calculateHumidity(rawHum);
    
    return true;
}

bool SHT31_Sensor::enableHeater(bool enable) {
    selectTCAChannel(tcaChannel);
    
    uint16_t command = enable ? SHT31_HEATEREN : SHT31_HEATERDIS;
    if (writeCommand(command)) {
        heaterEnabled = enable;
        return true;
    }
    return false;
}

bool SHT31_Sensor::softReset() {
    selectTCAChannel(tcaChannel);
    return writeCommand(SHT31_SOFTRESET);
}

uint16_t SHT31_Sensor::getStatus() {
    selectTCAChannel(tcaChannel);
    
    if (!writeCommand(SHT31_READSTATUS)) {
        return 0xFFFF;
    }
    
    uint8_t data[3];
    if (!readData(data, 3)) {
        return 0xFFFF;
    }
    
    return (data[0] << 8) | data[1];
}

bool SHT31_Sensor::writeCommand(uint16_t command) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(command >> 8);   // MSB
    Wire.write(command & 0xFF); // LSB
    return (Wire.endTransmission() == 0);
}

bool SHT31_Sensor::readData(uint8_t* data, uint8_t length) {
    Wire.requestFrom(i2cAddress, length);
    
    uint8_t i = 0;
    while (Wire.available() && i < length) {
        data[i++] = Wire.read();
    }
    
    return (i == length);
}

float SHT31_Sensor::calculateTemperature(uint16_t rawTemp) {
    return 175.0 * (float)rawTemp / 65535.0 - 45.0;
}

float SHT31_Sensor::calculateHumidity(uint16_t rawHum) {
    return 100.0 * (float)rawHum / 65535.0;
}

bool SHT31_Sensor::checkCRC(uint8_t data[], uint8_t checksum) {
    uint8_t crc = 0xFF;
    
    for (int i = 0; i < 2; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return (crc == checksum);
}
