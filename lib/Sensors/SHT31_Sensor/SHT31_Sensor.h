#ifndef SHT31_SENSOR_H
#define SHT31_SENSOR_H

#include "Device.h"

#define SHT31_DEFAULT_ADDRESS 0x44

class SHT31_Sensor : public Device {
public:
    SHT31_Sensor(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    
    // Sensor reading methods
    float getTemperature() const { return temperature; }
    float getHumidity() const { return humidity; }
    bool readSensor();
    
    // Advanced features
    bool enableHeater(bool enable);
    bool softReset();
    uint16_t getStatus();

private:
    float temperature;
    float humidity;
    bool heaterEnabled;
    
    bool writeCommand(uint16_t command);
    bool readData(uint8_t* data, uint8_t length);
    float calculateTemperature(uint16_t rawTemp);
    float calculateHumidity(uint16_t rawHum);
    bool checkCRC(uint8_t data[], uint8_t checksum);
};

#endif // SHT31_SENSOR_H
