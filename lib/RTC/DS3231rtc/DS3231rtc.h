#ifndef DS3231RTC_H
#define DS3231RTC_H

#include "Device.h"
#include <Wire.h>
#include <map>
#include <string>

#define DS3231_ADDRESS 0x68

class DS3231rtc : public Device {
public:
    DS3231rtc(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex)
        : Device(i2cChannel, tcaPort, threshold, channels, deviceIndex), wire(wire) {}
    
    bool begin() override {
        wire->begin();
        return isConnected();
    }
    
    bool isConnected() override;
    void update() override;
    std::map<String, String> readData() override;
    float getThreshold(const String& channelKey) const override { return threshold; }
    
    bool readTime(unsigned char* second, unsigned char* minute, unsigned char* hour,
                 unsigned char* dayOfWeek, unsigned char* dayOfMonth, unsigned char* month,
                 unsigned char* year);
                 
    bool setTime(unsigned char second, unsigned char minute, unsigned char hour,
                unsigned char dayOfWeek, unsigned char dayOfMonth, unsigned char month,
                unsigned char year);
    
private:
    TwoWire* wire;
    unsigned char decToBcd(unsigned char val);
    unsigned char bcdToDec(unsigned char val);
};

#endif // DS3231RTC_H