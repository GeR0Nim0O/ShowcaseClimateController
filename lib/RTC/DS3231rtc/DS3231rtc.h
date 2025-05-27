#ifndef DS3231RTC_H
#define DS3231RTC_H

#include "Device.h"
#include <Arduino.h> // Include Arduino header for byte type
#include <map>
#include <string>

#define DS3231_ADDRESS 0x68 // Define the I2C address for the DS3231

class DS3231rtc : public Device {
public:
    DS3231rtc(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex);
    bool begin() override;
    bool isConnected() override;
    void update() override;
    std::map<String, String> readData() override; // Return a map of sensor data    
    // Override pure virtual methods from Device
    std::map<String, String> getChannels() const override { return channels; }
    float getThreshold(const String& channelKey = "") const override { return threshold; }
    
    void setTime(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year);
    bool readTime(byte* second, byte* minute, byte* hour, byte* dayOfWeek, byte* dayOfMonth, byte* month, byte* year);
    void displayTime();

private:
    TwoWire* wire;
    uint8_t _address;
    byte decToBcd(byte val);
    byte bcdToDec(byte val);
};

#endif // DS3231RTC_H