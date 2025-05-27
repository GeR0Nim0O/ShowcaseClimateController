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
    std::map<std::string, float> readData() override; // Return a map of sensor data
    void setTime(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year);
    bool readTime(byte* second, byte* minute, byte* hour, byte* dayOfWeek, byte* dayOfMonth, byte* month, byte* year);
    void displayTime();
    uint8_t getTcaPort() const { return tcaPort; } // Add getter method
    std::map<String, String> getChannels() const { return channels; } // Add getter method for channels
    int getDeviceIndex() const { return deviceIndex; } // Add getDeviceIndex function

private:
    TwoWire* wire;
    uint8_t _address;
    byte decToBcd(byte val);
    byte bcdToDec(byte val);
    std::map<String, String> channels; // Add channels property
    int deviceIndex; // Add deviceIndex property
};

#endif // DS3231RTC_H