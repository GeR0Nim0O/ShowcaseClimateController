#include "DS3231rtc.h"

bool DS3231rtc::isConnected() {
    Wire.beginTransmission(DS3231_ADDRESS);
    return (Wire.endTransmission() == 0);
}

void DS3231rtc::update() {
    // Update internal time values from RTC
    unsigned char second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    readTime(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
}

std::map<String, String> DS3231rtc::readData() {
    std::map<String, String> data;
    unsigned char second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    
    if (readTime(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year)) {
        char timeBuffer[20];
        sprintf(timeBuffer, "%02d:%02d:%02d", hour, minute, second);
        data["time"] = String(timeBuffer);
        
        char dateBuffer[20];
        sprintf(dateBuffer, "%02d/%02d/20%02d", dayOfMonth, month, year);
        data["date"] = String(dateBuffer);
        
        data["day"] = String(dayOfWeek);
    }
    
    return data;
}

bool DS3231rtc::readTime(unsigned char* second, unsigned char* minute, unsigned char* hour,
                         unsigned char* dayOfWeek, unsigned char* dayOfMonth, unsigned char* month,
                         unsigned char* year) {
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(0x00); // Start at register 0x00
    if (Wire.endTransmission() != 0) {
        return false;
    }

    if (Wire.requestFrom(DS3231_ADDRESS, 7) != 7) {
        return false;
    }

    // Convert from BCD to decimal
    *second = bcdToDec(Wire.read() & 0x7F);
    *minute = bcdToDec(Wire.read());
    *hour = bcdToDec(Wire.read() & 0x3F); // 24 hour mode
    *dayOfWeek = bcdToDec(Wire.read());
    *dayOfMonth = bcdToDec(Wire.read());
    *month = bcdToDec(Wire.read());
    *year = bcdToDec(Wire.read());
    
    return true;
}

bool DS3231rtc::setTime(unsigned char second, unsigned char minute, unsigned char hour,
                        unsigned char dayOfWeek, unsigned char dayOfMonth, unsigned char month,
                        unsigned char year) {
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(0x00); // Start at register 0x00
    
    // Convert from decimal to BCD
    Wire.write(decToBcd(second));
    Wire.write(decToBcd(minute));
    Wire.write(decToBcd(hour));
    Wire.write(decToBcd(dayOfWeek));
    Wire.write(decToBcd(dayOfMonth));
    Wire.write(decToBcd(month));
    Wire.write(decToBcd(year));
    
    return (Wire.endTransmission() == 0);
}

// Helper functions for BCD conversion
unsigned char DS3231rtc::decToBcd(unsigned char val) {
    return ((val / 10) << 4) + (val % 10);
}

unsigned char DS3231rtc::bcdToDec(unsigned char val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}
