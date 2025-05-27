#include "DS3231rtc.h"

bool DS3231rtc::isConnected() {
    wire->beginTransmission(DS3231_ADDRESS);
    return (wire->endTransmission() == 0);
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
    wire->beginTransmission(DS3231_ADDRESS);
    wire->write(0x00); // Start at register 0x00
    if (wire->endTransmission() != 0) {
        return false;
    }

    if (wire->requestFrom(DS3231_ADDRESS, 7) != 7) {
        return false;
    }

    // Convert from BCD to decimal
    *second = bcdToDec(wire->read() & 0x7F);
    *minute = bcdToDec(wire->read());
    *hour = bcdToDec(wire->read() & 0x3F); // 24 hour mode
    *dayOfWeek = bcdToDec(wire->read());
    *dayOfMonth = bcdToDec(wire->read());
    *month = bcdToDec(wire->read());
    *year = bcdToDec(wire->read());
    
    return true;
}

bool DS3231rtc::setTime(unsigned char second, unsigned char minute, unsigned char hour,
                        unsigned char dayOfWeek, unsigned char dayOfMonth, unsigned char month,
                        unsigned char year) {
    wire->beginTransmission(DS3231_ADDRESS);
    wire->write(0x00); // Start at register 0x00
    
    // Convert from decimal to BCD
    wire->write(decToBcd(second));
    wire->write(decToBcd(minute));
    wire->write(decToBcd(hour));
    wire->write(decToBcd(dayOfWeek));
    wire->write(decToBcd(dayOfMonth));
    wire->write(decToBcd(month));
    wire->write(decToBcd(year));
    
    return (wire->endTransmission() == 0);
}

// Helper functions for BCD conversion
unsigned char DS3231rtc::decToBcd(unsigned char val) {
    return ((val / 10) << 4) + (val % 10);
}

unsigned char DS3231rtc::bcdToDec(unsigned char val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}
