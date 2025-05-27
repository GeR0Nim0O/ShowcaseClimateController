#include "DS3231rtc.h"
#include <Arduino.h> // Include Arduino header for byte type and Serial object

DS3231rtc::DS3231rtc(TwoWire* wire, uint8_t i2cChannel, uint8_t tcaPort, float threshold, std::map<String, String> channels, int deviceIndex)
    : Device(wire, i2cChannel, tcaPort, threshold, channels, deviceIndex), wire(wire), _address(DS3231_ADDRESS) {
    type = "RTC"; // Fixed type
    typeNumber = "DS3231"; // Fixed type number
    Serial.println("DS3231rtc created:");
    Serial.print("Address: ");
    Serial.println(_address, HEX);
    Serial.print("Threshold: ");
    Serial.println(threshold);
    Serial.print("Number of Channels: ");
    Serial.println(channels.size());
    Serial.print("Type: ");
    Serial.println(type);
    Serial.print("TypeNumber: ");
    Serial.println(typeNumber);
    Serial.print("Device Index: ");
    Serial.println(deviceIndex);
}

bool DS3231rtc::begin() {
    wire->beginTransmission(_address);
    int error = wire->endTransmission();
    if (error != 0)
    {
        return false;
    }
    return true;
}

std::map<String, String> DS3231rtc::readData() {
    // This function is not applicable for RTC, so return an empty map
    return {};
}

void DS3231rtc::setTime(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year) {
    wire->beginTransmission(_address);
    wire->write(0x00); // set DS3231 register pointer to 00h
    wire->write(decToBcd(second));
    wire->write(decToBcd(minute));
    wire->write(decToBcd(hour));
    wire->write(decToBcd(dayOfWeek));
    wire->write(decToBcd(dayOfMonth));
    wire->write(decToBcd(month));
    wire->write(decToBcd(year));
    wire->endTransmission();
}

bool DS3231rtc::readTime(byte* second, byte* minute, byte* hour, byte* dayOfWeek, byte* dayOfMonth, byte* month, byte* year) {
    wire->beginTransmission(_address);
    wire->write(0x00); // set DS3231 register pointer to 00h
    int endTransmissionResult = wire->endTransmission();
    if (endTransmissionResult != 0) {
        Serial.print("Failed to end transmission, error: ");
        Serial.println(endTransmissionResult);
        return false; // Return false if transmission fails
    }
    int bytesRequested = wire->requestFrom((uint8_t)_address, (uint8_t)7);
    if (bytesRequested != 7) {
        Serial.print("Failed to request data from RTC, bytes requested: ");
        Serial.println(bytesRequested);
        return false; // Return false if request fails
    }
    *second = bcdToDec(wire->read() & 0x7f);
    *minute = bcdToDec(wire->read());
    *hour = bcdToDec(wire->read() & 0x3f);
    *dayOfWeek = bcdToDec(wire->read());
    *dayOfMonth = bcdToDec(wire->read());
    *month = bcdToDec(wire->read());
    *year = bcdToDec(wire->read());
    return true; // Return true if read is successful
}

void DS3231rtc::displayTime() {
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    readTime(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
    Serial.print(hour, DEC);
    Serial.print(":");
    Serial.print(minute, DEC);
    Serial.print(":");
    Serial.print(second, DEC);
    Serial.print(" ");
    Serial.print(dayOfMonth, DEC);
    Serial.print("/");
    Serial.print(month, DEC);
    Serial.print("/");
    Serial.print(year, DEC);
    Serial.print(" Day of week: ");
    Serial.println(dayOfWeek, DEC);
}

byte DS3231rtc::decToBcd(byte val) {
    return ( (val/10*16) + (val%10) );
}

byte DS3231rtc::bcdToDec(byte val) {
    return ( (val/16*10) + (val%16) );
}

// Implementation of pure virtual methods from Device base class
bool DS3231rtc::isConnected() {
    wire->beginTransmission(_address);
    return (wire->endTransmission() == 0);
}

void DS3231rtc::update() {
    // RTC doesn't need regular updates, but this method is required by the base class
    // Could potentially sync with NTP here if needed
}