#ifndef TIMEHANDLER_H
#define TIMEHANDLER_H

#include <Arduino.h>
#include <ArduinoJson.h> // Include ArduinoJson for JSON handling
#include "DS3231rtc.h"

class TimeHandler {
public:
    static String getFormattedTime(DS3231rtc& rtc); // Get formatted time
    static void setRTCFromAPI(DS3231rtc& rtc, String datetime); // Set RTC time from API
    static String getTimeFromRTC(DS3231rtc& rtc); // Get time from RTC
    static void updateRTCFromNTP(DS3231rtc& rtc); // Update RTC from NTP server
    static String getCurrentTime(DS3231rtc& rtc); // Get current time    // New function to fetch current time periodically
    static void fetchCurrentTimePeriodically(DS3231rtc* rtc, unsigned long& lastTimeFetch, const unsigned long timeFetchInterval);
    static String fetchTime(DS3231rtc& rtc); // Fetch time from API or RTC
    
    // Status reporting
    static void printTimeStatus(DS3231rtc* rtc);

private:
    static String fetchTimeFromAPI(const String& url); // Fetch time from the given API URL
    static String fetchTimeFromNTP(); // Fetch time from NTP server
};

#endif // TIMEHANDLER_H