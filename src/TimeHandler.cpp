#include "TimeHandler.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include "DS3231rtc/DS3231rtc.h" // Update include path for DS3231rtc
#include <ArduinoJson.h>
#include "I2CHandler.h"
#include "Configuration.h"
#include <NTPClient.h>
#include <WiFiUdp.h>


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // Update interval set to 60 seconds

const unsigned long retryDelay = 1000; // Retry delay in milliseconds
const int maxRetryCount = 3; // Maximum number of retries
const unsigned long responseTimeout = 10000; // Response timeout in milliseconds

bool initialConnection = true; // Flag to track initial connection
bool failureMessagePrinted = false; // Flag to track if failure message has been printed

// Get the current formatted time from the timeapi.io API
String TimeHandler::getFormattedTime(DS3231rtc& rtc) {
    String time = fetchTime(rtc);
    if (time == "Time API Failed" || time == "WiFi not connected" || time == "RTC Read Error") {
        return getTimeFromRTC(rtc); // Return RTC time if API fails
    }
    return time;
}

// Fetch the current time and timezone using timeapi.io or based on timezone from configuration
String TimeHandler::fetchTime(DS3231rtc& rtc) {
    if (initialConnection) {
        Serial.println("Starting to connect to time API using IP...");
        String time = fetchTimeFromAPI("https://timeapi.io/api/Time/current/ip");
        if (time != "Time API Failed") {
            setRTCFromAPI(rtc, time); // Set RTC time from API if successful
            Serial.println("RTC updated using time API.");
            initialConnection = false; // Set flag to false after first connection
            return time;
        }

        String timezone = Configuration::getTimezone();
        String url = "https://timeapi.io/api/Time/current/zone?timeZone=" + timezone;
        Serial.println("Starting to connect to timezone API...");
        time = fetchTimeFromAPI(url);
        if (time != "Time API Failed") {
            setRTCFromAPI(rtc, time); // Set RTC time from API if successful
            Serial.println("RTC updated using timezone API.");
            initialConnection = false; // Set flag to false after first connection
            return time;
        }

        Serial.println("Time API failed, using NTP time.");
        time = fetchTimeFromNTP();
        if (time != "NTP Failed") {
            setRTCFromAPI(rtc, time); // Set RTC time from NTP if successful
            Serial.println("RTC updated using NTP.");
            initialConnection = false; // Set flag to false after first connection
            return time;
        }

        if (!failureMessagePrinted) {
            Serial.println("Time API and NTP failed or already connected, using RTC time.");
            failureMessagePrinted = true; // Set flag to true after printing the message
        }
    }
    return getTimeFromRTC(rtc); // Return RTC time if all other methods fail or already connected
}

// Fetch the current time from the given API URL
String TimeHandler::fetchTimeFromAPI(const String& url) {
    HTTPClient http;
    http.setTimeout(10000); // Set timeout to 10 seconds
    int retryCount = maxRetryCount; // Number of retries

    while (retryCount > 0) {
        Serial.println("Attempting to connect to: " + url);
        if (!http.begin(url)) {
            Serial.println("Failed to connect to " + url);
            return "Time API Failed";
        }
        unsigned long startTime = millis();
        while (millis() - startTime < responseTimeout) {
            int httpCode = http.GET();
            if (httpCode > 0) {
                String payload = http.getString();
                http.end(); // Ensure proper cleanup
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, payload);
                if (error || doc["dateTime"].isNull()) {
                    Serial.print("deserializeJson() failed or dateTime is null: ");
                    Serial.println(error.c_str());
                    return "Time API Failed";
                }
                String datetime = doc["dateTime"].as<String>();
                Serial.println("Fetched time from API: " + datetime);
                return datetime;
            } else {
                Serial.print("HTTP GET failed, error: ");
                Serial.println(httpCode);
                http.end(); // Ensure proper cleanup
                retryCount--;
                Serial.println("Retrying... (" + String(maxRetryCount - retryCount) + "/" + String(maxRetryCount) + ")");
                delay(retryDelay); // Wait for retry delay before retrying
            }
        }
    }
    return "Time API Failed";
}

// Fetch the current time from the NTP server
String TimeHandler::fetchTimeFromNTP() {
    int retryCount = maxRetryCount; // Number of retries

    while (retryCount > 0) {
        Serial.println("Attempting to connect to NTP server...");
        timeClient.begin();
        if (timeClient.update()) {
            unsigned long epochTime = timeClient.getEpochTime();
            struct tm *ptm = gmtime((time_t *)&epochTime);
            char buffer[20];
            sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02d", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
            Serial.println("Fetched time from NTP: " + String(buffer));
            return String(buffer);
        } else {
            Serial.println("Failed to get time from NTP server");
            retryCount--;
            Serial.println("Retrying... (" + String(maxRetryCount - retryCount) + "/" + String(maxRetryCount) + ")");
            delay(retryDelay); // Wait for retry delay before retrying
        }
    }
    return "NTP Failed";
}

// Update RTC from NTP server
void TimeHandler::updateRTCFromNTP(DS3231rtc& rtc) {
    String datetime = fetchTimeFromNTP();
    if (datetime != "NTP Failed") {
        setRTCFromAPI(rtc, datetime);
        Serial.println("RTC updated using NTP.");
    }
}

void TimeHandler::setRTCFromAPI(DS3231rtc& rtc, String datetime) {
    I2CHandler::selectTCA(rtc.getTCAChannel());
    // Parse datetime string and set RTC
    int year = datetime.substring(0, 4).toInt();
    int month = datetime.substring(5, 7).toInt();
    int day = datetime.substring(8, 10).toInt();
    int hour = datetime.substring(11, 13).toInt();
    int minute = datetime.substring(14, 16).toInt();
    int second = datetime.substring(17, 19).toInt();
    rtc.setTime(second, minute, hour, 0, day, month, year - 2000);
}

String TimeHandler::getTimeFromRTC(DS3231rtc& rtc) {
    I2CHandler::selectTCA(rtc.getTcaPort());
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    if (!rtc.readTime(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year)) {
        Serial.println("Failed to read from RTC");
        return "RTC Read Error"; // Return error message if read fails
    }
    char buffer[20];
    sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02d", year + 2000, month, dayOfMonth, hour, minute, second);
    return String(buffer);
}

String TimeHandler::getCurrentTime(DS3231rtc& rtc) {
  String currentTime = getFormattedTime(rtc);
  if (currentTime == "RTC Read Error") {
    Serial.println("RTC Read Error, using last known time.");
    currentTime = getTimeFromRTC(rtc); // Use last known time if RTC read fails
  }
  return currentTime;
}

void TimeHandler::fetchCurrentTimePeriodically(DS3231rtc* rtc, unsigned long& lastTimeFetch, const unsigned long timeFetchInterval) {
    if (millis() - lastTimeFetch > timeFetchInterval) {
        String currentTime = getCurrentTime(*rtc);
        Serial.println("Updated Time: " + currentTime);
        lastTimeFetch = millis();
    }
}
