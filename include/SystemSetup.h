#ifndef SYSTEM_SETUP_H
#define SYSTEM_SETUP_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <vector>
#include <map>
#include "Device.h"
#include "DS3231rtc.h"
#include "ClimateController.h"
#include "Display.h"

class SystemSetup {
public:
    // Main setup functions
    static bool initializeSystem();
    static bool setupNetworking(WiFiClientSecure& espClient, PubSubClient& client, 
                                String& clientId, String& topic);
    
    // Individual setup components
    static bool initializeHardware(std::vector<Device*>& devices, 
                                  std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, 
                                  DS3231rtc*& rtc);
    static bool setupConfiguration();
    static bool setupDevices(std::vector<Device*>& devices, 
                           std::map<uint8_t, std::vector<uint8_t>>& tcaScanResults, 
                           DS3231rtc*& rtc);
    static bool setupClimateController(ClimateController*& climateController);
    static bool setupDisplay(Display*& displayDevice);
    
    // Helper functions
    static void printInitialSensorData(const std::vector<Device*>& devices);
    static void printSystemInfo();
    static void validateDevices(const std::vector<Device*>& devices);
    
private:
    SystemSetup() = delete; // Static class
};

#endif // SYSTEM_SETUP_H
