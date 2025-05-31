#ifndef NETWORKDIAGNOSTICS_H
#define NETWORKDIAGNOSTICS_H

#include <Arduino.h>

class NetworkDiagnostics {
public:
    // Main diagnostic function
    static void runCompleteDiagnostics();
    
    // WiFi status and configuration
    static void printWiFiStatus();
    static void printDNSConfig();
    
    // DNS and connectivity testing
    static void testDNSResolution();
    static void testBasicConnectivity();
    static void testMQTTHostResolution();
    static void testPing(const char* host, int port);
    
    // Network repair and alternative testing
    static void fixDNSSettings();
    static void testAlternateMQTTServers();
};

#endif // NETWORKDIAGNOSTICS_H
