#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_wifi.h>
#include "NetworkDiagnostics.h"

void NetworkDiagnostics::runCompleteDiagnostics() {
    Serial.println("\n=== Network Diagnostics ===");
    
    // Basic WiFi status
    printWiFiStatus();
    
    // DNS configuration
    printDNSConfig();
    
    // Test DNS resolution
    testDNSResolution();
    
    // Test basic connectivity
    testBasicConnectivity();
    
    // Test MQTT host specifically
    testMQTTHostResolution();
    
    Serial.println("=== End Diagnostics ===\n");
}

void NetworkDiagnostics::printWiFiStatus() {
    Serial.println("\n--- WiFi Status ---");
    Serial.print("WiFi Status: ");
    Serial.println(WiFi.status());
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("SSID: ");
        Serial.println(WiFi.SSID());
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Gateway: ");
        Serial.println(WiFi.gatewayIP());
        Serial.print("Subnet Mask: ");
        Serial.println(WiFi.subnetMask());
        Serial.print("DNS Server 1: ");
        Serial.println(WiFi.dnsIP(0));
        Serial.print("DNS Server 2: ");
        Serial.println(WiFi.dnsIP(1));
        Serial.print("Signal Strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("WiFi not connected");
    }
}

void NetworkDiagnostics::printDNSConfig() {
    Serial.println("\n--- DNS Configuration ---");
    Serial.print("Primary DNS: ");
    Serial.println(WiFi.dnsIP(0));
    Serial.print("Secondary DNS: ");
    Serial.println(WiFi.dnsIP(1));
}

void NetworkDiagnostics::testDNSResolution() {
    Serial.println("\n--- DNS Resolution Test ---");
    
    const char* testHosts[] = {
        "google.com",
        "8.8.8.8",
        "mqtt.flespi.io",
        "cloudflare.com"
    };
    
    for (int i = 0; i < 4; i++) {
        Serial.print("Testing DNS resolution for: ");
        Serial.println(testHosts[i]);
        
        IPAddress resolvedIP;
        if (WiFi.hostByName(testHosts[i], resolvedIP)) {
            Serial.print("  Resolved to: ");
            Serial.println(resolvedIP);
        } else {
            Serial.println("  DNS resolution FAILED");
        }
    }
}

void NetworkDiagnostics::testBasicConnectivity() {
    Serial.println("\n--- Basic Connectivity Test ---");
    
    // Test ping to Google DNS
    testPing("8.8.8.8", 53);
    testPing("1.1.1.1", 53);
}

void NetworkDiagnostics::testMQTTHostResolution() {
    Serial.println("\n--- MQTT Host Resolution Test ---");
    
    const char* mqttHost = "mqtt.flespi.io";
    Serial.print("Testing resolution for MQTT host: ");
    Serial.println(mqttHost);
    
    IPAddress mqttIP;
    if (WiFi.hostByName(mqttHost, mqttIP)) {
        Serial.print("MQTT host resolved to: ");
        Serial.println(mqttIP);
        
        // Test connection to resolved IP
        testPing(mqttIP.toString().c_str(), 8883);
    } else {
        Serial.println("MQTT host DNS resolution FAILED");
        Serial.println("Attempting direct IP connection...");
        
        // Try known Flespi IP addresses (these may change)
        testPing("3.125.183.140", 8883);  // Example IP - may not be current
        testPing("18.197.182.101", 8883); // Example IP - may not be current
    }
}

void NetworkDiagnostics::testPing(const char* host, int port) {
    Serial.print("Testing connection to ");
    Serial.print(host);
    Serial.print(":");
    Serial.print(port);
    Serial.print(" ... ");
    
    WiFiClient testClient;
    testClient.setTimeout(5000);
    
    if (testClient.connect(host, port)) {
        Serial.println("SUCCESS");
        testClient.stop();
    } else {
        Serial.println("FAILED");
    }
}

void NetworkDiagnostics::fixDNSSettings() {
    Serial.println("\n--- Applying DNS Fixes ---");
    
    // Disconnect and reconnect with specific DNS servers
    WiFi.disconnect();
    delay(1000);
    
    // Configure static DNS servers
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, IPAddress(8, 8, 8, 8), IPAddress(1, 1, 1, 1));
    
    Serial.println("DNS servers set to Google (8.8.8.8) and Cloudflare (1.1.1.1)");
    Serial.println("Reconnecting to WiFi...");
    
    // Note: WiFi credentials will need to be set again by the calling code
}

void NetworkDiagnostics::testAlternateMQTTServers() {
    Serial.println("\n--- Testing Alternate MQTT Servers ---");
    
    const char* alternateServers[] = {
        "broker.hivemq.com",
        "test.mosquitto.org",
        "broker.emqx.io"
    };
    
    for (int i = 0; i < 3; i++) {
        Serial.print("Testing alternate MQTT server: ");
        Serial.println(alternateServers[i]);
        
        IPAddress serverIP;
        if (WiFi.hostByName(alternateServers[i], serverIP)) {
            Serial.print("  Resolved to: ");
            Serial.println(serverIP);
            testPing(serverIP.toString().c_str(), 1883);
        } else {
            Serial.println("  DNS resolution FAILED");
        }
    }
}
