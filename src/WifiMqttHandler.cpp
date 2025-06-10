#include "WifiMqttHandler.h"
#include "CACert.h"

void WifiMqttHandler::connectToWiFi(const char* ssid, const char* password) {
    if (ssid == nullptr || password == nullptr) {
        Serial.println("Error: SSID or password is null");
        return;
    }
    
    // Scan for available networks first
    int networkCount = WiFi.scanNetworks();
    
    if (networkCount < 0) {
        Serial.println("Network scan failed, retrying...");
        delay(2000);
        networkCount = WiFi.scanNetworks(false, false);
    }
    
    bool targetNetworkFound = false;
    for (int i = 0; i < networkCount; i++) {
        if (WiFi.SSID(i).equals(ssid)) {
            targetNetworkFound = true;
            break;
        }
    }
    
    if (!targetNetworkFound && networkCount > 0) {
        Serial.println("WARNING: Target network not found in scan");
    }
    
    // Clean up any existing connection
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    delay(1000);
    
    int attempts = 0;
    const int maxAttempts = 5;
    
    while (attempts < maxAttempts && WiFi.status() != WL_CONNECTED) {
        attempts++;
        Serial.print("WiFi attempt ");
        Serial.print(attempts);
        Serial.print("/");
        Serial.println(maxAttempts);
        
        WiFi.begin(ssid, password);
        
        unsigned long startAttemptTime = millis();
        const unsigned long attemptTimeout = 15000; // 15 seconds timeout
        
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < attemptTimeout) {
                        delay(500);
            Serial.print(".");
            
            // Check for specific error conditions
            if (WiFi.status() == WL_CONNECT_FAILED) {
                Serial.println(" Connection failed - wrong password?");
                break;
            } else if (WiFi.status() == WL_NO_SSID_AVAIL) {
                Serial.println(" SSID not available");
                break;
            }
        }
        Serial.println();
        
        if (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
            WiFi.disconnect();
            delay(3000);
        }
    }
    
    // Final connection status
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected to " + WiFi.SSID());
        Serial.println("IP: " + WiFi.localIP().toString());
    } else {
        Serial.println("WiFi connection failed after " + String(maxAttempts) + " attempts");
    }
}

bool WifiMqttHandler::connectToWiFiWithCheck(const String& ssid, const String& password) {
    if (ssid.isEmpty() || password.isEmpty()) {
        Serial.println("Error: SSID or password is empty");
        return false;
    }    connectToWiFi(ssid.c_str(), password.c_str());
    return (WiFi.status() == WL_CONNECTED);
}

void WifiMqttHandler::setupSecureClient(WiFiClientSecure &espClient, const char* rootCACertificate) {
    Serial.println("Setting up secure client with certificate...");
    
    // Set the certificate
    espClient.setCACert(rootCACertificate);
    
    // Set connection timeout
    espClient.setTimeout(15000);
    
    // Optional: For development only - set to false in production
    // espClient.setInsecure(); 
    
    Serial.println("Client configured with certificate");
}

void WifiMqttHandler::connectToMqttBroker(PubSubClient &client, WiFiClientSecure &espClient, const char* mqtt_server, const char* rootCACertificate, int mqtt_port, const char* clientId, const char* topic, const char* token) {
    // Check WiFi status before attempting MQTT connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("MQTT connection aborted: WiFi not connected");
        Serial.print("WiFi status: ");
        Serial.println(WiFi.status());
        return;
    }

    // Verify we have a valid IP address
    if (WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
        Serial.println("MQTT connection aborted: No valid IP address assigned");
        return;
    }

    Serial.println("MQTT parameters:");
    if (mqtt_server) {
        Serial.print("mqtt_server: ");
        Serial.println(mqtt_server);
    } else {
        Serial.println("mqtt_server: null");
    }
    if (clientId) {
        Serial.print("clientId: ");
        Serial.println(clientId);
    } else {
        Serial.println("clientId: null");
    }
    if (topic) {
        Serial.print("topic: ");
        Serial.println(topic);
    } else {
        Serial.println("topic: null");
    }    if (mqtt_server == nullptr || clientId == nullptr || topic == nullptr) {
        Serial.println("Error: MQTT parameters are null");
        return;
    }

    // Additional WiFi connectivity check
    Serial.print("WiFi status: Connected to ");
    Serial.print(WiFi.SSID());
    Serial.print(" (IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(", Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm)");

    Serial.println("Setting up SSL/TLS connection...");
    setupSecureClient(espClient, rootCACertificate);

    client.setServer(mqtt_server, mqtt_port);

    // Print SSL/TLS settings for debugging
    Serial.println("SSL/TLS settings:");
    Serial.print("MQTT Server: ");
    Serial.println(mqtt_server);
    Serial.print("MQTT Port: ");
    Serial.println(mqtt_port);
    Serial.print("Client ID: ");
    Serial.println(clientId);
    Serial.print("Topic: ");
    Serial.println(topic);

    int attempts = 0;
    const int maxAttempts = 5;
    
    while (!client.connected() && attempts < maxAttempts) {
        attempts++;
        Serial.print("MQTT connection attempt ");
        Serial.print(attempts);
        Serial.print("/");
        Serial.print(maxAttempts);
        Serial.print("...");
        
        // Connect with flespi token if provided
        bool success = false;
        if (token && strlen(token) > 0) {
            // Use token for authentication (flespi style)
            success = client.connect(clientId, token, "");
            if (success) {
                Serial.println("connected with flespi token");
            }
        } else {
            // Fallback to default secure connection
            success = client.connect(clientId, nullptr, nullptr, nullptr, 0, true, nullptr, true);
            if (success) {
                Serial.println("connected with secure TLS");
            }
        }
        
        if (success) {
            client.subscribe(topic);
            // Publish connection message
            String connMsg = String("Device ") + clientId + " connected securely";
            client.publish((String(topic) + "/status").c_str(), connMsg.c_str(), true);
            break; // Exit loop on successful connection
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            if (attempts < maxAttempts) {
                Serial.println(" - retrying in 5 seconds");
                delay(5000); // Delay for retry
            } else {
                Serial.println(" - max attempts reached");
            }
        }
    }
    
    if (client.connected()) {
        Serial.print("MQTT connection status: ");
        Serial.println(client.state());
        } else {
        Serial.print("Failed to connect to MQTT broker after ");
        Serial.print(maxAttempts);
        Serial.println(" attempts. Skipping MQTT connection.");
    }
}

bool WifiMqttHandler::connectToMqttBrokerWithCheck(PubSubClient &client, WiFiClientSecure &espClient, const String& mqtt_server, const char* rootCACertificate, int mqtt_port, const String& clientId, const String& topic, const String& token) {
    // Ensure MQTT parameters are not null
    if (mqtt_server.isEmpty() || clientId.isEmpty() || topic.isEmpty()) {
        Serial.println("Error: One or more MQTT parameters are empty");
        return false;
    }

    // Connect to MQTT broker
    connectToMqttBroker(client, espClient, mqtt_server.c_str(), rootCACertificate, mqtt_port, clientId.c_str(), topic.c_str(), token.isEmpty() ? nullptr : token.c_str());

    if (!client.connected()) {
        Serial.println("Failed to connect to MQTT broker");
        return false;
    }

    return true;
}

void WifiMqttHandler::keepAlive(PubSubClient &client, WiFiClientSecure &espClient, const char* ssid, const char* password, const char* mqtt_server, const char* rootCACertificate, int mqtt_port, const char* clientId, const char* topic) {
    static unsigned long lastWiFiReconnectAttempt = 0;
    static unsigned long lastMqttReconnectAttempt = 0;
    static int wifiReconnectAttempts = 0;
    static int mqttReconnectAttempts = 0;
    static bool wifiSkipped = false;
    static bool mqttSkipped = false;
    const int maxReconnectAttempts = 3;
    const unsigned long WIFI_RETRY_INTERVAL = 60000; // 60 seconds

    // WiFi reconnection
    if (WiFi.status() != WL_CONNECTED && !wifiSkipped) {
        if (millis() - lastWiFiReconnectAttempt > WIFI_RETRY_INTERVAL) {
            wifiReconnectAttempts++;
            
            if (wifiReconnectAttempts <= maxReconnectAttempts) {
                Serial.print("WiFi reconnect attempt ");
                Serial.print(wifiReconnectAttempts);
                Serial.print("/");
                Serial.println(maxReconnectAttempts);
                
                WiFi.disconnect();
                delay(1000);
                WiFi.begin(ssid, password);
                lastWiFiReconnectAttempt = millis();
                mqttReconnectAttempts = 0;
                mqttSkipped = false;            } else {
                Serial.println("Max WiFi attempts reached");
                wifiSkipped = true;
                static unsigned long wifiResetTime = 0;
                if (wifiResetTime == 0) {
                    wifiResetTime = millis(); // Set reset time only when first reaching max attempts
                }
                if (millis() - wifiResetTime > 300000) { // Reset after 5 minutes
                    wifiReconnectAttempts = 0;
                    wifiSkipped = false;
                    wifiResetTime = 0; // Reset the timer
                }
            }
        }
        return;
    }

    // Reset WiFi reconnection attempts when WiFi reconnects successfully
    if (WiFi.status() == WL_CONNECTED && wifiSkipped) {
        wifiReconnectAttempts = 0;
        wifiSkipped = false;
        mqttReconnectAttempts = 0;
        mqttSkipped = false;
    }

    // MQTT reconnection
    if (!client.connected() && !mqttSkipped && !wifiSkipped) {
        if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
            return;
        }

        if (millis() - lastMqttReconnectAttempt > 5000) {
            mqttReconnectAttempts++;
            
            if (mqttReconnectAttempts <= maxReconnectAttempts) {
                Serial.print("MQTT reconnect attempt ");
                Serial.print(mqttReconnectAttempts);
                Serial.print("/");
                Serial.println(maxReconnectAttempts);
                
                connectToMqttBroker(client, espClient, mqtt_server, rootCACertificate, mqtt_port, clientId, topic);
                lastMqttReconnectAttempt = millis();
            } else {
                Serial.println("Max MQTT attempts reached");
                mqttSkipped = true;
            }
        }
        return;
    }

    // Reset MQTT reconnection attempts when MQTT reconnects successfully
    if (client.connected() && mqttSkipped) {
        mqttReconnectAttempts = 0;
        mqttSkipped = false;
    }

    // Maintain MQTT connection
    if (client.connected()) {
        client.loop();
    }
}

void WifiMqttHandler::printConnectionStatus(PubSubClient &client) {
    Serial.print("WiFi: ");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected (");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm)");
    } else {
        Serial.println("Disconnected");
    }
    
    Serial.print("MQTT: ");
    if (client.connected()) {
        Serial.println("Connected");
    } else {
        Serial.print("Disconnected (");
        Serial.print(client.state());
        Serial.println(")");
    }
}

// WiFi scanning and debugging functions
void WifiMqttHandler::scanAndDisplayNetworks() {
    int networkCount = WiFi.scanNetworks();
    
    if (networkCount <= 0) {
        Serial.println("No networks found");
        return;
    }
    
    Serial.print("Found ");
    Serial.print(networkCount);
    Serial.println(" networks");
}

bool WifiMqttHandler::isNetworkAvailable(const String& ssid) {
    int networkCount = WiFi.scanNetworks();
    
    for (int i = 0; i < networkCount; i++) {
        if (WiFi.SSID(i).equals(ssid)) {
            return true;
        }
    }
    return false;
}

// Enhanced WiFi scanning and analysis functions
void WifiMqttHandler::scanAndAnalyzeNetworks() {
    WiFi.mode(WIFI_STA);
    delay(1000);
    
    int networkCount = WiFi.scanNetworks(false, true);
    
    if (networkCount < 0) {
        Serial.println("Network scan failed");
        return;
    }
    
    if (networkCount == 0) {
        Serial.println("No networks found");
        return;
    }
    
    Serial.print("Found ");
    Serial.print(networkCount);
    Serial.println(" networks");
}

void WifiMqttHandler::displayNetworkDetails(int networkIndex) {
    String ssid = WiFi.SSID(networkIndex);
    int32_t rssi = WiFi.RSSI(networkIndex);
    wifi_auth_mode_t encType = WiFi.encryptionType(networkIndex);
    uint8_t* bssid = WiFi.BSSID(networkIndex);
    int32_t channel = WiFi.channel(networkIndex);
    
    // Create visual signal strength indicator
    String signalBars = getSignalQuality(rssi);
    
    Serial.println("┌────────────────────────────────────────────────────────────┐");
    Serial.printf("│ Network: %-47s│\n", ssid.c_str());
    Serial.printf("│ Signal:  %s (%d dBm)                        │\n", signalBars.c_str(), rssi);
    Serial.printf("│ Channel: %-47d│\n", channel);
    
    // Security information
    String security;
    String recommendation;
    switch(encType) {
        case WIFI_AUTH_OPEN: 
            security = "OPEN"; 
            recommendation = "⚠️  UNSECURED - Avoid for sensitive data";
            break;
        case WIFI_AUTH_WEP: 
            security = "WEP"; 
            recommendation = "⚠️  WEAK - Consider upgrading to WPA2/3";
            break;
        case WIFI_AUTH_WPA_PSK: 
            security = "WPA-PSK"; 
            recommendation = "⚠️  OUTDATED - Upgrade to WPA2/3 recommended";
            break;
        case WIFI_AUTH_WPA2_PSK: 
            security = "WPA2-PSK"; 
            recommendation = "✅ GOOD - Secure for most uses";
            break;
        case WIFI_AUTH_WPA_WPA2_PSK: 
            security = "WPA/WPA2-PSK"; 
            recommendation = "✅ GOOD - Backward compatible";
            break;
        case WIFI_AUTH_WPA2_ENTERPRISE: 
            security = "WPA2-Enterprise"; 
            recommendation = "🏢 ENTERPRISE - May require special setup";
            break;
        case WIFI_AUTH_WPA3_PSK: 
            security = "WPA3-PSK"; 
            recommendation = "✅ EXCELLENT - Latest security standard";
            break;
        case WIFI_AUTH_WPA2_WPA3_PSK: 
            security = "WPA2/WPA3-PSK"; 
            recommendation = "✅ EXCELLENT - Best compatibility + security";
            break;
        default: 
            security = "UNKNOWN"; 
            recommendation = "❓ UNKNOWN - Check router settings";
            break;
    }
    
    Serial.printf("│ Security: %-44s│\n", security.c_str());
    Serial.printf("│ %s│\n", recommendation.c_str());
    
    // BSSID (MAC address)
    Serial.printf("│ BSSID: %02X:%02X:%02X:%02X:%02X:%02X                           │\n", 
                  bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
    
    Serial.println("└────────────────────────────────────────────────────────────┘");
}

String WifiMqttHandler::getSignalQuality(int rssi) {
    if (rssi >= -50) {
        return "████████ Excellent";
    } else if (rssi >= -60) {
        return "██████   Good     ";
    } else if (rssi >= -70) {
        return "████     Fair     ";
    } else if (rssi >= -80) {
        return "██       Poor     ";
    } else {
        return "▌        Very Poor";
    }
}

String WifiMqttHandler::getSecurityRecommendation(wifi_auth_mode_t encType) {
    switch(encType) {
        case WIFI_AUTH_OPEN: return "⚠️  UNSECURED";
        case WIFI_AUTH_WEP: return "⚠️  WEAK";
        case WIFI_AUTH_WPA_PSK: return "⚠️  OUTDATED";
        case WIFI_AUTH_WPA2_PSK: return "✅ GOOD";
        case WIFI_AUTH_WPA_WPA2_PSK: return "✅ GOOD";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "🏢 ENTERPRISE";
        case WIFI_AUTH_WPA3_PSK: return "✅ EXCELLENT";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "✅ EXCELLENT";
        default: return "❓ UNKNOWN";
    }
}