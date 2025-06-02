#include "WifiMqttHandler.h"
#include "CACert.h"

void WifiMqttHandler::connectToWiFi(const char* ssid, const char* password) {
    if (ssid == nullptr || password == nullptr) {
        Serial.println("Error: SSID or password is null");
        return;
    }
    
    Serial.println("=== WiFi Connection Debug ===");
    Serial.print("Target SSID: ");
    Serial.println(ssid);
    Serial.print("Password length: ");
    Serial.println(strlen(password));
    
    // Scan for available networks first
    Serial.println("Scanning for available WiFi networks...");
    int networkCount = WiFi.scanNetworks();
    Serial.print("Found ");
    Serial.print(networkCount);
    Serial.println(" networks:");
    
    bool targetNetworkFound = false;
    for (int i = 0; i < networkCount; i++) {
        String networkSSID = WiFi.SSID(i);
        int32_t networkRSSI = WiFi.RSSI(i);
        wifi_auth_mode_t encryptionType = WiFi.encryptionType(i);
        
        Serial.print("  ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(networkSSID);
        Serial.print(" (");
        Serial.print(networkRSSI);
        Serial.print(" dBm) ");
        
        switch(encryptionType) {
            case WIFI_AUTH_OPEN: Serial.print("[OPEN]"); break;
            case WIFI_AUTH_WEP: Serial.print("[WEP]"); break;
            case WIFI_AUTH_WPA_PSK: Serial.print("[WPA_PSK]"); break;
            case WIFI_AUTH_WPA2_PSK: Serial.print("[WPA2_PSK]"); break;
            case WIFI_AUTH_WPA_WPA2_PSK: Serial.print("[WPA_WPA2_PSK]"); break;
            case WIFI_AUTH_WPA2_ENTERPRISE: Serial.print("[WPA2_ENTERPRISE]"); break;
            case WIFI_AUTH_WPA3_PSK: Serial.print("[WPA3_PSK]"); break;
            case WIFI_AUTH_WPA2_WPA3_PSK: Serial.print("[WPA2_WPA3_PSK]"); break;
            default: Serial.print("[UNKNOWN]"); break;
        }
        
        if (networkSSID.equals(ssid)) {
            targetNetworkFound = true;
            Serial.print(" *** TARGET NETWORK ***");
        }
        Serial.println();
    }
    
    if (!targetNetworkFound) {
        Serial.println("WARNING: Target network not found in scan!");
        Serial.println("Please check:");
        Serial.println("1. Network name (SSID) is correct");
        Serial.println("2. Router is powered on and broadcasting");
        Serial.println("3. ESP32 is within range of the router");
    }
    
    Serial.println("============================");
    
    // Clean up any existing connection
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    delay(1000);
    
    int attempts = 0;
    const int maxAttempts = 5;
    
    while (attempts < maxAttempts && WiFi.status() != WL_CONNECTED) {
        attempts++;
        Serial.print("WiFi connection attempt ");
        Serial.print(attempts);
        Serial.print("/");
        Serial.println(maxAttempts);
        
        // More robust connection attempt
        WiFi.begin(ssid, password);
        
        unsigned long startAttemptTime = millis();
        const unsigned long attemptTimeout = 15000; // Increased timeout to 15 seconds
        
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < attemptTimeout) {
            delay(500);
            Serial.print(".");
            
            // Check for specific error conditions
            if (WiFi.status() == WL_CONNECT_FAILED) {
                Serial.println();
                Serial.println("Connection failed - likely wrong password");
                break;
            } else if (WiFi.status() == WL_NO_SSID_AVAIL) {
                Serial.println();
                Serial.println("SSID not available - network not found");
                break;
            }
        }
        Serial.println();
        
        // Print detailed status information
        wl_status_t status = WiFi.status();
        Serial.print("WiFi Status: ");
        switch(status) {
            case WL_IDLE_STATUS: Serial.println("WL_IDLE_STATUS"); break;
            case WL_NO_SSID_AVAIL: Serial.println("WL_NO_SSID_AVAIL (Network not found)"); break;
            case WL_SCAN_COMPLETED: Serial.println("WL_SCAN_COMPLETED"); break;
            case WL_CONNECTED: Serial.println("WL_CONNECTED"); break;
            case WL_CONNECT_FAILED: Serial.println("WL_CONNECT_FAILED (Wrong password?)"); break;
            case WL_CONNECTION_LOST: Serial.println("WL_CONNECTION_LOST"); break;
            case WL_DISCONNECTED: Serial.println("WL_DISCONNECTED"); break;
            default: Serial.println("Unknown status: " + String(status)); break;
        }
        
        if (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
            Serial.println("Connection failed, trying again...");
            WiFi.disconnect();
            delay(3000); // Increased delay between attempts
        }    }
    
    // Final connection status
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("=== WiFi Connection Successful ===");
        Serial.print("Connected to: ");
        Serial.println(WiFi.SSID());
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal Strength (RSSI): ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        Serial.print("Gateway: ");
        Serial.println(WiFi.gatewayIP());
        Serial.print("DNS: ");
        Serial.println(WiFi.dnsIP());
        Serial.print("MAC Address: ");
        Serial.println(WiFi.macAddress());
        Serial.println("==================================");
    } else {
        Serial.println("=== WiFi Connection Failed ===");
        Serial.print("Failed to connect to WiFi after ");
        Serial.print(maxAttempts);
        Serial.println(" attempts.");
        Serial.println("Troubleshooting steps:");
        Serial.println("1. Verify the WiFi network is active and broadcasting");
        Serial.println("2. Check the SSID spelling (case-sensitive)");
        Serial.println("3. Verify the WiFi password is correct");
        Serial.println("4. Ensure the ESP32 is within range of the router");
        Serial.println("5. Try restarting the router if needed");
        Serial.println("6. Consider checking for MAC address filtering");
        Serial.println("==============================");
    }
}

bool WifiMqttHandler::connectToWiFiWithCheck(const String& ssid, const String& password) {
    // Check for null pointers
    if (ssid.isEmpty() || password.isEmpty()) {
        Serial.println("Error: SSID or password is empty");
        return false;
    }

    // Connect to WiFi
    Serial.println("Connecting to WiFi...");
    connectToWiFi(ssid.c_str(), password.c_str());

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to connect to WiFi");
        return false;
    }

    return true;
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
    const int maxReconnectAttempts = 5;

    // Case 0: WLAN DOWN, MQTT DOWN
    if (WiFi.status() != WL_CONNECTED && !wifiSkipped) {
        if (millis() - lastWiFiReconnectAttempt > 15000) {
            wifiReconnectAttempts++;
            Serial.print("Reconnecting to WiFi... Attempt ");
            Serial.print(wifiReconnectAttempts);
            Serial.print("/");
            Serial.println(maxReconnectAttempts);
            
            if (wifiReconnectAttempts <= maxReconnectAttempts) {
                WiFi.disconnect();
                WiFi.begin(ssid, password);
                lastWiFiReconnectAttempt = millis();
                mqttReconnectAttempts = 0; // Reset MQTT attempts when WiFi reconnects
                mqttSkipped = false; // Reset MQTT skip flag when WiFi reconnects
            } else {
                Serial.println("Max WiFi reconnection attempts reached. Skipping WiFi reconnection.");
                wifiSkipped = true;
            }
        }
        return; // Wait for WLAN UP before proceeding to case 2
    }

    // Reset WiFi reconnection attempts when WiFi reconnects successfully
    if (WiFi.status() == WL_CONNECTED && wifiSkipped) {
        Serial.println("WiFi reconnected successfully. Resetting WiFi attempt counter.");
        wifiReconnectAttempts = 0;
        wifiSkipped = false;
        mqttReconnectAttempts = 0;
        mqttSkipped = false;
    }

    // Case 2: WLAN UP, MQTT DOWN
    if (!client.connected() && !mqttSkipped && !wifiSkipped) {
        if (millis() - lastMqttReconnectAttempt > 5000) {
            mqttReconnectAttempts++;
            Serial.print("Reconnecting to MQTT broker... Attempt ");
            Serial.print(mqttReconnectAttempts);
            Serial.print("/");
            Serial.println(maxReconnectAttempts);
            
            if (mqttReconnectAttempts <= maxReconnectAttempts) {
                connectToMqttBroker(client, espClient, mqtt_server, rootCACertificate, mqtt_port, clientId, topic);
                lastMqttReconnectAttempt = millis();
            } else {
                Serial.println("Max MQTT reconnection attempts reached. Skipping MQTT reconnection.");
                mqttSkipped = true;
            }
        }
        return; // Wait for MQTT UP before proceeding to case 4
    }

    // Reset MQTT reconnection attempts when MQTT reconnects successfully
    if (client.connected() && mqttSkipped) {
        Serial.println("MQTT reconnected successfully. Resetting MQTT attempt counter.");
        mqttReconnectAttempts = 0;
        mqttSkipped = false;
    }    // Case 4: WLAN UP, MQTT UP
    if (client.connected()) {
        client.loop(); // Ensure the MQTT client loop is called to maintain the connection
    }
}

void WifiMqttHandler::printConnectionStatus(PubSubClient &client) {
    Serial.println("\n=== Connection Status ===");
    
    // WiFi Status
    Serial.print("WiFi Status: ");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected to ");
        Serial.println(WiFi.SSID());
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal Strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("Disconnected");
    }
    
    // MQTT Status
    Serial.print("MQTT Status: ");
    if (client.connected()) {
        Serial.println("Connected");
    } else {
        Serial.print("Disconnected (State: ");
        Serial.print(client.state());
        Serial.println(")");
    }
    
    Serial.println("===========================");
}