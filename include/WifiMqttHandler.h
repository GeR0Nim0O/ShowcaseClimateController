#ifndef WIFIMQTTHANDLER_H
#define WIFIMQTTHANDLER_H

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

class WifiMqttHandler {
public:
    static void connectToWiFi(const char* ssid, const char* password);
    static void connectToMqttBroker(PubSubClient &client, WiFiClientSecure &espClient, const char* mqtt_server, const char* rootCACertificate, int mqtt_port, const char* clientId, const char* topic, const char* token = nullptr);
    static void keepAlive(PubSubClient &client, WiFiClientSecure &espClient, const char* ssid, const char* password, const char* mqtt_server, const char* rootCACertificate, int mqtt_port, const char* clientId, const char* topic);

    // New function to connect to WiFi with check
    static bool connectToWiFiWithCheck(const String& ssid, const String& password);    // New function to connect to MQTT broker with check
    static bool connectToMqttBrokerWithCheck(PubSubClient &client, WiFiClientSecure &espClient, const String& mqtt_server, const char* rootCACertificate, int mqtt_port, const String& clientId, const String& topic, const String& token = "");

    static void setupSecureClient(WiFiClientSecure &espClient, const char* rootCACertificate);
    
    // Status reporting functions
    static void printConnectionStatus(PubSubClient &client);
};

#endif // WIFIMQTTHANDLER_H