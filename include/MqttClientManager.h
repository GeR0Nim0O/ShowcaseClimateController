#ifndef MQTTCLIENTMANAGER_H
#define MQTTCLIENTMANAGER_H

#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Arduino.h>

class MqttClientManager {
public:
    static MqttClientManager& getInstance();
    
    // Initialize the MQTT client
    void initialize();
    
    // Get client references
    WiFiClientSecure& getWiFiClient() { return espClient; }
    PubSubClient& getMqttClient() { return client; }
    
    // Connection management
    bool connectToMqtt(const String& server, const char* rootCACert, int port, 
                      const String& clientId, const String& topic, const String& token);
    bool isConnected() const { return client.connected(); }
    void loop() { if (client.connected()) client.loop(); }
    
    // Data sending
    void sendSensorData(const String& deviceName, const String& projectNr, 
                       const String& showcaseId, const String& sensorType, 
                       float value, const String& currentTime, int deviceIndex);
    
private:
    MqttClientManager() = default;
    ~MqttClientManager() = default;
    MqttClientManager(const MqttClientManager&) = delete;
    MqttClientManager& operator=(const MqttClientManager&) = delete;
    
    WiFiClientSecure espClient;
    PubSubClient client;
    bool initialized = false;
};

#endif // MQTTCLIENTMANAGER_H
