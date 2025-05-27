#include "JsonHandler.h"
#include <ArduinoJson.h>

void JsonHandler::sendJsonOverMqtt(PubSubClient &client, const char* deviceName, const char* projectNr, const char* showcaseId, 
                                   const char* sensorType, float sensorValue, const char* currentTime, int deviceIndex) {
    // Create topic for the sensor
    String baseTopic = "Casekeeper/" + String(projectNr) + "/" + String(showcaseId) + "/";
    String sensorTopic = baseTopic + String(sensorType) + "/" + String(deviceIndex);

    // Create JSON document
    JsonDocument doc;
    doc["timestamp"] = currentTime;
    doc["value"] = sensorValue;

    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    client.publish(sensorTopic.c_str(), buffer, n);
    Serial.println("Sending sensor data to MQTT topic:");
    Serial.println(sensorTopic + ": " + buffer);
}

String JsonHandler::createJson(float value, const char* currentTime) {
    JsonDocument doc; // Allocate memory for the JSON document

    // Add timestamp and value
    doc["timestamp"] = currentTime;
    doc["value"] = value;

    // Serialize the JSON document to a String
    String output;
    serializeJson(doc, output);
    return output;
}
