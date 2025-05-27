#ifndef JSONHANDLER_H
#define JSONHANDLER_H

#include <Arduino.h>
#include <PubSubClient.h>

class JsonHandler {
public:
    static void sendJsonOverMqtt(PubSubClient &client, const char* deviceName, const char* projectNr, const char* showcaseId, 
                                 const char* sensorType, float sensorValue, const char* currentTime, int deviceIndex);
    static String createJson(float value, const char* currentTime);
};

#endif