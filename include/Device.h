#ifndef DEVICE_H
#define DEVICE_H

#include <Arduino.h>
#include <Wire.h>
#include <map>
#include <vector>
#include <functional>
#include <ArduinoJson.h>

class Device {
public:
    Device(TwoWire* wire, const String& type, const String& deviceName, uint8_t i2cAddress, uint8_t tcaChannel, int deviceIndex);
    
    virtual bool begin();
    virtual bool isInitialized() const { return initialized; }
    virtual String getType() const { return type; }
    virtual String getDeviceName() const { return deviceName; }
    virtual uint8_t getI2CAddress() const { return i2cAddress; }
    virtual uint8_t getTCAChannel() const { return tcaChannel; }
    virtual int getDeviceIndex() const { return deviceIndex; }
    
    // Add a method to get the wire instance used by this device
    TwoWire* getWireInstance() const { return wire; }
    
    // Channel management
    void addChannel(const String& channelId, const String& channelType, float threshold = 0.0);
    void removeChannel(const String& channelId);
    void clearChannels();
    const std::map<String, String>& getChannels() const { return channels; }
    float getThreshold(const String& channelId) const;
    
    // Data reading/writing
    virtual std::map<String, String> readData();
    virtual bool writeData(const std::map<String, String>& data);
    
    // Connection management
    virtual bool connect();
    virtual bool disconnect();
    virtual bool isConnected() const;
    
protected:
    TwoWire* wire;
    String type;
    String deviceName;
    uint8_t i2cAddress;
    uint8_t tcaChannel;
    int deviceIndex;
    bool initialized = false;
    
    // Channels map - channel ID to type
    std::map<String, String> channels;
    // Thresholds map - channel ID to threshold value
    std::map<String, float> thresholds;
};

#endif // DEVICE_H