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
    // Primary constructor - used by most devices
    Device(TwoWire* wire, const String& type, const String& deviceName, uint8_t i2cAddress, uint8_t tcaChannel, int deviceIndex);
    
    // Alternative constructor for legacy devices that specify threshold and channels first
    Device(TwoWire* wire, float threshold, std::map<String, String> channels, uint8_t i2cAddress, uint8_t tcaChannel, int deviceIndex);
    
    // Constructor for devices that only need basic parameters
    Device(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex);
    
    virtual ~Device() = default;
    
    virtual bool begin();
    virtual bool isInitialized() const { return initialized; }
    virtual String getType() const { return type; }
    virtual String getDeviceName() const { return deviceName; }
    virtual uint8_t getI2CAddress() const { return i2cAddress; }
    virtual uint8_t getTCAChannel() const { return tcaChannel; }
    virtual int getDeviceIndex() const { return deviceIndex; }
    
    // Add a method to get the wire instance used by this device
    TwoWire* getWireInstance() const { return wire; }
    
    // Virtual methods that derived classes can override
    virtual bool isConnected();
    virtual void update() {}
    virtual std::map<String, String> getChannels() const { return channels; }
    virtual float getThreshold(const String& channelId) const;
    virtual void setChannelThresholds(const std::map<String, float>& channelThresholds) { thresholds = channelThresholds; }
    
    // Channel management
    void addChannel(const String& channelId, const String& channelType, float threshold = 0.0);
    void removeChannel(const String& channelId);
    void clearChannels();
    
    // Data reading/writing
    virtual std::map<String, String> readData();
    virtual bool writeData(const std::map<String, String>& data);
    
    // Connection management
    virtual bool connect();
    virtual bool disconnect();
    
    // I2C utility methods
    void selectTCAChannel(uint8_t channel);
    bool testI2CConnection();
    
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