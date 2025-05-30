#ifndef DEVICE_H
#define DEVICE_H

#include <Arduino.h>
#include <Wire.h>
#include <map>
#include <string>

class Device {
public:
    Device(TwoWire* wire, uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex);
    Device(TwoWire* wire, float threshold, std::map<String, String> channels, uint8_t i2cAddress, uint8_t tcaChannel, int deviceIndex);
    virtual ~Device() = default;
      virtual bool begin();
    virtual bool isConnected();
    virtual void update();
    virtual std::map<String, String> readData();
    virtual std::map<String, String> getChannels() const { return channels; }
    virtual float getThreshold(const String& channelKey = "") const { 
        if (channelKey.length() > 0 && channelThresholds.find(channelKey) != channelThresholds.end()) {
            return channelThresholds.at(channelKey);
        }
        return threshold; 
    }
    virtual int getChannelsCount() const { return channels.size(); }
    virtual String getTypeNumber() const { return typeNumber; }
    
    // Getters
    uint8_t getI2CAddress() const { return i2cAddress; }
    uint8_t getTCAChannel() const { return tcaChannel; }
    String getDeviceName() const { return deviceName; }
    String getDeviceLabel() const { return deviceLabel; }
    int getDeviceIndex() const { return deviceIndex; }    String getType() const { return type; }
    bool isInitialized() const { return initialized; }
    void setTypeNumber(const String& tn) { typeNumber = tn; }
    void setDeviceLabel(const String& label) { deviceLabel = label; }
    void setChannelThresholds(const std::map<String, float>& thresholds) { channelThresholds = thresholds; }    // Make deviceName public so it can be set from outside
    String deviceName;
    
    // Sensor labeling for location identification
    String deviceLabel;  // e.g., "Interior", "Exterior", "Control", etc.

protected:
    TwoWire* wire;  // Wire instance - first member for proper initialization
    uint8_t i2cAddress;
    uint8_t tcaChannel;
    int deviceIndex;
    String type;
    String typeNumber;
    bool initialized;
    float threshold;
    std::map<String, String> channels;
    std::map<String, float> channelThresholds;  // Channel-specific thresholds
    
    void selectTCAChannel(uint8_t channel);
    bool testI2CConnection();
};

#endif // DEVICE_H
