#ifndef DEVICE_H
#define DEVICE_H

#include <Arduino.h>
#include <Wire.h>
#include <map>
#include <string>

class Device {
public:
    Device(uint8_t i2cAddress, uint8_t tcaChannel, const String& deviceName, int deviceIndex);
    virtual ~Device() = default;
    
    virtual bool begin() = 0;
    virtual bool isConnected() = 0;
    virtual void update() = 0;
    
    // Pure virtual methods for polymorphic access
    virtual std::map<std::string, float> readData() = 0;
    virtual std::map<String, String> getChannels() const = 0;
    virtual float getThreshold(const String& channelKey) const = 0;
    
    // Getters
    uint8_t getI2CAddress() const { return i2cAddress; }
    uint8_t getTCAChannel() const { return tcaChannel; }
    String getDeviceName() const { return deviceName; }
    int getDeviceIndex() const { return deviceIndex; }
    String getType() const { return type; }
    bool isInitialized() const { return initialized; }

protected:
    uint8_t i2cAddress;
    uint8_t tcaChannel;
    String deviceName;
    int deviceIndex;
    String type;
    bool initialized;
    
    void selectTCAChannel(uint8_t channel);
    bool testI2CConnection();
};

#endif // DEVICE_H
