#ifndef CLIMATE_CONTROLLER_H
#define CLIMATE_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>
#include "PCF8574gpio.h"
#include "SHTsensor.h"
#include "GP8403dac.h"
#include "Configuration.h"

enum class ClimateMode {
    OFF,
    HEATING,
    COOLING,
    AUTO
};

enum class HumidityMode {
    OFF,
    HUMIDIFYING,
    DEHUMIDIFYING,
    AUTO
};

class ClimateController {
public:
    ClimateController(PCF8574gpio* gpioExpander, SHTsensor* tempHumSensor, GP8403dac* dac = nullptr);
    
    bool begin();
    void update();
    
    // Setpoint management
    void setTemperatureSetpoint(float temp) { temperatureSetpoint = temp; }
    void setHumiditySetpoint(float hum) { humiditySetpoint = hum; }
    float getTemperatureSetpoint() const { return temperatureSetpoint; }
    float getHumiditySetpoint() const { return humiditySetpoint; }
    
    // Mode control
    void setClimateMode(ClimateMode mode) { climateMode = mode; }
    void setHumidityMode(HumidityMode mode) { humidityMode = mode; }
    ClimateMode getClimateMode() const { return climateMode; }
    HumidityMode getHumidityMode() const { return humidityMode; }
    
    // Current readings
    float getCurrentTemperature() const { return currentTemperature; }
    float getCurrentHumidity() const { return currentHumidity; }
    
    // PID parameters
    void setTemperaturePID(double kp, double ki, double kd);
    void setHumidityPID(double kp, double ki, double kd);
    
    // Fan control
    void setFanInterior(bool enable);
    void setFanExterior(bool enable);
    
    // DAC control
    void setDACDevice(GP8403dac* dac) { this->dac = dac; }
    bool hasDACControl() const { return dac != nullptr; }
    void setHeatingPower(float percentage); // 0-100%
    void setCoolingPower(float percentage); // 0-100%
    void setHumidifierPower(float percentage); // 0-100%
    void setDehumidifierPower(float percentage); // 0-100%
    float getHeatingPower() const { return heatingPower; }
    float getCoolingPower() const { return coolingPower; }
    float getHumidifierPower() const { return humidifierPower; }
    float getDehumidifierPower() const { return dehumidifierPower; }

    // Status
    bool isHeating() const { return heatingActive; }
    bool isCooling() const { return coolingActive; }
    bool isHumidifying() const { return humidifyingActive; }
    bool isDehumidifying() const { return dehumidifyingActive; }

private:
    PCF8574gpio* gpio;
    SHTsensor* sensor;
    GP8403dac* dac;  // Add DAC device pointer
    
    // Setpoints
    float temperatureSetpoint;
    float humiditySetpoint;
    
    // Current values
    float currentTemperature;
    float currentHumidity;
    
    // Control modes
    ClimateMode climateMode;
    HumidityMode humidityMode;
    
    // PID controllers
    PID* temperaturePID;
    PID* humidityPID;
    
    // PID variables
    double tempInput, tempOutput, tempSetpoint;
    double humInput, humOutput, humSetpoint;
    
    // Control states
    bool heatingActive;
    bool coolingActive;
    bool humidifyingActive;
    bool dehumidifyingActive;
    bool tempControlEnabled;
    
    // Power levels for analog control
    float heatingPower;
    float coolingPower;
    float humidifierPower;
    float dehumidifierPower;
    
    // Timing
    unsigned long lastUpdate;
    unsigned long updateInterval;
    
    // Pin mappings from configuration
    uint8_t pinTemperatureEnable;
    uint8_t pinTemperatureHeat;
    uint8_t pinTemperatureCool;
    uint8_t pinHumidify;
    uint8_t pinDehumidify;
    uint8_t pinFanInterior;
    uint8_t pinFanExterior;
    
    // Control methods
    void updateTemperatureControl();
    void updateHumidityControl();
    void updateSensorReadings();
    void applyTemperatureControl();
    void applyHumidityControl();
    
    // DAC control method
    void applyDACControls();
    
    // Pin mapping helper methods
    void initializePinMappings();
    uint8_t getPinFromChannelName(const String& channelName);
    
    // Safety checks
    bool checkSafetyLimits();
    void emergencyShutdown();
    
    // Add this helper method for safe GPIO operations
    bool safeWritePin(uint8_t pin, bool value);
};

#endif // CLIMATE_CONTROLLER_H
