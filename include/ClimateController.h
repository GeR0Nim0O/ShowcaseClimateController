#ifndef CLIMATE_CONTROLLER_H
#define CLIMATE_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>
// Undefine LIBRARY_VERSION to prevent redefinition warning from PID_AutoTune_v0.h
#ifdef LIBRARY_VERSION
#undef LIBRARY_VERSION
#endif
#include <PID_AutoTune_v0.h>
#include "PCF8574gpio.h"
#include "SHTsensor.h"
#include "GP8403dac.h"
#include "Configuration.h"

// Removed ClimateMode and HumidityMode enums - using simple boolean enable flags

class ClimateController {
public:
    // Static factory method for automatic device discovery and initialization
    static ClimateController* createFromDeviceRegistry();
    
    // Static method for controlled update with timing management
    static void updateControllerWithTiming(ClimateController* controller);
      // Configuration method for setting all parameters at once
    void configure(float tempSetpoint, float humSetpoint, bool enableTemperatureControl, bool enableHumidityControl);
    
    ClimateController(PCF8574gpio* gpioExpander, SHTsensor* tempHumSensor, GP8403dac* dac = nullptr);
    ~ClimateController();
    
    bool begin();
    void update();
    
    // Setpoint management
    void setTemperatureSetpoint(float temp) { temperatureSetpoint = temp; }
    void setHumiditySetpoint(float hum) { humiditySetpoint = hum; }
    float getTemperatureSetpoint() const { return temperatureSetpoint; }
    float getHumiditySetpoint() const { return humiditySetpoint; }
      // Control enable/disable
    void setTemperatureControlEnabled(bool enabled) { temperatureControlEnabled = enabled; }
    void setHumidityControlEnabled(bool enabled) { humidityControlEnabled = enabled; }
    bool isTemperatureControlEnabled() const { return temperatureControlEnabled; }
    bool isHumidityControlEnabled() const { return humidityControlEnabled; }
    
    // Current readings
    float getCurrentTemperature() const { return currentTemperature; }
    float getCurrentHumidity() const { return currentHumidity; }      // PID parameters
    void setTemperaturePID(double kp, double ki, double kd);// PID AutoTune functionality - Temperature only (humidity uses digital on/off control)
    bool startTemperatureAutoTune(double targetSetpoint = 0.0, double outputStep = 0.0, double noiseband = 0.0, unsigned int lookBack = 0);
    bool startTemperatureAutoTuneFast(double targetSetpoint = 0.0, double outputStep = 0.0, double noiseband = 0.0, unsigned int lookBack = 0);
    void stopAutoTune();
    void setFastTestingMode(bool enabled); // Speeds up controller for testing
    bool isAutoTuning() const { return temperatureAutoTuning; }
    bool isTemperatureAutoTuning() const { return temperatureAutoTuning; }
    void updateAutoTune();
    void getAutoTuneResults(double& kp, double& ki, double& kd);
    void printAutoTuneStatus();
      // Configuration reload
    void reloadConfiguration();
    
    // Dynamic configuration update
    void updateClimateConfigFile();
    
    // Fan control
    void setFanInterior(bool enable);
    void setFanExterior(bool enable);
    void setAutoFanControl(bool enable) { autoFanControlEnabled = enable; }
    bool isAutoFanControlEnabled() const { return autoFanControlEnabled; }
    bool isFanInteriorOn() const { return fanInteriorActive; }
    bool isFanExteriorOn() const { return fanExteriorActive; }
      // DAC control
    void setDACDevice(GP8403dac* dac) { this->dac = dac; }
    bool hasDACControl() const { return dac != nullptr; }
    
    // DAC setup state control
    void setDACSetupMode(bool setupActive); // 5V during setup, 0V when finished
    
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
    bool isCooling() const { return coolingActive; }    bool isHumidifying() const { return humidifyingActive; }
    bool isDehumidifying() const { return dehumidifyingActive; }
    
    // Status reporting
    void printClimateStatus();
    
    // New methods for enhanced status reporting
    void printClimateStatusIfChanged();
    bool hasSignificantStateChange();
    bool hasSignificantSensorChange();
    void updateStatusPrintTracking();
    
    // Dew point compensation
    void setRadiatorSensor(SHTsensor* radiatorSensor) { this->radiatorSensor = radiatorSensor; }
    bool hasDewPointCompensation() const { return radiatorSensor != nullptr && isDewPointCompensationEnabled(); }
    bool isDewPointCompensationEnabled() const;
    float calculateDewPoint(float temperature, float humidity) const;
    float getDewPoint() const { return dewPoint; }
    float getCurrentRadiatorTemperature() const { return currentRadiatorTemperature; }
    float getMinAllowedCoolingTemperature() const { return minAllowedCoolingTemperature; }

private:
    PCF8574gpio* gpio;
    SHTsensor* sensor;
    GP8403dac* dac;  // Add DAC device pointer
    
    // Dew point compensation
    SHTsensor* radiatorSensor;
    float dewPoint;
    float currentRadiatorTemperature;
    float minAllowedCoolingTemperature;
    unsigned long lastDewPointUpdate;
    
    // Setpoints
    float temperatureSetpoint;
    float humiditySetpoint;
    
    // Current values
    float currentTemperature;
    float currentHumidity;
      // Control modes - simplified to boolean enable flags
    bool temperatureControlEnabled;
    bool humidityControlEnabled;// PID controllers
    PID* temperaturePID;
      // PID AutoTune controllers - Temperature only (humidity uses digital control)
    PID_ATune* temperatureAutoTuner;
    bool temperatureAutoTuning;
    double autoTuneSetpoint;
    double autoTuneOutputStep;
    unsigned long autoTuneStartTime;
    
    // AutoTune type tracking for percentage calculation
    enum class AutoTuneType { NORMAL, FAST };
    AutoTuneType currentAutoTuneType;
    unsigned long expectedAutoTuneDuration; // in milliseconds
      // PID variables
    double tempInput, tempOutput, tempSetpoint;
    
    // Control states
    bool heatingActive;
    bool coolingActive;
    bool humidifyingActive;
    bool dehumidifyingActive;
    bool tempControlEnabled;
    bool fanInteriorActive;      // Add fan state tracking
    bool fanExteriorActive;      // Add fan state tracking
    bool autoFanControlEnabled; // Add auto fan control flag
    
    // Power levels for analog control
    float heatingPower;
    float coolingPower;
    float humidifierPower;
    float dehumidifierPower;
    
    // Timing
    unsigned long lastUpdate;
    unsigned long updateInterval;
    
    // Status printing frequency and tracking
    unsigned long lastStatusPrint;
    unsigned long statusPrintInterval;
    float lastPrintedTemperature;
    float lastPrintedHumidity;
    bool lastPrintedHeatingActive;
    bool lastPrintedCoolingActive;
    bool lastPrintedHumidifyingActive;
    bool lastPrintedDehumidifyingActive;
    bool lastPrintedFanInteriorActive;    bool lastPrintedFanExteriorActive;
    bool lastPrintedTemperatureControlEnabled;
    bool lastPrintedHumidityControlEnabled;
    float temperatureThreshold;
    float humidityThreshold;
    
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
    void updateFanControl();        // Add fan control method
    void updateSensorReadings();
    void applyTemperatureControl();
    void applyHumidityControl();
    void applyFanControl();         // Add fan application method
    
    // Dew point compensation methods
    void updateDewPointCompensation();
    void updateRadiatorSensorReading();
    float limitCoolingOutputForDewPoint(float coolingOutput);
    
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
