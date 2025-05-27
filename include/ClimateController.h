#ifndef CLIMATE_CONTROLLER_H
#define CLIMATE_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>
#include "PCF8574gpio.h"
#include "SHT31sensor.h"
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
    ClimateController(PCF8574gpio* gpioExpander, SHT31sensor* tempHumSensor);
    
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
    
    // Status
    bool isHeating() const { return heatingActive; }
    bool isCooling() const { return coolingActive; }
    bool isHumidifying() const { return humidifyingActive; }
    bool isDehumidifying() const { return dehumidifyingActive; }

private:
    PCF8574gpio* gpio;
    SHT31sensor* sensor;
    
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
    
    // Timing
    unsigned long lastUpdate;
    unsigned long updateInterval;
    
    // Control methods
    void updateTemperatureControl();
    void updateHumidityControl();
    void updateSensorReadings();
    void applyTemperatureControl();
    void applyHumidityControl();
    
    // Safety checks
    bool checkSafetyLimits();
    void emergencyShutdown();
};

#endif // CLIMATE_CONTROLLER_H
