#include "SHT31sensor.h"
#include <Wire.h>
#include "I2CHandler.h"

SHT31sensor::SHT31sensor(uint8_t address, uint8_t tcaChannel) : Sensor(address, tcaChannel), sht(address) {
    // Constructor implementation
}

void SHT31sensor::begin() {
    // Initialization code for SHT31 sensor
    Wire.begin();
    sht.begin();
}

void SHT31sensor::update() {
    // Always select the correct TCA channel before communicating
    selectTCAChannel(tcaChannel);
    
    // Reset readingSuccess flag
    bool readingSuccess = false;
    
    // Try multiple times if needed
    for (int attempt = 0; attempt < 3 && !readingSuccess; attempt++) {
        if (sht.readTempHum()) {  // This reads both temperature and humidity
            temperature = sht.getTemperature();
            humidity = sht.getHumidity();
            readingSuccess = true;
            Serial.print("SHT31 reading successful - Temp: ");
            Serial.print(temperature);
            Serial.print("°C, Humidity: ");
            Serial.println(humidity);
        } else {
            Serial.print("SHT31 reading failed, attempt ");
            Serial.print(attempt + 1);
            Serial.println("/3");
            delay(100);  // Wait before retrying
        }
    }
    
    if (!readingSuccess) {
        Serial.println("SHT31 reading failed after multiple attempts");
    }
}

std::map<String, String> SHT31sensor::readData() {
    // Always call update to get fresh sensor readings before returning data
    update();
    
    std::map<String, String> result;
    result["T"] = String(temperature);
    result["H"] = String(humidity);
    
    return result;
}