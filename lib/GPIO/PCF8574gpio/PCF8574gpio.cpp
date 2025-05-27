#include "PCF8574gpio.h"

PCF8574gpio::PCF8574gpio(uint8_t addr) : i2cAddress(addr), initialized(false) {}

bool PCF8574gpio::begin() {
    pcf.begin(i2cAddress, Wire);
    
    // Test if PCF8574 is responding
    if (!pcf.isConnected()) {
        return false;
    }
    
    // Initialize all pins to desired states
    // ...any existing initialization code...
    
    // Set the initialized flag to true on successful initialization
    initialized = true;
    
    return true;
}

// ...existing code...