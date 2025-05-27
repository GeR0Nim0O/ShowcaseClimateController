#include "RotaryEncoder.h"

RotaryEncoder* RotaryEncoder::instance = nullptr;

RotaryEncoder::RotaryEncoder(uint8_t pinA, uint8_t pinB, uint8_t pinButton, const String& deviceName, int deviceIndex)
    : Device(0x00, 0xFF, deviceName, deviceIndex), // No I2C for direct GPIO
      pinA(pinA), pinB(pinB), pinButton(pinButton),
      position(0), lastPosition(0), minValue(LONG_MIN), maxValue(LONG_MAX),
      stepSize(1), accelerationEnabled(false),
      buttonState(HIGH), lastButtonState(HIGH), buttonPressed(false),
      buttonPressTime(0), lastButtonTime(0),
      lastStateA(HIGH), lastStateB(HIGH) {
    type = "Rotary_Encoder";
    instance = this; // Set static instance for ISR
}

bool RotaryEncoder::begin() {
    // Configure pins
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    pinMode(pinButton, INPUT_PULLUP);
    
    // Read initial states
    lastStateA = digitalRead(pinA);
    lastStateB = digitalRead(pinB);
    lastButtonState = digitalRead(pinButton);
    
    // Attach interrupts for encoder pins
    attachInterrupt(digitalPinToInterrupt(pinA), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), encoderISR, CHANGE);
    
    initialized = true;
    Serial.println("Rotary Encoder initialized successfully");
    return true;
}

bool RotaryEncoder::isConnected() {
    // For direct GPIO, always return true if initialized
    return initialized;
}

void RotaryEncoder::update() {
    readButton();
}

long RotaryEncoder::getPositionChange() {
    long change = position - lastPosition;
    lastPosition = position;
    return change;
}

bool RotaryEncoder::isButtonPressed() {
    return !buttonState; // Active low
}

bool RotaryEncoder::wasButtonPressed() {
    if (buttonPressed) {
        buttonPressed = false;
        return true;
    }
    return false;
}

bool RotaryEncoder::isButtonHeld(unsigned long holdTime) {
    if (isButtonPressed() && buttonPressTime > 0) {
        return (millis() - buttonPressTime) >= holdTime;
    }
    return false;
}

void RotaryEncoder::setMinMax(long minVal, long maxVal) {
    minValue = minVal;
    maxValue = maxVal;
    
    // Constrain current position
    if (position < minValue) position = minValue;
    if (position > maxValue) position = maxValue;
}

void RotaryEncoder::readEncoder() {
    uint8_t stateA = digitalRead(pinA);
    uint8_t stateB = digitalRead(pinB);
    
    // Check for state change
    if (stateA != lastStateA || stateB != lastStateB) {
        // Determine direction
        if (lastStateA == LOW && stateA == HIGH) {
            if (stateB == LOW) {
                position += stepSize; // Clockwise
            } else {
                position -= stepSize; // Counter-clockwise
            }
        }
        
        // Constrain to min/max values
        if (position < minValue) position = minValue;
        if (position > maxValue) position = maxValue;
        
        lastStateA = stateA;
        lastStateB = stateB;
    }
}

void RotaryEncoder::readButton() {
    bool currentButtonState = digitalRead(pinButton);
    unsigned long currentTime = millis();
    
    // Debounce button (50ms)
    if (currentTime - lastButtonTime > 50) {
        if (currentButtonState != lastButtonState) {
            buttonState = currentButtonState;
            
            if (!buttonState && lastButtonState) { // Button pressed (HIGH to LOW)
                buttonPressed = true;
                buttonPressTime = currentTime;
            } else if (buttonState && !lastButtonState) { // Button released (LOW to HIGH)
                buttonPressTime = 0;
            }
            
            lastButtonState = buttonState;
            lastButtonTime = currentTime;
        }
    }
}

void RotaryEncoder::encoderISR() {
    if (instance) {
        instance->readEncoder();
    }
}
