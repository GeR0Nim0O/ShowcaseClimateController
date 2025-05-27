#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "Device.h"

class RotaryEncoder : public Device {
public:
    RotaryEncoder(uint8_t pinA, uint8_t pinB, uint8_t pinButton, const String& deviceName, int deviceIndex);
    
    bool begin() override;
    bool isConnected() override;
    void update() override;
    
    // Encoder reading methods
    long getPosition() const { return position; }
    void setPosition(long pos) { position = pos; }
    long getPositionChange();
    
    // Button methods
    bool isButtonPressed();
    bool wasButtonPressed(); // Returns true once per press
    bool isButtonHeld(unsigned long holdTime = 1000);
    
    // Configuration
    void setMinMax(long minVal, long maxVal);
    void setStepSize(int step) { stepSize = step; }
    void enableAcceleration(bool enable) { accelerationEnabled = enable; }

private:
    uint8_t pinA, pinB, pinButton;
    volatile long position;
    long lastPosition;
    long minValue, maxValue;
    int stepSize;
    bool accelerationEnabled;
    
    // Button state tracking
    bool buttonState;
    bool lastButtonState;
    bool buttonPressed;
    unsigned long buttonPressTime;
    unsigned long lastButtonTime;
    
    // Encoder state tracking
    uint8_t lastStateA;
    uint8_t lastStateB;
    
    void readEncoder();
    void readButton();
    static void encoderISR();
    static RotaryEncoder* instance; // For ISR callback
};

#endif // ROTARY_ENCODER_H
