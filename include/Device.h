// Add the declaration of the new methods to the public section

public:
    // ...existing methods...
    
    // Safe method to manually set initialized state (for emergency recovery)
    void forceInitialized(bool state);
    
    // Check if device is connected to I2C bus
    bool isConnected();
