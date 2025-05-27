# Climate Controller Project

An object-oriented climate control system for ESP32-S3 that manages temperature and humidity using I2C devices through a PCA9548A multiplexer.

## Project Structure

### Device Hierarchy
```
Device (Base Class)
├── PCF8574_GPIO (GPIO Expander)
├── SHT31_Sensor (Temperature/Humidity Sensor)
├── Display (OLED Display)
├── GP8403dac (Digital-to-Analog Converter)
└── RotaryEncoder (User Input)
```

### Library Organization
```
lib/
├── Device/                     # Base device class and registry
│   ├── Device.h/.cpp          # Abstract base class for all devices
│   └── DeviceRegistry/        # Manages all devices
├── GPIO/                      # GPIO expansion devices
│   └── PCF8574_GPIO/         # PCF8574 I2C GPIO expander
├── Sensors/                   # Environmental sensors
│   └── SHT31_Sensor/         # Temperature/Humidity sensor
├── Display/                   # Display devices
│   └── Display.h/.cpp        # OLED display management
├── DAC/                      # Digital-to-analog converters
│   └── DAC_Module/           # MCP4725 DAC for power control
├── Input/                    # User input devices
│   └── RotaryEncoder/        # Rotary encoder with button
├── ClimateController/        # Main control logic
│   └── ClimateController.h/.cpp # PID-based climate control
└── Config/                   # Configuration management
    └── ClimateConfig/        # EEPROM-based settings storage
```

## Hardware Configuration

### I2C Device Mapping (via PCA9548A Multiplexer)
- **Channel 0**: PCF8574 GPIO Expander (0x20)
- **Channel 1**: SHT31 Temperature/Humidity Sensor (0x44)
- **Channel 2**: SSD1306 OLED Display (0x3C)
- **Channel 3**: MCP4725 DAC Module (0x62)

### GPIO Pin Assignments
- **Pin 4**: Rotary Encoder A
- **Pin 5**: Rotary Encoder B
- **Pin 6**: Rotary Encoder Button
- **Pin 17**: I2C SDA
- **Pin 16**: I2C SCL

### PCF8574 GPIO Expander Pin Mapping
- **Pin 0**: Interior Fan Control
- **Pin 1**: Exterior Fan Control
- **Pin 2**: Humidify Control
- **Pin 3**: Dehumidify Control
- **Pin 4**: Temperature Control Enable
- **Pin 5**: Cooling Control
- **Pin 6**: Heating Control
- **Pin 7**: Spare Output

## Control System Features

### Temperature Control
- **Enable/Disable**: Digital output for temperature module power
- **Heating**: Digital output for heating element
- **Cooling**: Digital output for cooling element
- **Power Control**: Analog output (0-100%) via DAC for temperature control power

### Humidity Control
- **Humidify**: Digital output for humidification
- **Dehumidify**: Digital output for dehumidification

### PID Control
- Independent PID controllers for temperature and humidity
- Configurable PID parameters stored in EEPROM
- Safety limits and emergency shutdown capabilities

### User Interface
- Rotary encoder for setpoint adjustment
- Button press for saving settings
- OLED display showing current values and setpoints
- Real-time status updates

## Software Architecture

### Device Management
- **DeviceRegistry**: Singleton pattern for managing all devices
- **Device Base Class**: Common interface for all I2C devices
- **Automatic Initialization**: Sequential device initialization with error handling

### Configuration Management
- **ClimateConfig**: Singleton configuration manager
- **EEPROM Storage**: Persistent settings with checksum validation
- **Default Values**: Automatic fallback to safe defaults

### Control Logic
- **ClimateController**: Main control class with PID loops
- **Safety Monitoring**: Continuous monitoring of sensor limits
- **Emergency Shutdown**: Automatic shutdown on safety violations

### Features
- Real-time climate monitoring and control
- Configurable PID parameters
- Fan control for air circulation
- User-friendly interface with rotary encoder
- Persistent configuration storage
- Automatic git commits for version tracking

## Usage

1. **Setup**: The system automatically initializes all devices on startup
2. **Monitor**: View current temperature and humidity on the display
3. **Adjust**: Use the rotary encoder to change setpoints
4. **Save**: Press the encoder button to save settings to EEPROM
5. **Control**: The system automatically maintains target conditions using PID control

## Dependencies

- PID Library for control algorithms
- Adafruit libraries for display support
- Standard Arduino libraries for I2C and EEPROM

## Safety Features

- Temperature and humidity limit monitoring
- Automatic emergency shutdown on sensor failures
- Configurable safety limits
- Checksum validation for stored settings

This project is an ESP32S3-based climate controller designed for demonstration and educational purposes. It features I2C device scanning, GPIO expansion, and basic climate control logic.

## Features

- I2C bus scanning (direct and via PCA9548A multiplexer)
- PCF8574 GPIO expander control
- Basic climate actuator control (fans, humidify, dehumidify, heat, cool)
- Designed for ESP32S3 with PlatformIO and Arduino framework
- Automatic Git commit and push on file save (with VSCode "Run on Save" extension)

## Hardware

- ESP32S3 DevKitC-1
- PCA9548A I2C multiplexer
- PCF8574 I2C GPIO expander
- Various actuators (fans, humidifier, dehumidifier, heater, cooler)

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) extension for VSCode
- [VSCode](https://code.visualstudio.com/)
- "Run on Save" VSCode extension (optional, for auto Git commit/push)
- Git installed and configured

### Setup

1. Clone this repository:
    ```sh
    git clone <your-repo-url>
    ```
2. Open the project folder in VSCode.
3. Install PlatformIO and required libraries (see `platformio.ini`).
4. Connect your ESP32S3 board.
5. Build and upload the firmware using PlatformIO.

### Auto Git Commit/Push (Optional)

To automatically commit and push changes on every file save:
- Install the "Run on Save" extension in VSCode.
- Ensure `.vscode/settings.json` contains:
    ```jsonc
    "emeraldwalk.runonsave": {
        "commands": [
            {
                "match": ".*",
                "cmd": "git add . && git commit -m \"Auto-commit on save\" && git push"
            }
        ]
    }
    ```

## Usage

- On boot, the ESP32S3 scans for I2C devices and initializes the PCF8574 GPIO expander.
- Actuators can be controlled via the PCF8574 outputs.
- Serial output provides I2C scan results and status messages.

## License

This project is for educational use. See LICENSE file for details (if present).

## Author

Ron Groenen
