# Showcase Climate Controller

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
