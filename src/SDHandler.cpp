#include "SDHandler.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

#define SD_MISO_PIN 5
#define SD_MOSI_PIN 6
#define SD_SCLK_PIN 7
#define SD_CS_PIN   4

File logFile;
bool overwriteConfirmed = false;

bool SDHandler::initSDCard() {
    pinMode(SD_MISO_PIN, INPUT_PULLUP);
    SPI.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        return false;
    }
    Serial.println("SD card initialized.");
    return true;
}

bool SDHandler::checkAndCreateLogFile() {
    if (!SD.exists("/log.txt")) {
        logFile = SD.open("/log.txt", FILE_WRITE);
        if (!logFile) {
            Serial.println("Failed to create log file");
            return false;
        }
        logFile.close();
        Serial.println("Log file created");
    }
    return true;
}

bool SDHandler::initializeSDCardAndConfig() {
    if (!initSDCard()) {
        Serial.println("SD Card initialization failed");
        return false;
    }

    if (!checkAndCreateLogFile()) {
        Serial.println("Failed to ensure log file exists");
        return false;
    } else {
        Serial.println("Log file is present and ready for use.");
        Serial.println(getLogFileInfo());
    }

    if (!SD.exists("/config.json")) {
        Serial.println("Config file does not exist. Copying default config to SD card.");
        if (copyDefaultConfig()) {
            Serial.println("Default config copied to SD card.");
        } else {
            Serial.println("Failed to copy default config to SD card.");
            return false;
        }
    }
    return true;
}

void SDHandler::logJson(const char* json) {
    if (!SD.cardSize()) {
        Serial.println("WARNING: SD card not present - data logging skipped");
        return;
    }
    logFile = SD.open("/log.txt", FILE_APPEND);
    if (!logFile) {
        Serial.println("WARNING: Error opening log file on SD card - data logging skipped");
        return;
    }
    if (logFile.size() > MAX_FILE_SIZE) {
        logFile.close();
        File tempFile = SD.open("/log.txt", FILE_READ);
        String fileContent = "";
        while (tempFile.available()) {
            fileContent += char(tempFile.read());
        }
        tempFile.close();
        int firstNewline = fileContent.indexOf('\n');
        if (firstNewline != -1) {
            fileContent = fileContent.substring(firstNewline + 1);
        }
        SD.remove("/log.txt");
        logFile = SD.open("/log.txt", FILE_WRITE);
        logFile.print(fileContent);
    }
    logFile.println(json);
    logFile.flush();
    Serial.println("Logged to SD: " + String(json));
}

// Listing directory contents
void SDHandler::listDir(const char* dirname, uint8_t levels) {
    Serial.printf("Listing directory: %s\n", dirname);

    File root = SD.open(dirname);
    if (!root) {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels) {
                listDir(file.name(), levels - 1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

// Create directory
void SDHandler::createDir(const char* path) {
    Serial.printf("Creating Dir: %s\n", path);
    if (SD.mkdir(path)) {
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

// Remove directory
void SDHandler::removeDir(const char* path) {
    Serial.printf("Removing Dir: %s\n", path);
    if (SD.rmdir(path)) {
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

// Read file contents
void SDHandler::readFile(const char* path) {
    Serial.printf("Reading file: %s\n", path);

    File file = SD.open(path);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while (file.available()) {
        Serial.write(file.read());
    }
}

// Write to a file
void SDHandler::writeFile(const char* path, const char* message) {
    Serial.printf("Writing file: %s\n", path);

    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message)) {
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
}

// Append to a file
void SDHandler::appendFile(const char* path, const char* message) {
    Serial.printf("Appending to file: %s\n", path);

    File file = SD.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message)) {
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
}

// Rename a file
void SDHandler::renameFile(const char* path1, const char* path2) {
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (SD.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

// Delete a file
void SDHandler::deleteFile(const char* path) {
    Serial.printf("Deleting file: %s\n", path);
    if (SD.remove(path)) {
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

// Test file I/O
void SDHandler::testFileIO(const char* path) {
    File file = SD.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    
    if (file) {
        len = file.size();
        size_t flen = len;
        start = millis();
        while (len) {
            size_t toRead = len;
            if (toRead > 512) {
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }

    file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for (i = 0; i < 2048; i++) {
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

String SDHandler::getLogFileInfo() {
    File logFile = SD.open("/log.txt");
    if (!logFile) {
        return "Failed to open log file for size and date information.";
    }

    struct stat fileStat;
    if (stat("/sdcard/log.txt", &fileStat) != 0) {
        logFile.close();
        return "Failed to get file status.";
    }

    String info = "";
    info += "Log file size: " + String(logFile.size()) + " bytes\n";
    info += "Log file last modified: " + String(ctime(&fileStat.st_mtime)) + "\n";
    info += "Log file created: " + String(ctime(&fileStat.st_ctime)) + "\n";
    info += "Space left of assigned space: " + String(MAX_FILE_SIZE - logFile.size()) + " bytes\n";
    logFile.close();
    return info;
}

bool SDHandler::copyFile(const char* srcPath, const char* destPath) {
    File srcFile = SD.open(srcPath);
    if (!srcFile) {
        Serial.println("Failed to open source file for copying");
        return false;
    }

    File destFile = SD.open(destPath, FILE_WRITE);
    if (!destFile) {
        Serial.println("Failed to open destination file for copying");
        srcFile.close();
        return false;
    }

    while (srcFile.available()) {
        destFile.write(srcFile.read());
    }

    srcFile.close();
    destFile.close();
    return true;
}

bool SDHandler::updateConfig() {
    if (SD.exists("/config.json")) {
        if (!overwriteConfirmed) {
            Serial.println("config.json already exists. Press the button again to overwrite.");
            overwriteConfirmed = true;
            return false;
        } else {
            Serial.println("Overwriting config.json...");
            SD.remove("/config.json");
            if (SDHandler::copyFile("/projectfolder/config.json", "/config.json")) {
                Serial.println("config.json copied to SD card.");
                overwriteConfirmed = false;
                return true;
            } else {
                Serial.println("Failed to copy config.json to SD card.");
                overwriteConfirmed = false;
                return false;
            }
        }
    } else {
        Serial.println("Copying config.json to SD card...");
        if (SDHandler::copyFile("/projectfolder/config.json", "/config.json")) {
            Serial.println("config.json copied to SD card.");
            return true;
        } else {
            Serial.println("Failed to copy config.json to SD card.");
            return false;
        }
    }
}

bool SDHandler::copyDefaultConfig() {
    const char* defaultConfig = R"(
    {
        "wifi": {
            "ssid": "Ron",
            "password": "ikweethet"
        },
        "ethernet": {
            "ip": "192.168.1.100",
            "gateway": "192.168.1.1",
            "subnet": "255.255.255.0",
            "dns": "8.8.8.8"
        },
        "mqtt": {
            "server": "145.220.74.140",
            "port": 1883,
            "username": "mqtt_user",
            "password": "mqtt_password"
        },
        "mqtts": {
            "server": "145.220.74.140",
            "port": 8883,
            "username": "mqtts_user",
            "password": "mqtts_password"
        },
        "project": {
            "device": "Casekeeper",
            "number": "18304",
            "showcase_id": "5.A3"
        },        "Devices": {
            "DS3231": {
                "Type": "RTC",
                "TypeNumber": "DS3231",
                "Address": "0x68",
                "Label": "System",
                "Channels": {
                    "Time": {
                        "Name": "Time",
                        "Threshold": 0.0
                    }
                }
            },
            "SHT_Interior": {
                "Type": "Sensor",
                "TypeNumber": "SHT",
                "Address": "0x44",
                "Label": "Interior",
                "Channels": {
                    "T": {
                        "Name": "Temperature",
                        "Threshold": 0.3
                    },
                    "H": {
                        "Name": "Humidity",
                        "Threshold": 1.0
                    }
                }
            },
            "SHT_Exterior": {
                "Type": "Sensor",
                "TypeNumber": "SHT",
                "Address": "0x44",
                "Label": "Exterior",
                "Channels": {
                    "T": {
                        "Name": "Temperature",
                        "Threshold": 0.3
                    },
                    "H": {
                        "Name": "Humidity",
                        "Threshold": 1.0
                    }
                }
            },            "BH1705": {
                "Type": "Sensor",
                "TypeNumber": "BH1705",
                "Address": "0x23",
                "Label": "Interior",
                "Channels": {
                    "L": {
                        "Name": "Lux",
                        "Threshold": 1.0
                    }
                }
            },
            "SCALE": {
                "Type": "Sensor",
                "TypeNumber": "SCALES",
                "Address": "0x26",
                "Label": "Interior",
                "Channels": {
                    "W": {
                        "Name": "Weight",
                        "Threshold": 1.0
                    }
                }            },
            "GMX02B": {
                "Type": "Sensor",
                "TypeNumber": "GMx02B",
                "Address": "0x08",
                "Label": "Interior",
                "Channels": {
                    "NH3": {
                        "Name": "Ammonia",
                        "Threshold": 1.0
                    },
                    "CO": {
                        "Name": "CarbonOxide",
                        "Threshold": 1.0
                    },
                    "NO2": {
                        "Name": "NitrogenDioxide",
                        "Threshold": 1.0
                    },
                    "C3H8": {
                        "Name": "Propane",
                        "Threshold": 1.0
                    },
                    "C4H10": {
                        "Name": "Butane",
                        "Threshold": 1.0
                    },
                    "CH4": {
                        "Name": "Methane",
                        "Threshold": 1.0
                    },
                    "H2": {
                        "Name": "Hydrogen",
                        "Threshold": 1.0
                    },
                    "C2H5OH": {
                        "Name": "Ethanol",
                        "Threshold": 1.0
                    },
                    "C2H4": {
                        "Name": "Ethylene",
                        "Threshold": 1.0
                    },
                    "VOC": {
                        "Name": "VOC",
                        "Threshold": 1.0
                    }
                }
            },            "PCF8574": {
                "Type": "GPIO",
                "TypeNumber": "PCF8574",
                "Address": "0x20",
                "Label": "Controller",
                "Mode": "OUTPUT",
                "Channels": {
                    "IO0": {
                        "Name": "FanExterior",
                        "Threshold": 1.0
                    },
                    "IO1": {
                        "Name": "FanInterior",
                        "Threshold": 1.0
                    },
                    "IO2": {
                        "Name": "Humidify",
                        "Threshold": 1.0
                    },
                    "IO3": {
                        "Name": "Dehumidify",
                        "Threshold": 1.0
                    },
                    "IO4": {
                        "Name": "TemperatureEnable",
                        "Threshold": 1.0
                    },
                    "IO5": {
                        "Name": "TemperatureCool",
                        "Threshold": 1.0
                    },
                    "IO6": {
                        "Name": "TemperatureHeat",
                        "Threshold": 1.0
                    },
                    "IO7": {
                        "Name": "IO7",
                        "Threshold": 1.0
                    }
                }
            },            "GP8403": {
                "Type": "DAC",
                "TypeNumber": "GP8403",
                "Address": "0x5F",
                "Label": "Controller",
                "Channels": {
                    "DAC_A": {
                        "Name": "TemperaturePower"
                    }
                }
            },
            "LCD_Display": {
                "Type": "Display",
                "TypeNumber": "LCD2x16",
                "Address": "0x27",
                "Label": "Controller",
                "Channels": {
                    "Display": {
                        "Name": "ClimateDisplay",
                        "Threshold": 0.0
                    }
                }
            }
        },
        "timezone": "Europe/Amsterdam",
        "sd_logfile_size": 1048576
    }
    )";

    File configFile = SD.open("/config.json", FILE_WRITE);
    if (!configFile) {
        Serial.println("Failed to open config file for writing");
        return false;
    }

    configFile.print(defaultConfig);
    configFile.close();
    return true;
}

bool SDHandler::readJsonFile(const char* filename, JsonDocument& doc) {
    File file = SD.open(filename);
    if (!file) {
        Serial.print("Failed to open file: ");
        Serial.println(filename);
        return false;
    }

    size_t size = file.size();
    if (size > 8192) {
        Serial.println("Config file size is too large");
        file.close();
        return false;
    }

    String fileContent;
    while (file.available()) {
        fileContent += (char)file.read();
    }
    file.close();

    DeserializationError error = deserializeJson(doc, fileContent);
    if (error) {
        Serial.print("Failed to parse JSON file: ");
        Serial.println(error.c_str());
    return false;
    }

    return true;
}

void SDHandler::printSDCardStatus() {
    Serial.println("\n=== SD Card Status ===");
    
    if (SD.begin()) {
        Serial.println("SD Card: Connected and operational");
        
        // Get card info
        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);
        uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
        
        Serial.print("Card Size: ");
        Serial.print(cardSize);
        Serial.println(" MB");
        
        Serial.print("Total Space: ");
        Serial.print(totalBytes);
        Serial.println(" MB");
        
        Serial.print("Used Space: ");
        Serial.print(usedBytes);
        Serial.println(" MB");
        
        Serial.print("Free Space: ");
        Serial.print(totalBytes - usedBytes);
        Serial.println(" MB");
        
        // Check log file
        if (SD.exists("/sensor_data.json")) {
            File logFile = SD.open("/sensor_data.json", FILE_READ);
            if (logFile) {
                Serial.print("Log File Size: ");
                Serial.print(logFile.size() / 1024);
                Serial.println(" KB");
                logFile.close();
            } else {
                Serial.println("Log File: Unable to read");
            }
        } else {
            Serial.println("Log File: Not found");
        }
        
        // Check config file
        if (SD.exists("/config.json")) {
            Serial.println("Config File: Present");
        } else {
            Serial.println("Config File: Missing");
        }
    } else {
        Serial.println("SD Card: Not connected or initialization failed");
    }
    
    Serial.println("======================");
}
