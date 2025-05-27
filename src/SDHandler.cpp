#include "SDHandler.h"
#include <sys/stat.h>

// ...existing code...

File logFile;
bool overwriteConfirmed = false;

bool SDHandler::initSDCard() {
    if (!SD_MMC.begin("/sdcard", true)) {  // Adjust the path based on your setup
        Serial.println("SD card initialization failed!");
        return false;
    }
    Serial.println("SD card initialized.");
    return true;
}

bool SDHandler::checkAndCreateLogFile() {
    if (!SD_MMC.exists("/log.txt")) {
        logFile = SD_MMC.open("/log.txt", FILE_WRITE);
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

    if (!SD_MMC.exists("/config.json")) {
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
    // Check if the SD card is mounted
    if (!SD_MMC.cardType()) {
        Serial.println("SD card not present.");
        return;
    }

    // Open the file for appending
    logFile = SD_MMC.open("/log.txt", FILE_APPEND);
    if (!logFile) {
        Serial.println("Error opening log file");
        return;
    }

    // Check file size and overwrite if necessary
    if (logFile.size() > MAX_FILE_SIZE) {
        logFile.close();
        // Read the entire file into memory
        File tempFile = SD_MMC.open("/log.txt", FILE_READ);
        String fileContent = "";
        while (tempFile.available()) {
            fileContent += char(tempFile.read());
        }
        tempFile.close();

        // Find the first newline character after the first log entry
        int firstNewline = fileContent.indexOf('\n');
        if (firstNewline != -1) {
            fileContent = fileContent.substring(firstNewline + 1); // Remove the oldest log entry
        }

        // Write the remaining content back to the file
        SD_MMC.remove("/log.txt");
        logFile = SD_MMC.open("/log.txt", FILE_WRITE);
        logFile.print(fileContent);
    }

    // Write the JSON string to the log file
    logFile.println(json);
    logFile.flush(); // Ensures data is written to the SD card
    Serial.println("Logged to SD: " + String(json));
}

// Listing directory contents
void SDHandler::listDir(const char* dirname, uint8_t levels) {
    Serial.printf("Listing directory: %s\n", dirname);

    File root = SD_MMC.open(dirname);
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
    if (SD_MMC.mkdir(path)) {
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

// Remove directory
void SDHandler::removeDir(const char* path) {
    Serial.printf("Removing Dir: %s\n", path);
    if (SD_MMC.rmdir(path)) {
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

// Read file contents
void SDHandler::readFile(const char* path) {
    Serial.printf("Reading file: %s\n", path);

    File file = SD_MMC.open(path);
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

    File file = SD_MMC.open(path, FILE_WRITE);
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

    File file = SD_MMC.open(path, FILE_APPEND);
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
    if (SD_MMC.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

// Delete a file
void SDHandler::deleteFile(const char* path) {
    Serial.printf("Deleting file: %s\n", path);
    if (SD_MMC.remove(path)) {
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

// Test file I/O
void SDHandler::testFileIO(const char* path) {
    File file = SD_MMC.open(path);
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

    file = SD_MMC.open(path, FILE_WRITE);
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
    File logFile = SD_MMC.open("/log.txt");
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
    File srcFile = SD_MMC.open(srcPath);
    if (!srcFile) {
        Serial.println("Failed to open source file for copying");
        return false;
    }

    File destFile = SD_MMC.open(destPath, FILE_WRITE);
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
    if (SD_MMC.exists("/config.json")) {
        if (!overwriteConfirmed) {
            Serial.println("config.json already exists. Press the button again to overwrite.");
            overwriteConfirmed = true;
            return false;
        } else {
            Serial.println("Overwriting config.json...");
            SD_MMC.remove("/config.json");
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
        },
        "Devices": {
            "DS3231:": {
                "Type": "RTC",
                "TypeNumber": "DS3231",
                "Address": "0x68",
                "Channels": {
                    "Time": "Time"
                }
            },
            "SHT31": {
                "Type": "Sensor",
                "TypeNumber": "SHT31",
                "Address": "0x44",
                "Threshold": "0.3",
                "Channels": {
                    "Temperature": "T",
                    "Humidity": "H"
                }
            },
            "BH1705": {
                "Type": "Sensor",
                "TypeNumber": "BH1705",
                "Address": "0x23",
                "Threshold": "1",
                "Channels": {
                    "Lux": "L"
                }
            },
            "SCALE": {
                "Type": "Sensor",
                "TypeNumber": "SCALES",
                "Address": "0x26",
                "Threshold": "1",
                "Channels": {
                    "Weight": "W"
                }
            },
            "GMX02B": {
                "Type": "Sensor",
                "TypeNumber": "GMx02B",
                "Address": "0x08",
                "Threshold": "1",
                "Channels": {
                    "Ammonia": "NH3",
                    "CarbonOxide": "CO",
                    "NitrogenDioxide": "NO2",
                    "Propane": "C3H8",
                    "Butane": "C4H10",
                    "Methane": "CH4",
                    "Hydrogen": "H2",
                    "Ethanol": "C2H5OH",
                    "Ethylene": "C2H4",
                    "VOC": "VOC"
                }
            },
            "PCF8574": {
                "Type": "GPIO",
                "TypeNumber": "PCF8574",
                "Address": "0x20",
                "Threshold": "1",
                "Channels": {
                    "IO0": "IO0",
                    "IO1": "IO1",
                    "IO2": "IO2",
                    "IO3": "IO3",
                    "IO4": "IO4",
                    "IO5": "IO5",
                    "IO6": "IO6",
                    "IO7": "IO7"
                }
            }
        },
        "timezone": "Europe/Amsterdam",
        "sd_logfile_size": 1048576
    }
    )";

    File configFile = SD_MMC.open("/config.json", FILE_WRITE);
    if (!configFile) {
        Serial.println("Failed to open config file for writing");
        return false;
    }

    configFile.print(defaultConfig);
    configFile.close();
    return true;
}
