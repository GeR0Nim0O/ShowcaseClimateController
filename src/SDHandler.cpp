#include "SDHandler.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

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
        Serial.println("WARNING: SD card initialization failed - SD card may not be present");
        return false;
    }
    Serial.println("SD card initialized.");
    return true;
}

bool SDHandler::checkAndCreateLogFile() {
    if (!SD.exists("/log.txt")) {
        logFile = SD.open("/log.txt", FILE_WRITE);
        if (!logFile) {
            Serial.println("WARNING: Failed to create log file on SD card");
            return false;
        }
        logFile.close();
        Serial.println("Log file created");
    }
    return true;
}

bool SDHandler::initializeSDCardAndConfig() {
    if (!initSDCard()) {
        Serial.println("WARNING: SD Card initialization failed - continuing without SD card support");
        return false;
    }

    if (!checkAndCreateLogFile()) {
        Serial.println("WARNING: Failed to ensure log file exists - SD logging will not be available");
        return false;
    } else {
        Serial.println("Log file is present and ready for use.");
        Serial.println(getLogFileInfo());
    }    if (!SD.exists("/config.json")) {
        Serial.println("ERROR: Config file does not exist on SD card!");
        Serial.println("No hardcoded fallback configuration available.");
        Serial.println("Please place a valid config.json file on the SD card or upload it to SPIFFS.");
        return false;
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

bool SDHandler::forceUpdateSDConfig() {
    Serial.println("Force updating SD card configuration...");
    
    // Remove existing config file if it exists
    if (SD.exists("/config.json")) {
        SD.remove("/config.json");
        Serial.println("Removed existing config.json from SD card");
    }
    
    // Copy project config.json from SPIFFS to SD card
    if (!SPIFFS.begin()) {
        Serial.println("Failed to mount SPIFFS");
        return false;
    }
    
    File spiffsFile = SPIFFS.open("/config.json", "r");
    if (!spiffsFile) {
        Serial.println("Failed to open project config.json from SPIFFS");
        return false;
    }
    
    File sdFile = SD.open("/config.json", FILE_WRITE);
    if (!sdFile) {
        Serial.println("Failed to create config.json on SD card");
        spiffsFile.close();
        return false;
    }
    
    // Copy content from SPIFFS to SD card
    while (spiffsFile.available()) {
        sdFile.write(spiffsFile.read());
    }
    
    spiffsFile.close();
    sdFile.close();
    
    Serial.println("Successfully updated SD card config.json with project configuration");
    return true;
}

bool SDHandler::copyDefaultConfig() {
    // NO HARDCODED CONFIGURATION - Use proper config files only
    Serial.println("ERROR: No hardcoded fallback configuration available");
    Serial.println("Please ensure a valid config.json file exists in the data folder");
    Serial.println("and upload it to SPIFFS or place it on the SD card");
    return false;
}

bool SDHandler::readJsonFile(const char* filename, JsonDocument& doc) {
    // Check if SD card is available
    if (!SD.cardSize()) {
        Serial.print("WARNING: SD card not available - cannot read file: ");
        Serial.println(filename);
        return false;
    }
    
    File file = SD.open(filename);
    if (!file) {
        Serial.print("WARNING: Failed to open file on SD card: ");
        Serial.println(filename);
        return false;
    }

    size_t size = file.size();
    if (size > 8192) {
        Serial.println("WARNING: Config file size is too large");
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
        Serial.print("WARNING: Failed to parse JSON file: ");
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
