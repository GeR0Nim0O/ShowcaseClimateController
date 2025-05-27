#ifndef SDHANDLER_H
#define SDHANDLER_H

#include <SD_MMC.h>
#include <FS.h>

const int MAX_FILE_SIZE = 1024 * 10; // Value *1KB limit for the log file

class SDHandler {
public:
    // Function to initialize the SD card
    static bool initSDCard();

    // Function to log JSON data to the SD card
    static void logJson(const char* json);

    // Directory and file operations
    static void listDir(const char* dirname, uint8_t levels);
    static void createDir(const char* path);
    static void removeDir(const char* path);
    static void readFile(const char* path);
    static void writeFile(const char* path, const char* message);
    static void appendFile(const char* path, const char* message);
    static void renameFile(const char* path1, const char* path2);
    static void deleteFile(const char* path);
    static void testFileIO(const char* path);
    static bool checkAndCreateLogFile();
    static String getLogFileInfo();
    static bool copyFile(const char* srcPath, const char* destPath);
    static bool updateConfig();
    static bool copyDefaultConfig();
    
    // New function to initialize SD card and config
    static bool initializeSDCardAndConfig();
};

#endif