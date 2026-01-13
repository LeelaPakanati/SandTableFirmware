#pragma once
#include <SdFat.h>
#include <SPI.h>

// SD Card SPI pins
#define SD_CS_PIN   5
#define SD_MOSI_PIN 23
#define SD_CLK_PIN  18
#define SD_MISO_PIN 19

// Global SD card instance
extern SdFat SD;

// Initialize SD card - call once in setup()
inline bool initSDCard() {
    SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    if (!SD.begin(SD_CS_PIN, SD_SCK_MHZ(25))) {
        Serial.println("ERROR: SD card mount failed!");
        return false;
    }

    Serial.println("SD card mounted successfully");

    // Print card info
    uint64_t cardSize = SD.card()->sectorCount() * 512ULL;
    Serial.printf("SD Card Size: %llu MB\n", cardSize / (1024 * 1024));

    return true;
}

// Helper to list files on SD card
inline void listSDFiles(const char* dirname = "/") {
    Serial.printf("\n=== Files on SD Card (%s) ===\n", dirname);

    FsFile root;
    if (!root.open(dirname)) {
        Serial.println("ERROR: Failed to open directory");
        return;
    }

    if (!root.isDirectory()) {
        Serial.println("ERROR: Not a directory");
        root.close();
        return;
    }

    FsFile file;
    int count = 0;
    while (file.openNext(&root, O_RDONLY)) {
        if (!file.isDirectory()) {
            char name[64];
            file.getName(name, sizeof(name));
            Serial.printf("  %s (%lu bytes)\n", name, (unsigned long)file.fileSize());
            count++;
        }
        file.close();
    }
    root.close();

    Serial.printf("Total files: %d\n\n", count);
}
