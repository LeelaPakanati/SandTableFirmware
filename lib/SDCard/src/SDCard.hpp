#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// SD Card SPI pins
#define SD_CS_PIN   5
#define SD_MOSI_PIN 23
#define SD_CLK_PIN  18
#define SD_MISO_PIN 19

// Initialize SD card - call once in setup()
inline bool initSDCard() {
    // Explicitly initialize SPI with defined pins
    SPI.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    // Initialize SD with CS pin, SPI instance, and high frequency (20MHz)
    if (!SD.begin(SD_CS_PIN, SPI, 20000000)) {
        Serial.println("ERROR: SD card mount failed!");
        return false;
    }

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return false;
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) Serial.println("MMC");
    else if (cardType == CARD_SD) Serial.println("SDSC");
    else if (cardType == CARD_SDHC) Serial.println("SDHC");
    else Serial.println("UNKNOWN");

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %llu MB\n", cardSize);

    // Ensure directory structure exists
    if (!SD.exists("/patterns")) {
        Serial.println("Creating /patterns directory");
        SD.mkdir("/patterns");
    }
    if (!SD.exists("/playlists")) {
        Serial.println("Creating /playlists directory");
        SD.mkdir("/playlists");
    }

    return true;
}

// Helper to list files on SD card
inline void listSDFiles(const char* dirname = "/") {
    Serial.printf("\n=== Files on SD Card (%s) ===\n", dirname);

    File root = SD.open(dirname);
    if (!root) {
        Serial.println("ERROR: Failed to open directory");
        return;
    }

    if (!root.isDirectory()) {
        Serial.println("ERROR: Not a directory");
        return;
    }

    File file = root.openNextFile();
    int count = 0;
    while (file) {
        if (!file.isDirectory()) {
            Serial.printf("  %s (%lu bytes)\n", file.name(), (unsigned long)file.size());
            count++;
        }
        file.close();
        file = root.openNextFile();
    }
    
    Serial.printf("Total files: %d\n\n", count);
}

// Recursively delete a directory and all its contents
inline bool removeDirectoryRecursive(const char* path) {
    File dir = SD.open(path);
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return false;
    }

    File entry = dir.openNextFile();
    while (entry) {
        String entryPath = String(path) + "/" + entry.name();
        if (entry.isDirectory()) {
            entry.close();
            removeDirectoryRecursive(entryPath.c_str());
        } else {
            entry.close();
            SD.remove(entryPath.c_str());
        }
        entry = dir.openNextFile();
    }
    dir.close();
    return SD.rmdir(path);
}
