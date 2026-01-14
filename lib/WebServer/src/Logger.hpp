#pragma once
#include <Arduino.h>
#include <freertos/semphr.h>

// Log buffer for web console
class WebLogger : public Print {
public:
    static constexpr int MAX_LOG_SIZE = 4096;
    static constexpr int MAX_LINES = 100;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    String getNewLogs();  // Get and clear pending logs

private:
    String m_buffer;
    SemaphoreHandle_t m_mutex = NULL;
    void ensureMutex();
};

extern WebLogger webLogger;

// Macro to log to both Serial and web console
#define LOG(fmt, ...) do { \
    Serial.printf(fmt, ##__VA_ARGS__); \
    webLogger.printf(fmt, ##__VA_ARGS__); \
} while(0)
