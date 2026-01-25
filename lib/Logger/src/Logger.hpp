#pragma once

#ifdef NATIVE_BUILD
#include <cstddef>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <mutex>

using String = std::string;

class Print {
public:
    virtual ~Print() = default;
    virtual size_t write(uint8_t c) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;

    size_t printf(const char *format, ...) {
        char buffer[512];
        va_list args;
        va_start(args, format);
        int len = vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        if (len < 0) {
            return 0;
        }
        size_t outLen = static_cast<size_t>(len);
        if (outLen > sizeof(buffer)) {
            outLen = sizeof(buffer);
        }
        return write(reinterpret_cast<const uint8_t *>(buffer), outLen);
    }
};
#else
#include <Arduino.h>
#include <freertos/semphr.h>
#endif

// Log buffer for web console
class WebLogger : public Print {
public:
    static constexpr int MAX_LOG_SIZE = 4096;
    static constexpr int MAX_LINES = 100;

    WebLogger();

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    String getNewLogs();  // Get and clear pending logs

private:
    String m_buffer;
#ifdef NATIVE_BUILD
    std::mutex m_mutex;
#else
    SemaphoreHandle_t m_mutex = NULL;
#endif
};

extern WebLogger webLogger;

// Macro to log to both Serial and web console
#ifdef NATIVE_BUILD
#define LOG(fmt, ...) do { \
    std::printf(fmt, ##__VA_ARGS__); \
} while(0)
#else
#define LOG(fmt, ...) do { \
    Serial.printf(fmt, ##__VA_ARGS__); \
    webLogger.printf(fmt, ##__VA_ARGS__); \
} while(0)
#endif
