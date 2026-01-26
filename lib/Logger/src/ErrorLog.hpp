#pragma once
#ifdef NATIVE_BUILD
#include <cstddef>
#include <cstdint>
#include <cstring>
class Print;
#else
#include <Arduino.h>
#include <Print.h>
#endif

class ErrorLog {
public:
    struct Entry {
        uint32_t id;
        uint32_t tsMs;
        char level[6];
        char category[10];
        char code[16];
        char message[64];
        char context[64];
    };

    static ErrorLog& instance();
    void log(const char* level, const char* category, const char* code,
             const char* message, const char* context = nullptr);
    uint32_t totalCount() const;
    uint32_t droppedCount() const;
    uint32_t size() const;
    void writeJson(Print& out) const;

private:
    static constexpr size_t kMaxEntries = 8;

    ErrorLog() = default;
    void copyField(char* dest, size_t len, const char* src);
    void writeJsonString(Print& out, const char* value) const;

#ifdef NATIVE_BUILD
    void lock() const {}
    void unlock() const {}
#else
    mutable portMUX_TYPE m_mutex = portMUX_INITIALIZER_UNLOCKED;
    void lock() const { portENTER_CRITICAL(&m_mutex); }
    void unlock() const { portEXIT_CRITICAL(&m_mutex); }
#endif

    Entry m_entries[kMaxEntries];
    size_t m_head = 0;
    size_t m_size = 0;
    uint32_t m_nextId = 1;
    uint32_t m_total = 0;
    uint32_t m_dropped = 0;
};
