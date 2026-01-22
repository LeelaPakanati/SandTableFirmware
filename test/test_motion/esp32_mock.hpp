#pragma once

// Mock ESP32 functions for native desktop testing
// This provides stubs for timer, GPIO, and FreeRTOS primitives

#include <cstdint>
#include <chrono>
#include <atomic>

// Simulated time - can be advanced manually for testing
static std::atomic<uint64_t> g_mockMicros{0};

inline uint32_t micros() {
    return static_cast<uint32_t>(g_mockMicros.load());
}

inline uint64_t micros64() {
    return g_mockMicros.load();
}

inline void advanceMicros(uint32_t us) {
    g_mockMicros.fetch_add(us);
}

inline void setMicros(uint64_t us) {
    g_mockMicros.store(us);
}

// Use real time for wall-clock timing
inline uint64_t realMicros() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

// GPIO stubs
#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0

inline void pinMode(int pin, int mode) {
    (void)pin; (void)mode;
}

inline void digitalWrite(int pin, int value) {
    (void)pin; (void)value;
}

inline int digitalRead(int pin) {
    (void)pin;
    return LOW;
}

// ESP timer stubs
typedef void* esp_timer_handle_t;

struct esp_timer_create_args_t {
    void (*callback)(void* arg);
    void* arg;
    int dispatch_method;
    const char* name;
    bool skip_unhandled_events;
};

#define ESP_TIMER_TASK 0

inline int esp_timer_create(const esp_timer_create_args_t* args, esp_timer_handle_t* handle) {
    (void)args; (void)handle;
    *handle = nullptr;
    return 0;
}

inline int esp_timer_start_periodic(esp_timer_handle_t handle, uint64_t period_us) {
    (void)handle; (void)period_us;
    return 0;
}

inline int esp_timer_stop(esp_timer_handle_t handle) {
    (void)handle;
    return 0;
}

inline int esp_timer_delete(esp_timer_handle_t handle) {
    (void)handle;
    return 0;
}

// FreeRTOS stubs (minimal)
typedef void* SemaphoreHandle_t;

inline SemaphoreHandle_t xSemaphoreCreateMutex() {
    return nullptr;
}

inline int xSemaphoreTake(SemaphoreHandle_t sem, int timeout) {
    (void)sem; (void)timeout;
    return 1;  // pdTRUE
}

inline int xSemaphoreGive(SemaphoreHandle_t sem) {
    (void)sem;
    return 1;  // pdTRUE
}

#define portMAX_DELAY 0xFFFFFFFF
