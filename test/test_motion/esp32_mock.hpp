#pragma once

// Mock ESP32 functions for native desktop testing
// This provides stubs for timer, GPIO, and FreeRTOS primitives

#include <cstdint>
#include <chrono>
#include <atomic>

// Simulated time - can be advanced manually for testing
inline std::atomic<uint64_t> g_mockMicros{0};

// Mock timer state
inline void (*g_timerCallback)(void* arg) = nullptr;
inline void* g_timerArg = nullptr;
inline uint64_t g_timerPeriod = 0;
inline bool g_timerActive = false;

inline uint32_t micros() {
    return static_cast<uint32_t>(g_mockMicros.load());
}

inline uint64_t micros64() {
    return g_mockMicros.load();
}

inline void setMicros(uint64_t us) {
    g_mockMicros.store(us);
}

inline void resetMock() {
    g_mockMicros.store(0);
    g_timerActive = false;
    g_timerCallback = nullptr;
    g_timerArg = nullptr;
    g_timerPeriod = 0;
}

// Advance time and fire timer callbacks if active
inline void advanceMicros(uint32_t us) {
    if (!g_timerActive || g_timerPeriod == 0) {
        g_mockMicros.fetch_add(us);
        return;
    }

    // Step through time in increments of the period to fire callbacks
    uint64_t current = g_mockMicros.load();
    uint64_t target = current + us;
    
    // Align next interrupt to period boundary relative to 0
    // This isn't strictly necessary for all tests but helps with consistency
    uint64_t nextInterrupt = ((current / g_timerPeriod) + 1) * g_timerPeriod;
    
    while (current < target) {
        if (nextInterrupt <= target) {
            // Jump to next interrupt
            g_mockMicros.store(nextInterrupt);
            current = nextInterrupt;
            
            // Fire callback
            if (g_timerCallback) {
                g_timerCallback(g_timerArg);
            }
            
            nextInterrupt += g_timerPeriod;
        } else {
            // Remaining time is less than a period
            g_mockMicros.store(target);
            current = target;
        }
    }
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

// Mock ESP32 GPIO registers for direct port manipulation
struct GPIO_reg_t {
    uint32_t out_w1ts;
    uint32_t out_w1tc;
    struct {
        uint32_t val;
    } out1_w1ts;
    struct {
        uint32_t val;
    } out1_w1tc;
};

inline GPIO_reg_t GPIO;

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
    g_timerCallback = args->callback;
    g_timerArg = args->arg;
    
    // Return a dummy non-null handle
    static int dummyHandle = 1;
    *handle = (esp_timer_handle_t)&dummyHandle;
    return 0;
}

inline int esp_timer_start_periodic(esp_timer_handle_t handle, uint64_t period_us) {
    (void)handle;
    // printf("[MOCK] Timer started: period=%lu\n", (unsigned long)period_us);
    g_timerPeriod = period_us;
    g_timerActive = true;
    return 0;
}

inline int esp_timer_stop(esp_timer_handle_t handle) {
    (void)handle;
    g_timerActive = false;
    return 0;
}

inline int esp_timer_delete(esp_timer_handle_t handle) {
    (void)handle;
    g_timerActive = false;
    g_timerCallback = nullptr;
    g_timerArg = nullptr;
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
