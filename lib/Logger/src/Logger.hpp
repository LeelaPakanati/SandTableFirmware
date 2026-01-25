#pragma once

#ifdef NATIVE_BUILD
#include <cstdio>

#define LOG(fmt, ...) do { \
    std::printf(fmt, ##__VA_ARGS__); \
} while(0)

#else
#include <Arduino.h>

#define LOG(fmt, ...) do { \
    Serial.printf(fmt, ##__VA_ARGS__); \
} while(0)

#endif
