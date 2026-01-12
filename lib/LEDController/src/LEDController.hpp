#pragma once
#include <Arduino.h>

class LEDController {
public:
    LEDController(uint8_t pin = 18);
    void begin();
    void setBrightness(uint8_t brightness); // 0-255
    uint8_t getBrightness();
    void on();
    void off();

private:
    uint8_t m_pin;
    uint8_t m_brightness;
    static constexpr int PWM_CHANNEL = 0;
    static constexpr int PWM_FREQ = 5000;
    static constexpr int PWM_RESOLUTION = 8; // 8-bit = 0-255
};
