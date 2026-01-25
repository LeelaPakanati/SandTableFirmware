#include "LEDController.hpp"
#include "Logger.hpp"

LEDController::LEDController(uint8_t pin) : m_pin(pin), m_brightness(128) {
}

void LEDController::begin() {
    // Configure LEDC peripheral for PWM control
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(m_pin, PWM_CHANNEL);

    // Set initial brightness
    ledcWrite(PWM_CHANNEL, m_brightness);

    LOG("LED Controller initialized on GPIO %d with brightness %d\r\n", m_pin, m_brightness);
}

void LEDController::setBrightness(uint8_t brightness) {
    m_brightness = brightness;
    ledcWrite(PWM_CHANNEL, m_brightness);
}

uint8_t LEDController::getBrightness() {
    return m_brightness;
}

void LEDController::on() {
    setBrightness(255);
}

void LEDController::off() {
    setBrightness(0);
}
