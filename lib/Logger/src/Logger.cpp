#include "Logger.hpp"

// Global WebLogger instance
WebLogger webLogger;

// WebLogger implementation
WebLogger::WebLogger() {
    m_mutex = xSemaphoreCreateMutex();
}

size_t WebLogger::write(uint8_t c) {
    if (m_mutex == NULL) m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == NULL) return 0;

    xSemaphoreTake(m_mutex, portMAX_DELAY);
    if (m_buffer.length() < MAX_LOG_SIZE) {
        m_buffer += (char)c;
    }
    xSemaphoreGive(m_mutex);
    return 1;
}

size_t WebLogger::write(const uint8_t *buffer, size_t size) {
    if (m_mutex == NULL) m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == NULL) return 0;

    xSemaphoreTake(m_mutex, portMAX_DELAY);
    for (size_t i = 0; i < size && m_buffer.length() < MAX_LOG_SIZE; i++) {
        m_buffer += (char)buffer[i];
    }
    xSemaphoreGive(m_mutex);
    return size;
}

String WebLogger::getNewLogs() {
    if (m_mutex == NULL) m_mutex = xSemaphoreCreateMutex();
    if (m_mutex == NULL) return "";

    xSemaphoreTake(m_mutex, portMAX_DELAY);
    String result = m_buffer;
    m_buffer = "";
    xSemaphoreGive(m_mutex);
    return result;
}
