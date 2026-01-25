#pragma once
#include <stdint.h>
#include <algorithm>
#include <atomic>

class Profiler {
public:
    void reset() {
        m_count = 0;
        m_totalTime = 0;
        m_maxTime = 0;
        m_minTime = UINT32_MAX;
    }

    void addSample(uint32_t timeUs) {
        m_count++;
        m_totalTime += timeUs;
        if (timeUs > m_maxTime) m_maxTime = timeUs;
        if (timeUs < m_minTime) m_minTime = timeUs;
    }

    uint32_t getMax() const { return m_maxTime.load(); }
    uint32_t getMin() const { return (m_count > 0) ? m_minTime.load() : 0; }
    uint32_t getAvg() const { return (m_count > 0) ? (uint32_t)(m_totalTime.load() / m_count.load()) : 0; }
    uint32_t getCount() const { return m_count.load(); }

private:
    std::atomic<uint32_t> m_count{0};
    std::atomic<uint64_t> m_totalTime{0};
    std::atomic<uint32_t> m_maxTime{0};
    std::atomic<uint32_t> m_minTime{UINT32_MAX};
};
