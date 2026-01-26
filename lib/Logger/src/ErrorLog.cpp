#include "ErrorLog.hpp"
#include <cstring>

#ifdef NATIVE_BUILD
static uint32_t nowMs() {
    return 0;
}
#else
static uint32_t nowMs() {
    return millis();
}
#endif

ErrorLog& ErrorLog::instance() {
    static ErrorLog log;
    return log;
}

void ErrorLog::copyField(char* dest, size_t len, const char* src) {
    if (!dest || len == 0) return;
    if (!src) {
        dest[0] = '\0';
        return;
    }
    strncpy(dest, src, len - 1);
    dest[len - 1] = '\0';
}

void ErrorLog::log(const char* level, const char* category, const char* code,
                   const char* message, const char* context) {
    lock();
    if (m_size == kMaxEntries) {
        m_dropped++;
    } else {
        m_size++;
    }

    Entry& entry = m_entries[m_head];
    entry.id = m_nextId++;
    entry.tsMs = nowMs();
    copyField(entry.level, sizeof(entry.level), level);
    copyField(entry.category, sizeof(entry.category), category);
    copyField(entry.code, sizeof(entry.code), code);
    copyField(entry.message, sizeof(entry.message), message);
    copyField(entry.context, sizeof(entry.context), context);

    m_head = (m_head + 1) % kMaxEntries;
    m_total++;
    unlock();
}

uint32_t ErrorLog::totalCount() const {
    lock();
    uint32_t total = m_total;
    unlock();
    return total;
}

uint32_t ErrorLog::droppedCount() const {
    lock();
    uint32_t dropped = m_dropped;
    unlock();
    return dropped;
}

uint32_t ErrorLog::size() const {
    lock();
    uint32_t size = m_size;
    unlock();
    return size;
}

#ifdef NATIVE_BUILD
void ErrorLog::writeJsonString(Print& out, const char* value) const {
    (void)out;
    (void)value;
}
#else
void ErrorLog::writeJsonString(Print& out, const char* value) const {
    out.print('"');
    if (value) {
        for (const char* p = value; *p; ++p) {
            char c = *p;
            if (c == '"' || c == '\\') {
                out.print('\\');
                out.print(c);
            } else if (c == '\n') {
                out.print("\\n");
            } else if (c == '\r') {
                out.print("\\r");
            } else if (c == '\t') {
                out.print("\\t");
            } else {
                out.print(c);
            }
        }
    }
    out.print('"');
}
#endif

#ifdef NATIVE_BUILD
void ErrorLog::writeJson(Print& out) const {
    (void)out;
}
#else
void ErrorLog::writeJson(Print& out) const {
    Entry entries[kMaxEntries];
    size_t size = 0;
    uint32_t total = 0;
    uint32_t dropped = 0;

    lock();
    size = m_size;
    total = m_total;
    dropped = m_dropped;
    size_t start = (m_head + kMaxEntries - m_size) % kMaxEntries;
    for (size_t i = 0; i < m_size; ++i) {
        size_t idx = (start + i) % kMaxEntries;
        entries[i] = m_entries[idx];
    }
    unlock();

    out.print("{\"count\":");
    out.print(static_cast<uint32_t>(size));
    out.print(",\"total\":");
    out.print(total);
    out.print(",\"dropped\":");
    out.print(dropped);
    out.print(",\"errors\":[");

    for (size_t i = 0; i < size; ++i) {
        const Entry& entry = entries[i];
        if (i > 0) out.print(',');
        out.print("{\"id\":");
        out.print(entry.id);
        out.print(",\"tsMs\":");
        out.print(entry.tsMs);
        out.print(",\"level\":");
        writeJsonString(out, entry.level);
        out.print(",\"category\":");
        writeJsonString(out, entry.category);
        out.print(",\"code\":");
        writeJsonString(out, entry.code);
        out.print(",\"message\":");
        writeJsonString(out, entry.message);
        out.print(",\"context\":");
        writeJsonString(out, entry.context);
        out.print('}');
    }
    out.print("]}");
}
#endif
