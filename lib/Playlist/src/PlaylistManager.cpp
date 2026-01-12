#include "PlaylistManager.hpp"
#include <LittleFS.h>
#include <ArduinoJson.h>

PlaylistManager::PlaylistManager()
    : m_mode(SEQUENTIAL), m_currentIndex(0) {
}

void PlaylistManager::addPattern(String filename, ClearingPattern clearing, bool useClearing) {
    m_items.push_back(PlaylistItem(filename, clearing, useClearing));
}

void PlaylistManager::removePattern(int index) {
    if (index >= 0 && index < m_items.size()) {
        m_items.erase(m_items.begin() + index);

        // Adjust current index if needed
        if (m_currentIndex >= m_items.size() && m_items.size() > 0) {
            m_currentIndex = 0;
        }
    }
}

void PlaylistManager::clear() {
    m_items.clear();
    m_currentIndex = 0;
    m_shuffleOrder.clear();
}

void PlaylistManager::movePattern(int from, int to) {
    if (from >= 0 && from < m_items.size() && to >= 0 && to < m_items.size()) {
        PlaylistItem item = m_items[from];
        m_items.erase(m_items.begin() + from);
        m_items.insert(m_items.begin() + to, item);
    }
}

void PlaylistManager::setMode(PlaylistMode mode) {
    m_mode = mode;
    if (mode == SHUFFLE && m_shuffleOrder.empty()) {
        generateShuffleOrder();
    }
}

String PlaylistManager::getNextPattern(ClearingPattern& outClearing, bool& outUseClearing) {
    if (m_items.empty()) {
        return "";
    }

    int index = getNextIndex();
    if (index < 0) {
        return ""; // No more patterns in SEQUENTIAL mode
    }

    const PlaylistItem& item = m_items[index];

    // Always use random clearing pattern between playlist items
    outClearing = getRandomClearing();
    outUseClearing = item.useClearing;

    return item.filename;
}

bool PlaylistManager::hasNext() const {
    if (m_items.empty()) {
        return false;
    }

    switch (m_mode) {
        case SEQUENTIAL:
            return m_currentIndex < m_items.size();
        case LOOP:
            return true; // Always has next in loop mode
        case SHUFFLE:
            return !m_shuffleOrder.empty() || m_currentIndex < m_items.size();
        default:
            return false;
    }
}

void PlaylistManager::reset() {
    m_currentIndex = 0;
    if (m_mode == SHUFFLE) {
        generateShuffleOrder();
    }
}

void PlaylistManager::shuffle() {
    setMode(SHUFFLE);
    generateShuffleOrder();
}

const PlaylistItem& PlaylistManager::getItem(int index) const {
    static PlaylistItem empty;
    if (index >= 0 && index < m_items.size()) {
        return m_items[index];
    }
    return empty;
}

// Private methods

void PlaylistManager::generateShuffleOrder() {
    m_shuffleOrder.clear();

    // Create sequential order
    for (int i = 0; i < m_items.size(); i++) {
        m_shuffleOrder.push_back(i);
    }

    // Fisher-Yates shuffle algorithm
    for (int i = m_shuffleOrder.size() - 1; i > 0; i--) {
        int j = random(0, i + 1);
        int temp = m_shuffleOrder[i];
        m_shuffleOrder[i] = m_shuffleOrder[j];
        m_shuffleOrder[j] = temp;
    }

    m_currentIndex = 0;
}

int PlaylistManager::getNextIndex() {
    if (m_items.empty()) {
        return -1;
    }

    int index;

    switch (m_mode) {
        case SEQUENTIAL:
            if (m_currentIndex >= m_items.size()) {
                return -1; // End of playlist
            }
            index = m_currentIndex;
            m_currentIndex++;
            return index;

        case LOOP:
            index = m_currentIndex;
            m_currentIndex = (m_currentIndex + 1) % m_items.size();
            return index;

        case SHUFFLE:
            if (m_shuffleOrder.empty()) {
                generateShuffleOrder();
            }

            if (m_currentIndex >= m_shuffleOrder.size()) {
                // Re-shuffle for next round in loop mode
                generateShuffleOrder();
            }

            if (m_currentIndex < m_shuffleOrder.size()) {
                index = m_shuffleOrder[m_currentIndex];
                m_currentIndex++;
                return index;
            }

            return -1;

        default:
            return -1;
    }
}

// Persistence

bool PlaylistManager::saveToFile(String playlistName) {
    JsonDocument doc;

    doc["name"] = playlistName;

    switch (m_mode) {
        case SEQUENTIAL:
            doc["mode"] = "sequential";
            break;
        case LOOP:
            doc["mode"] = "loop";
            break;
        case SHUFFLE:
            doc["mode"] = "shuffle";
            break;
    }

    JsonArray patterns = doc["patterns"].to<JsonArray>();

    for (const auto& item : m_items) {
        JsonObject pattern = patterns.add<JsonObject>();
        pattern["file"] = item.filename;

        // Convert ClearingPattern enum to string
        switch (item.clearing) {
            case SPIRAL_OUTWARD:
                pattern["clearing"] = "spiral_outward";
                break;
            case SPIRAL_INWARD:
                pattern["clearing"] = "spiral_inward";
                break;
            case CONCENTRIC_CIRCLES:
                pattern["clearing"] = "concentric_circles";
                break;
            case ZIGZAG_RADIAL:
                pattern["clearing"] = "zigzag_radial";
                break;
            case PETAL_FLOWER:
                pattern["clearing"] = "petal_flower";
                break;
        }

        pattern["useClearing"] = item.useClearing;
    }

    // Ensure playlist directory exists
    if (!LittleFS.exists("/playlists")) {
        LittleFS.mkdir("/playlists");
    }

    // Save to file
    String filepath = "/playlists/" + playlistName + ".json";
    File file = LittleFS.open(filepath, "w");

    if (!file) {
        Serial.println("Failed to create playlist file");
        return false;
    }

    if (serializeJson(doc, file) == 0) {
        Serial.println("Failed to write playlist");
        file.close();
        return false;
    }

    file.close();
    Serial.println("Playlist saved: " + filepath);
    return true;
}

bool PlaylistManager::loadFromFile(String playlistName) {
    String filepath = "/playlists/" + playlistName + ".json";

    if (!LittleFS.exists(filepath)) {
        Serial.println("Playlist file not found: " + filepath);
        return false;
    }

    File file = LittleFS.open(filepath, "r");
    if (!file) {
        Serial.println("Failed to open playlist file");
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.println("Failed to parse playlist JSON");
        return false;
    }

    // Clear existing playlist
    clear();

    // Load mode
    String modeStr = doc["mode"] | "sequential";
    if (modeStr == "loop") {
        m_mode = LOOP;
    } else if (modeStr == "shuffle") {
        m_mode = SHUFFLE;
    } else {
        m_mode = SEQUENTIAL;
    }

    // Load patterns
    JsonArray patterns = doc["patterns"];
    for (JsonObject pattern : patterns) {
        String filename = pattern["file"] | "";
        String clearingStr = pattern["clearing"] | "spiral_outward";
        bool useClearing = pattern["useClearing"] | true;

        // Convert string to ClearingPattern enum
        ClearingPattern clearing = SPIRAL_OUTWARD;
        if (clearingStr == "spiral_inward") {
            clearing = SPIRAL_INWARD;
        } else if (clearingStr == "concentric_circles") {
            clearing = CONCENTRIC_CIRCLES;
        } else if (clearingStr == "zigzag_radial") {
            clearing = ZIGZAG_RADIAL;
        } else if (clearingStr == "petal_flower") {
            clearing = PETAL_FLOWER;
        }

        addPattern(filename, clearing, useClearing);
    }

    if (m_mode == SHUFFLE) {
        generateShuffleOrder();
    }

    Serial.println("Playlist loaded: " + filepath);
    return true;
}

ClearingPattern PlaylistManager::getRandomClearing() {
    // Randomly select one of the 5 clearing patterns
    int randomIndex = random(0, 5);

    switch (randomIndex) {
        case 0: return SPIRAL_OUTWARD;
        case 1: return SPIRAL_INWARD;
        case 2: return CONCENTRIC_CIRCLES;
        case 3: return ZIGZAG_RADIAL;
        case 4: return PETAL_FLOWER;
        default: return SPIRAL_OUTWARD;
    }
}
