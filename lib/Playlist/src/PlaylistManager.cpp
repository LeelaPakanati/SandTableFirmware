#include "PlaylistManager.hpp"
#include <SDCard.hpp>
#include <ArduinoJson.h>

PlaylistManager::PlaylistManager()
    : m_mode(SEQUENTIAL), m_currentIndex(0), m_clearingEnabled(true),
      m_isFirstPattern(true), m_shuffleIndex(0) {
}

void PlaylistManager::addPattern(String filename) {
  m_playlist.emplace_back(filename);
  if (m_mode == SHUFFLE) {
    reshuffle();
  }
}

void PlaylistManager::removePattern(int index) {
    if (index >= 0 && index < (int)m_playlist.size()) {
        m_playlist.erase(m_playlist.begin() + index);

        // Adjust current index if needed
        if (m_currentIndex >= (int)m_playlist.size() && !m_playlist.empty()) {
            m_currentIndex = 0;
        }
        if (m_mode == SHUFFLE) {
            reshuffle();
        }
    }
}

void PlaylistManager::clear() {
    m_playlist.clear();
    m_currentIndex = 0;
    m_shuffleIndices.clear();
    m_shuffleIndex = 0;
}

void PlaylistManager::setMode(PlaylistMode mode) {
    m_mode = mode;
    if (mode == SHUFFLE) {
        reshuffle();
    }
}

void PlaylistManager::reset() {
    m_currentIndex = 0;
    m_shuffleIndex = 0;
    m_isFirstPattern = true;
    if (m_mode == SHUFFLE) {
        reshuffle();
    }
}

bool PlaylistManager::hasNext() {
    if (m_playlist.empty()) return false;
    
    if (m_mode == SEQUENTIAL) {
        return m_currentIndex < (int)m_playlist.size();
    }
    // LOOP and SHUFFLE always have next if not empty
    return true;
}

NextPatternResult PlaylistManager::getNextPattern() {
  NextPatternResult result;
  result.filename = "";
  result.clearingPattern = CLEARING_NONE;
  result.needsClearing = false;

  if (m_playlist.empty()) return result;

  int index = -1;

  if (m_mode == SEQUENTIAL) {
    if (m_currentIndex >= (int)m_playlist.size()) return result;
    index = m_currentIndex;
    m_currentIndex++;
  }
  else if (m_mode == LOOP) {
    if (m_currentIndex >= (int)m_playlist.size()) m_currentIndex = 0;
    index = m_currentIndex;
    m_currentIndex++;
  }
  else if (m_mode == SHUFFLE) {
    if (m_shuffleIndices.empty()) reshuffle();
    if (m_shuffleIndex >= (int)m_shuffleIndices.size()) {
        reshuffle();
        m_shuffleIndex = 0;
    }
    index = m_shuffleIndices[m_shuffleIndex];
    m_shuffleIndex++;
    m_currentIndex = index; // Sync display index
  }

  if (index >= 0 && index < (int)m_playlist.size()) {
    result.filename = m_playlist[index].filename;

    // Determine if clearing is needed
    if (m_clearingEnabled && !m_isFirstPattern) {
      result.needsClearing = true;
      result.clearingPattern = getRandomClearingPattern();
    }
    m_isFirstPattern = false;
  }

  return result;
}

void PlaylistManager::reshuffle() {
    m_shuffleIndices.clear();
    for (int i = 0; i < (int)m_playlist.size(); i++) {
        m_shuffleIndices.push_back(i);
    }
    
    // Fisher-Yates shuffle
    for (int i = m_shuffleIndices.size() - 1; i > 0; i--) {
        int j = random(0, i + 1);
        int temp = m_shuffleIndices[i];
        m_shuffleIndices[i] = m_shuffleIndices[j];
        m_shuffleIndices[j] = temp;
    }
    m_shuffleIndex = 0;
}

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

    for (const auto& item : m_playlist) {
        JsonObject pattern = patterns.add<JsonObject>();
        pattern["file"] = item.filename;
    }

    // Ensure playlist directory exists
    if (!SD.exists("/playlists")) {
        SD.mkdir("/playlists");
    }

    // Save to file
    String filepath = "/playlists/" + playlistName + ".json";
    
    // Remove existing file to overwrite
    if (SD.exists(filepath)) {
        SD.remove(filepath);
    }

    File file = SD.open(filepath.c_str(), FILE_WRITE);
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

    if (!SD.exists(filepath.c_str())) {
        Serial.println("Playlist file not found: " + filepath);
        return false;
    }

    File file = SD.open(filepath.c_str(), FILE_READ);
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
        String file = pattern["file"].as<String>();
        addPattern(file);
    }

    if (m_mode == SHUFFLE) {
        reshuffle();
    }

    Serial.println("Playlist loaded: " + filepath);
    return true;
}
