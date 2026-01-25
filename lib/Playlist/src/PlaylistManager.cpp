#include "PlaylistManager.hpp"
#include <algorithm>
#include <random>

PlaylistManager::PlaylistManager()
  : m_loop(false),
    m_currentIndex(-1),
    m_clearingEnabled(true),
    m_isFirstPattern(true)
{
}

void PlaylistManager::addPattern(String filename) {
  m_playlist.push_back(PlaylistItem(filename));
}

void PlaylistManager::removePattern(int index) {
  if (index >= 0 && index < m_playlist.size()) {
    m_playlist.erase(m_playlist.begin() + index);
    if (index <= m_currentIndex) {
      m_currentIndex--;
    }
  }
}

void PlaylistManager::clear() {
  m_playlist.clear();
  m_currentIndex = -1;
  m_isFirstPattern = true;
}

void PlaylistManager::movePattern(int fromIndex, int toIndex) {
  if (fromIndex < 0 || fromIndex >= m_playlist.size() || 
      toIndex < 0 || toIndex >= m_playlist.size() || fromIndex == toIndex) {
    return;
  }

  PlaylistItem item = m_playlist[fromIndex];
  m_playlist.erase(m_playlist.begin() + fromIndex);
  m_playlist.insert(m_playlist.begin() + toIndex, item);

  // Adjust current index if needed
  if (m_currentIndex == fromIndex) {
    m_currentIndex = toIndex;
  } else if (fromIndex < m_currentIndex && toIndex >= m_currentIndex) {
    m_currentIndex--;
  } else if (fromIndex > m_currentIndex && toIndex <= m_currentIndex) {
    m_currentIndex++;
  }
}

void PlaylistManager::shuffle() {
  if (m_playlist.empty()) return;

  // Simple Fisher-Yates shuffle using Arduino's random
  for (int i = m_playlist.size() - 1; i > 0; i--) {
    int j = random(i + 1);
    std::swap(m_playlist[i], m_playlist[j]);
  }
  
  // If we were playing something, try to track it? 
  // No, shuffle resets the order. We might lose track of "current".
  // Let's reset index to 0 or -1?
  // User expects shuffle to reorder the *queue*. 
  // If playing, maybe keep current item at current index?
  // Too complex. Let's just shuffle everything.
  // Ideally, if playing, current item stays playing.
  // But changing order affects "next".
  
  // Let's just reset playlist state effectively
  m_currentIndex = -1; 
}

void PlaylistManager::reset() {
  m_currentIndex = -1;
  m_isFirstPattern = true;
}

bool PlaylistManager::hasNext() {
  if (m_playlist.empty()) return false;
  if (m_loop) return true;
  return m_currentIndex < (int)m_playlist.size() - 1;
}

NextPatternResult PlaylistManager::getNextPattern() {
  NextPatternResult result;
  result.needsClearing = false;
  result.clearingPattern = CLEARING_NONE;

  if (m_playlist.empty()) {
    return result;
  }

  m_currentIndex++;
  if (m_currentIndex >= m_playlist.size()) {
    if (m_loop) {
      m_currentIndex = 0;
    } else {
      // End of playlist
      m_currentIndex = m_playlist.size(); // Keep at end
      return result;
    }
  }

  const PlaylistItem& item = m_playlist[m_currentIndex];
  result.filename = item.filename;

  // Determine clearing
  if (m_clearingEnabled && !m_isFirstPattern) {
    result.needsClearing = true;
    result.clearingPattern = CLEARING_RANDOM; 
  }

  m_isFirstPattern = false;
  return result;
}

void PlaylistManager::setCurrentIndex(int index) {
  if (index >= -1 && index < (int)m_playlist.size()) {
    m_currentIndex = index - 1; // Set to previous so getNextPattern() returns index
    m_isFirstPattern = true; // Skip clearing if jumping manually
  }
}

void PlaylistManager::skipToIndex(int index) {
  setCurrentIndex(index);
}

void PlaylistManager::skipNext() {
  // getNextPattern advances index automatically
}

void PlaylistManager::skipPrevious() {
  if (m_playlist.empty()) return;
  
  m_currentIndex -= 2; // Go back 2 so next increment lands on previous
  if (m_currentIndex < -1) {
    if (m_loop) {
      m_currentIndex = m_playlist.size() - 2;
    } else {
      m_currentIndex = -1;
    }
  }
  m_isFirstPattern = true;
}

bool PlaylistManager::saveToFile(String filename) {
  if (!filename.startsWith("/")) filename = "/" + filename;
  if (!filename.endsWith(".json")) filename += ".json";
  
  // Use /playlists directory
  if (!SD.exists("/playlists")) {
    SD.mkdir("/playlists");
  }
  String path = "/playlists" + filename;

  JsonDocument doc;
  JsonArray arr = doc["items"].to<JsonArray>();
  
  for (const auto& item : m_playlist) {
    JsonObject obj = arr.add<JsonObject>();
    obj["file"] = item.filename;
  }
  
  doc["loop"] = m_loop;
  doc["clearing"] = m_clearingEnabled;

  File file = SD.open(path.c_str(), FILE_WRITE);
  if (!file) return false;

  serializeJson(doc, file);
  file.close();
  return true;
}

bool PlaylistManager::loadFromFile(String filename) {
  if (!filename.startsWith("/")) filename = "/" + filename;
  if (!filename.endsWith(".json")) filename += ".json";
  String path = "/playlists" + filename;

  File file = SD.open(path.c_str(), FILE_READ);
  if (!file) return false;

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) return false;

  m_playlist.clear();
  JsonArray arr = doc["items"];
  for (JsonObject obj : arr) {
    String f = obj["file"].as<String>();
    m_playlist.push_back(PlaylistItem(f));
  }

  if (doc["loop"].is<bool>()) m_loop = doc["loop"];
  if (doc["clearing"].is<bool>()) m_clearingEnabled = doc["clearing"];

  reset();
  return true;
}