#pragma once
#include <vector>
#include <Arduino.h>
#include <SDCard.hpp>
#include <ArduinoJson.h>
#include <ClearingPatternGen.hpp>

struct PlaylistItem {
  String filename;

  PlaylistItem(String f)
    : filename(f) {}
};

enum PlaylistMode {
  SEQUENTIAL,
  LOOP,
  SHUFFLE
};

// Result from getNextPattern - includes clearing info
struct NextPatternResult {
  String filename;
  ClearingPattern clearingPattern;
  bool needsClearing;  // True if this is not the first pattern
};

class PlaylistManager {
public:
  PlaylistManager();

  void addPattern(String filename);
  void removePattern(int index);
  void clear();

  int count() const { return m_playlist.size(); }
  const PlaylistItem& getItem(int index) const { return m_playlist[index]; }

  void setMode(PlaylistMode mode);
  PlaylistMode getMode() const { return m_mode; }

  // Clearing settings
  void setClearingEnabled(bool enabled) { m_clearingEnabled = enabled; }
  bool isClearingEnabled() const { return m_clearingEnabled; }

  // Playlist control
  void reset();
  bool hasNext();
  NextPatternResult getNextPattern();
  int getCurrentIndex() const { return m_currentIndex; }

  // File I/O
  bool saveToFile(String filename);
  bool loadFromFile(String filename);

private:
  std::vector<PlaylistItem> m_playlist;
  PlaylistMode m_mode;
  int m_currentIndex;
  bool m_clearingEnabled;
  bool m_isFirstPattern;  // Track if this is the first pattern (no clearing needed)

  // Shuffle support
  std::vector<int> m_shuffleIndices;
  int m_shuffleIndex;
  void reshuffle();
};
