#pragma once
#include <Arduino.h>
#include <vector>
#include <ClearingPatternGen.hpp>

enum PlaylistMode {
    SEQUENTIAL,
    LOOP,
    SHUFFLE
};

struct PlaylistItem {
    String filename;
    ClearingPattern clearing;
    bool useClearing;

    PlaylistItem() : filename(""), clearing(SPIRAL_OUTWARD), useClearing(true) {}
    PlaylistItem(String f, ClearingPattern c, bool use = true)
        : filename(f), clearing(c), useClearing(use) {}
};

class PlaylistManager {
public:
    PlaylistManager();

    // Playlist management
    void addPattern(String filename, ClearingPattern clearing, bool useClearing = true);
    void removePattern(int index);
    void clear();
    void movePattern(int from, int to);
    int count() const { return m_items.size(); }

    // Playback control
    void setMode(PlaylistMode mode);
    PlaylistMode getMode() const { return m_mode; }
    String getNextPattern(ClearingPattern& outClearing, bool& outUseClearing);
    bool hasNext() const;
    void reset();
    void shuffle();
    int getCurrentIndex() const { return m_currentIndex; }

    // Access items
    const PlaylistItem& getItem(int index) const;

    // Persistence
    bool saveToFile(String playlistName);
    bool loadFromFile(String playlistName);

private:
    std::vector<PlaylistItem> m_items;
    PlaylistMode m_mode;
    int m_currentIndex;
    std::vector<int> m_shuffleOrder;

    void generateShuffleOrder();
    int getNextIndex();
    ClearingPattern getRandomClearing();
};
