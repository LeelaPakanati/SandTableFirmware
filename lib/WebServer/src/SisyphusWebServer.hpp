#pragma once
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <PolarControl.hpp>
#include <LEDController.hpp>
#include <ClearingPatternGen.hpp>
#include <PlaylistManager.hpp>
#include <SDCard.hpp>
#include <vector>

class SisyphusWebServer {
public:
    SisyphusWebServer(uint16_t port = 80);
    void begin(PolarControl *polarControl, LEDController *ledController);
    void loop(); // Check for pattern queue processing

private:
    AsyncWebServer m_server;
    PolarControl *m_polarControl;
    LEDController *m_ledController;

    // Pattern queue management
    String m_queuedPattern;
    ClearingPattern m_queuedClearing;
    bool m_isClearing;
    bool m_hasQueuedPattern;
    unsigned long m_lastUploadTime;
    uint8_t m_speedBeforeClearing; // Save speed to restore after clearing

    // Playlist management
    PlaylistManager m_playlist;
    bool m_playlistMode;

    // File upload handling
    FsFile m_uploadFile;

    // Path history for viewer (stores points as x,y in normalized 0-1 range)
    static constexpr int MAX_PATH_POINTS = 500;
    std::vector<float> m_pathX;
    std::vector<float> m_pathY;
    PolarCord_t m_lastRecordedPos;
    bool m_pathInitialized;

    // Route handlers
    void handlePosition(AsyncWebServerRequest *request);
    void handleStatus(AsyncWebServerRequest *request);
    void handlePatternStart(AsyncWebServerRequest *request);
    void handlePatternStop(AsyncWebServerRequest *request);
    void handlePatternPause(AsyncWebServerRequest *request);
    void handlePatternResume(AsyncWebServerRequest *request);
    void handleHome(AsyncWebServerRequest *request);
    void handleFileList(AsyncWebServerRequest *request);
    void handleFileUpload(AsyncWebServerRequest *request, String filename,
                         size_t index, uint8_t *data, size_t len, bool final);
    void handleFileDelete(AsyncWebServerRequest *request);
    void handleLEDBrightnessGet(AsyncWebServerRequest *request);
    void handleLEDBrightnessSet(AsyncWebServerRequest *request);
    void handleSpeedGet(AsyncWebServerRequest *request);
    void handleSpeedSet(AsyncWebServerRequest *request);
    void handleSystemInfo(AsyncWebServerRequest *request);
    void handleRoot(AsyncWebServerRequest *request);

    // Playlist handlers
    void handlePlaylistAdd(AsyncWebServerRequest *request);
    void handlePlaylistAddAll(AsyncWebServerRequest *request);
    void handlePlaylistRemove(AsyncWebServerRequest *request);
    void handlePlaylistClear(AsyncWebServerRequest *request);
    void handlePlaylistGet(AsyncWebServerRequest *request);
    void handlePlaylistStart(AsyncWebServerRequest *request);
    void handlePlaylistStop(AsyncWebServerRequest *request);
    void handlePlaylistMode(AsyncWebServerRequest *request);
    void handlePlaylistSave(AsyncWebServerRequest *request);
    void handlePlaylistLoad(AsyncWebServerRequest *request);
    void handlePlaylistList(AsyncWebServerRequest *request);

    // Helper methods
    void processPatternQueue();
    void recordPosition();
    void clearPathHistory();
    String buildStatusJSON();
    String buildFileListJSON();
    String buildSystemInfoJSON();
    String getStateString();
    ClearingPattern stringToClearingPattern(const String &str);
};
