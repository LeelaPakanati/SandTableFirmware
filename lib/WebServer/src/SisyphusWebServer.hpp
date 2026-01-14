#pragma once
#include <ESPAsyncWebServer.h>
#include <vector>
#include <ArduinoJson.h>
#include <freertos/semphr.h>
#include <SDCard.hpp>
#include <PolarControl.hpp>
#include <LEDController.hpp>
#include <PlaylistManager.hpp>
#include "Logger.hpp"

class SisyphusWebServer {
public:
    SisyphusWebServer(uint16_t port = 80);
    void begin(PolarControl *polarControl, LEDController *ledController);
    void loop(); // Check for pattern queue processing

private:
    AsyncWebServer m_server;
    AsyncEventSource m_events;  // SSE for console logs
    PolarControl *m_polarControl;
    LEDController *m_ledController;

    // Pattern queue management
    String m_queuedPattern;
    String m_currentPattern;  // Currently running pattern filename
    bool m_hasQueuedPattern;
    unsigned long m_lastUploadTime;

    // Playlist management
    PlaylistManager m_playlist;
    bool m_playlistMode;
    bool m_runningClearing;       // True if currently running a clearing pattern
    String m_pendingPattern;      // Pattern to run after clearing completes

    // File upload handling
    File m_uploadFile;

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
    void handlePlaylistClearing(AsyncWebServerRequest *request);

    // Helper methods
    void processPatternQueue();
    void broadcastLogs();
    String buildStatusJSON();
    String buildFileListJSON();
    String buildSystemInfoJSON();
    String getStateString();

    unsigned long m_lastLogBroadcast = 0;
};
