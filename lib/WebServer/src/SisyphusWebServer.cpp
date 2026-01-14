#include "WiFi.h"
#include "SisyphusWebServer.hpp"
#include "WebUI.h"
#include <ClearingPatternGen.hpp>

// Global WebLogger instance
WebLogger webLogger;

// WebLogger implementation
void WebLogger::ensureMutex() {
    if (m_mutex == NULL) {
        m_mutex = xSemaphoreCreateMutex();
    }
}

size_t WebLogger::write(uint8_t c) {
    ensureMutex();
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    if (m_buffer.length() < MAX_LOG_SIZE) {
        m_buffer += (char)c;
    }
    xSemaphoreGive(m_mutex);
    return 1;
}

size_t WebLogger::write(const uint8_t *buffer, size_t size) {
    ensureMutex();
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    for (size_t i = 0; i < size && m_buffer.length() < MAX_LOG_SIZE; i++) {
        m_buffer += (char)buffer[i];
    }
    xSemaphoreGive(m_mutex);
    return size;
}

String WebLogger::getNewLogs() {
    ensureMutex();
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    String result = m_buffer;
    m_buffer = "";
    xSemaphoreGive(m_mutex);
    return result;
}

SisyphusWebServer::SisyphusWebServer(uint16_t port)
    : m_server(port),
      m_events("/api/console"),
      m_polarControl(nullptr),
      m_ledController(nullptr),
      m_hasQueuedPattern(false),
      m_playlistMode(false),
      m_runningClearing(false),
      m_lastUploadTime(0),
      m_lastRecordedPos({0, 0}),
      m_pathInitialized(false) {
    m_pathX.reserve(MAX_PATH_POINTS);
    m_pathY.reserve(MAX_PATH_POINTS);
}

void SisyphusWebServer::begin(PolarControl *polarControl, LEDController *ledController) {
    if (m_pathMutex == NULL) {
        m_pathMutex = xSemaphoreCreateMutex();
    }
    m_polarControl = polarControl;
    m_ledController = ledController;

    // Web interface routes
    m_server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleRoot(request);
    });

    m_server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleStatus(request);
    });

    m_server.on("/api/pattern/start", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePatternStart(request);
    });

    m_server.on("/api/pattern/stop", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePatternStop(request);
    });

    m_server.on("/api/pattern/pause", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePatternPause(request);
    });

    m_server.on("/api/pattern/resume", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePatternResume(request);
    });

    m_server.on("/api/home", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handleHome(request);
    });

    m_server.on("/api/position", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handlePosition(request);
    });

    m_server.on("/api/files", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleFileList(request);
    });

    m_server.on("/api/files/upload", HTTP_POST,
        [](AsyncWebServerRequest *request) {
            request->send(200);
        },
        [this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
            handleFileUpload(request, filename, index, data, len, final);
        }
    );

    m_server.on("/api/files/delete", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handleFileDelete(request);
    });

    m_server.on("/api/led/brightness", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleLEDBrightnessGet(request);
    });

    m_server.on("/api/led/brightness", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handleLEDBrightnessSet(request);
    });

    m_server.on("/api/speed", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleSpeedGet(request);
    });

    m_server.on("/api/speed", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handleSpeedSet(request);
    });

    m_server.on("/api/system/info", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleSystemInfo(request);
    });

    // Playlist routes
    m_server.on("/api/playlist/add", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePlaylistAdd(request);
    });

    m_server.on("/api/playlist/addall", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePlaylistAddAll(request);
    });

    m_server.on("/api/playlist/remove", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePlaylistRemove(request);
    });

    m_server.on("/api/playlist/clear", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePlaylistClear(request);
    });

    m_server.on("/api/playlist", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handlePlaylistGet(request);
    });

    m_server.on("/api/playlist/start", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePlaylistStart(request);
    });

    m_server.on("/api/playlist/stop", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePlaylistStop(request);
    });

    m_server.on("/api/playlist/mode", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePlaylistMode(request);
    });

    m_server.on("/api/playlist/save", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePlaylistSave(request);
    });

    m_server.on("/api/playlist/load", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePlaylistLoad(request);
    });

    m_server.on("/api/playlist/list", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handlePlaylistList(request);
    });

    m_server.on("/api/playlist/clearing", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handlePlaylistClearing(request);
    });

    // SSE for console logs
    m_events.onConnect([](AsyncEventSourceClient *client) {
        client->send("Connected to console", NULL, millis(), 1000);
    });
    m_server.addHandler(&m_events);

    m_server.begin();
    Serial.println("Web server started on port 80");
}

void SisyphusWebServer::loop() {
    processPatternQueue();
    recordPosition();
    broadcastLogs();
}

void SisyphusWebServer::broadcastLogs() {
    // Broadcast logs every 100ms if there are connected clients
    unsigned long now = millis();
    if (now - m_lastLogBroadcast < 100) return;
    m_lastLogBroadcast = now;

    if (m_events.count() == 0) return;

    String logs = webLogger.getNewLogs();
    if (logs.length() > 0) {
        m_events.send(logs.c_str(), "log", millis());
    }
}

void SisyphusWebServer::processPatternQueue() {
    auto state = m_polarControl->getState();
    uint8_t stateValue = static_cast<uint8_t>(state);

    // Handle single pattern queue
    if (m_hasQueuedPattern && stateValue == 2) { // IDLE
        clearPathHistory();
        m_currentPattern = m_queuedPattern;  // Track running pattern
        m_polarControl->loadAndRunFile("/" + m_queuedPattern);
        m_hasQueuedPattern = false;
        m_queuedPattern = "";
        return;
    }

    // Handle playlist mode
    if (m_playlistMode && stateValue == 2) { // IDLE state
        // If we just finished a clearing pattern, start the pending pattern
        if (m_runningClearing && m_pendingPattern.length() > 0) {
            LOG("Playlist: Starting pattern after clearing: %s\r\n", m_pendingPattern.c_str());
            clearPathHistory();
            m_currentPattern = m_pendingPattern;  // Track running pattern
            m_polarControl->loadAndRunFile("/" + m_pendingPattern);
            m_pendingPattern = "";
            m_runningClearing = false;
            return;
        }

        if (m_playlist.hasNext()) {
            NextPatternResult next = m_playlist.getNextPattern();

            if (next.filename.length() > 0) {
                // Check if we need to run clearing first
                if (next.needsClearing && next.clearingPattern != CLEARING_NONE) {
                    LOG("Playlist: Running clearing pattern %d before: %s\r\n",
                        (int)next.clearingPattern, next.filename.c_str());

                    // Store the pattern to run after clearing
                    m_pendingPattern = next.filename;
                    m_runningClearing = true;

                    // Start the clearing pattern
                    ClearingPattern pattern = next.clearingPattern;
                    if (pattern == CLEARING_RANDOM) {
                        pattern = getRandomClearingPattern();
                    }
                    ClearingPatternGen* clearingGen = new ClearingPatternGen(pattern, m_polarControl->getMaxRho());
                    m_polarControl->start(clearingGen);
                } else {
                    // No clearing needed, start pattern directly
                    LOG("Playlist: Starting pattern: %s\r\n", next.filename.c_str());
                    clearPathHistory();
                    m_currentPattern = next.filename;  // Track running pattern
                    m_polarControl->loadAndRunFile("/" + next.filename);
                }
            }
        } else {
            // Playlist ended (only possible in SEQUENTIAL mode)
            LOG("Playlist complete\r\n");
            m_playlistMode = false;
            m_runningClearing = false;
            m_pendingPattern = "";
        }
    }
}

String SisyphusWebServer::getStateString() {
    auto state = m_polarControl->getState();
    uint8_t stateValue = static_cast<uint8_t>(state);

    switch (stateValue) {
        case 0: return "UNINITIALIZED";
        case 1: return "INITIALIZED";
        case 2: return "IDLE";
        case 3: return "RUNNING";
        case 4: return "PAUSED";
        case 5: return "STOPPING";
        default: return "UNKNOWN";
    }
}

String SisyphusWebServer::buildStatusJSON() {
    JsonDocument doc;

    doc["state"] = getStateString();
    doc["currentPattern"] = m_currentPattern;
    uint8_t brightness = m_ledController->getBrightness();
    doc["ledBrightness"] = map(brightness, 0, 255, 0, 100);
    doc["heap"] = ESP.getFreeHeap();
    doc["uptime"] = millis() / 1000;

    String output;
    serializeJson(doc, output);
    return output;
}

String SisyphusWebServer::buildFileListJSON() {
    JsonDocument doc;
    JsonArray files = doc["files"].to<JsonArray>();

    File root = SD.open("/");
    if (root) {
        File file = root.openNextFile();
        while (file) {
            if (!file.isDirectory()) {
                String name = String(file.name());
                // Remove leading slash if present
                if (name.startsWith("/")) name = name.substring(1);
                
                if (name.endsWith(".thr")) {
                    JsonObject fileObj = files.add<JsonObject>();
                    fileObj["name"] = name;
                    fileObj["size"] = file.size();
                }
            }
            file.close();
            file = root.openNextFile();
        }
        root.close();
    }

    String output;
    serializeJson(doc, output);
    return output;
}

String SisyphusWebServer::buildSystemInfoJSON() {
    JsonDocument doc;

    doc["heap"] = ESP.getFreeHeap();
    doc["uptime"] = millis() / 1000;

    JsonObject wifi = doc["wifi"].to<JsonObject>();
    wifi["ssid"] = WiFi.SSID();
    wifi["ip"] = WiFi.localIP().toString();
    wifi["rssi"] = WiFi.RSSI();

    String output;
    serializeJson(doc, output);
    return output;
}

void SisyphusWebServer::handleRoot(AsyncWebServerRequest *request) {
    request->send(200, "text/html", WEB_UI_HTML);
}

void SisyphusWebServer::handleStatus(AsyncWebServerRequest *request) {
    String json = buildStatusJSON();
    request->send(200, "application/json", json);
}

void SisyphusWebServer::handlePatternStart(AsyncWebServerRequest *request) {
    auto state = m_polarControl->getState();
    uint8_t stateValue = static_cast<uint8_t>(state);

    if (stateValue != 2) {
        request->send(409, "application/json",
            "{\"success\":false,\"message\":\"System not idle\"}");
        return;
    }

    if (!request->hasParam("file", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing parameters\"}");
        return;
    }

    String file = request->getParam("file", true)->value();

    String filePath = "/" + file;
    if (!SD.exists(filePath)) {
        request->send(404, "application/json",
            "{\"success\":false,\"message\":\"File not found\"}");
        return;
    }

    m_queuedPattern = file;
    m_hasQueuedPattern = true;

    Serial.print("Queued pattern: ");
    Serial.println(file);

    request->send(200, "application/json",
        "{\"success\":true,\"message\":\"Pattern started\"}");
}

void SisyphusWebServer::handlePatternStop(AsyncWebServerRequest *request) {
    bool success = m_polarControl->stop();
    m_hasQueuedPattern = false;
    m_queuedPattern = "";

    if (success) {
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(500, "application/json",
            "{\"success\":false,\"message\":\"Failed to stop\"}");
    }
}

void SisyphusWebServer::handlePatternPause(AsyncWebServerRequest *request) {
    bool success = m_polarControl->pause();

    if (success) {
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(409, "application/json",
            "{\"success\":false,\"message\":\"Cannot pause\"}");
    }
}

void SisyphusWebServer::handlePatternResume(AsyncWebServerRequest *request) {
    bool success = m_polarControl->resume();

    if (success) {
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(409, "application/json",
            "{\"success\":false,\"message\":\"Cannot resume\"}");
    }
}

void SisyphusWebServer::handleHome(AsyncWebServerRequest *request) {
    auto state = m_polarControl->getState();
    uint8_t stateValue = static_cast<uint8_t>(state);

    // Only allow homing when system is idle
    if (stateValue != 2) { // 2 = IDLE
        request->send(409, "application/json",
            "{\"success\":false,\"message\":\"System must be idle to home\"}");
        return;
    }

    Serial.println("Web request: Homing device...");
    m_polarControl->home();

    request->send(200, "application/json",
        "{\"success\":true,\"message\":\"Homing complete\"}");
}

void SisyphusWebServer::handleFileList(AsyncWebServerRequest *request) {
    String json = buildFileListJSON();
    request->send(200, "application/json", json);
}

void SisyphusWebServer::handleFileUpload(AsyncWebServerRequest *request, String filename,
                                        size_t index, uint8_t *data, size_t len, bool final) {
    if (index == 0) {
        unsigned long now = millis();
        if (now - m_lastUploadTime < 10000) {
            request->send(429, "application/json",
                "{\"success\":false,\"message\":\"Too many requests\"}");
            return;
        }
        m_lastUploadTime = now;

        if (!filename.endsWith(".thr")) {
            request->send(400, "application/json",
                "{\"success\":false,\"message\":\"Invalid file type\"}");
            return;
        }

        int lastSlash = filename.lastIndexOf('/');
        if (lastSlash != -1) {
            filename = filename.substring(lastSlash + 1);
        }

        Serial.print("Upload start: ");
        Serial.println(filename);

        String path = "/" + filename;
        if (SD.exists(path)) {
            SD.remove(path);
        }
        
        m_uploadFile = SD.open(path.c_str(), FILE_WRITE);
        if (!m_uploadFile) {
            Serial.println("Failed to open file for writing");
            request->send(500, "application/json",
                "{\"success\":false,\"message\":\"Failed to create file\"}");
            return;
        }
    }

    if (m_uploadFile && len) {
        m_uploadFile.write(data, len);
    }

    if (final) {
        if (m_uploadFile) {
            m_uploadFile.close();
            Serial.print("Upload complete: ");
            Serial.print(filename);
            Serial.print(" (");
            Serial.print(index + len);
            Serial.println(" bytes)");

            JsonDocument doc;
            doc["success"] = true;
            doc["filename"] = filename;
            doc["size"] = index + len;

            String output;
            serializeJson(doc, output);

            request->send(200, "application/json", output);
        } else {
            request->send(500, "application/json",
                "{\"success\":false,\"message\":\"Upload failed\"}");
        }
    }
}

void SisyphusWebServer::handleFileDelete(AsyncWebServerRequest *request) {
    if (!request->hasParam("file", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing filename\"}");
        return;
    }

    String filename = request->getParam("file", true)->value();
    String path = "/" + filename;

    if (path.indexOf("..") != -1) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Invalid filename\"}");
        return;
    }

    if (!SD.exists(path)) {
        request->send(404, "application/json",
            "{\"success\":false,\"message\":\"File not found\"}");
        return;
    }

    if (SD.remove(path)) {
        Serial.print("Deleted: ");
        Serial.println(filename);
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(500, "application/json",
            "{\"success\":false,\"message\":\"Failed to delete\"}");
    }
}

void SisyphusWebServer::handleLEDBrightnessGet(AsyncWebServerRequest *request) {
    JsonDocument doc;
    uint8_t brightness = m_ledController->getBrightness();
    doc["brightness"] = map(brightness, 0, 255, 0, 100);

    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
}

void SisyphusWebServer::handleLEDBrightnessSet(AsyncWebServerRequest *request) {
    if (!request->hasParam("brightness", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing brightness parameter\"}");
        return;
    }

    int brightness = request->getParam("brightness", true)->value().toInt();

    if (brightness < 0 || brightness > 100) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Brightness must be 0-100\"}");
        return;
    }

    uint8_t ledValue = map(brightness, 0, 100, 0, 255);
    m_ledController->setBrightness(ledValue);

    Serial.print("LED brightness set to: ");
    Serial.print(brightness);
    Serial.print("% (");
    Serial.print(ledValue);
    Serial.println("/255)");

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handleSpeedGet(AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["speed"] = 5;

    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
}

void SisyphusWebServer::handleSpeedSet(AsyncWebServerRequest *request) {
    if (!request->hasParam("speed", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing speed parameter\"}");
        return;
    }

    int speed = request->getParam("speed", true)->value().toInt();

    if (speed < 1 || speed > 10) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Speed must be 1-10\"}");
        return;
    }

    m_polarControl->setSpeed(speed);

    Serial.print("Speed set to: ");
    Serial.println(speed);

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handleSystemInfo(AsyncWebServerRequest *request) {
    String json = buildSystemInfoJSON();
    request->send(200, "application/json", json);
}

// ============================================================================
// Playlist Handlers
// ============================================================================

void SisyphusWebServer::handlePlaylistAdd(AsyncWebServerRequest *request) {
    if (!request->hasParam("file", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing parameters\"}");
        return;
    }

    String file = request->getParam("file", true)->value();
    m_playlist.addPattern(file);

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handlePlaylistAddAll(AsyncWebServerRequest *request) {
    // Get all .thr files from SD
    File root = SD.open("/");
    if (!root) {
        request->send(500, "application/json",
            "{\"success\":false,\"message\":\"Failed to open root directory\"}");
        return;
    }

    int count = 0;
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            String filename = String(file.name());
            // Remove leading slash if present
            if (filename.startsWith("/")) filename = filename.substring(1);

            // Only add .thr files
            if (filename.endsWith(".thr")) {
                m_playlist.addPattern(filename);
                count++;
                Serial.print("Added to playlist: ");
                Serial.println(filename);
            }
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();

    String response = "{\"success\":true,\"count\":" + String(count) + "}";
    request->send(200, "application/json", response);

    Serial.print("Added all patterns to playlist. Total: ");
    Serial.println(count);
}

void SisyphusWebServer::handlePlaylistRemove(AsyncWebServerRequest *request) {
    if (!request->hasParam("index", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing index\"}");
        return;
    }

    int index = request->getParam("index", true)->value().toInt();
    m_playlist.removePattern(index);

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handlePlaylistClear(AsyncWebServerRequest *request) {
    m_playlist.clear();
    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handlePlaylistGet(AsyncWebServerRequest *request) {
    JsonDocument doc;

    String modeStr;
    switch (m_playlist.getMode()) {
        case SEQUENTIAL:
            modeStr = "sequential";
            break;
        case LOOP:
            modeStr = "loop";
            break;
        case SHUFFLE:
            modeStr = "shuffle";
            break;
    }

    doc["mode"] = modeStr;
    doc["playing"] = m_playlistMode;
    doc["currentIndex"] = m_playlist.getCurrentIndex();
    doc["count"] = m_playlist.count();
    doc["clearingEnabled"] = m_playlist.isClearingEnabled();

    JsonArray items = doc["items"].to<JsonArray>();
    for (int i = 0; i < m_playlist.count(); i++) {
        const PlaylistItem& item = m_playlist.getItem(i);
        JsonObject obj = items.add<JsonObject>();
        obj["filename"] = item.filename;
    }

    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
}

void SisyphusWebServer::handlePlaylistStart(AsyncWebServerRequest *request) {
    auto state = m_polarControl->getState();
    uint8_t stateValue = static_cast<uint8_t>(state);

    if (stateValue != 2) { // Not IDLE
        request->send(409, "application/json",
            "{\"success\":false,\"message\":\"System not idle\"}");
        return;
    }

    if (m_playlist.count() == 0) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Playlist is empty\"}");
        return;
    }

    m_playlist.reset();
    m_playlistMode = true;

    Serial.println("Playlist started");
    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handlePlaylistStop(AsyncWebServerRequest *request) {
    m_playlistMode = false;
    m_polarControl->stop();

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handlePlaylistMode(AsyncWebServerRequest *request) {
    if (!request->hasParam("mode", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing mode parameter\"}");
        return;
    }

    String modeStr = request->getParam("mode", true)->value();
    PlaylistMode mode;

    if (modeStr == "sequential") {
        mode = SEQUENTIAL;
    } else if (modeStr == "loop") {
        mode = LOOP;
    } else if (modeStr == "shuffle") {
        mode = SHUFFLE;
    } else {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Invalid mode\"}");
        return;
    }

    m_playlist.setMode(mode);

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handlePlaylistSave(AsyncWebServerRequest *request) {
    if (!request->hasParam("name", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing name parameter\"}");
        return;
    }

    String name = request->getParam("name", true)->value();

    if (m_playlist.saveToFile(name)) {
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(500, "application/json",
            "{\"success\":false,\"message\":\"Failed to save playlist\"}");
    }
}

void SisyphusWebServer::handlePlaylistLoad(AsyncWebServerRequest *request) {
    if (!request->hasParam("name", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing name parameter\"}");
        return;
    }

    String name = request->getParam("name", true)->value();

    if (m_playlist.loadFromFile(name)) {
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(404, "application/json",
            "{\"success\":false,\"message\":\"Playlist not found\"}");
    }
}

void SisyphusWebServer::handlePlaylistList(AsyncWebServerRequest *request) {
    JsonDocument doc;
    JsonArray playlists = doc["playlists"].to<JsonArray>();

    // List all .json files in /playlists directory
    File root = SD.open("/playlists");
    if (root) {
        File file = root.openNextFile();
        while (file) {
            if (!file.isDirectory()) {
                String filename = String(file.name());
                // Remove leading slash if present
                if (filename.startsWith("/")) filename = filename.substring(1);

                if (filename.endsWith(".json")) {
                    // Remove .json extension
                    filename = filename.substring(0, filename.length() - 5);
                    playlists.add(filename);
                }
            }
            file.close();
            file = root.openNextFile();
        }
        root.close();
    }

    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
}

void SisyphusWebServer::handlePlaylistClearing(AsyncWebServerRequest *request) {
    if (!request->hasParam("enabled", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing enabled parameter\"}");
        return;
    }

    String enabledStr = request->getParam("enabled", true)->value();
    bool enabled = (enabledStr == "true" || enabledStr == "1");

    m_playlist.setClearingEnabled(enabled);

    Serial.print("Clearing patterns ");
    Serial.println(enabled ? "enabled" : "disabled");

    request->send(200, "application/json", "{\"success\":true}");
}

// ============================================================================
// Position and Path Tracking
// ============================================================================

void SisyphusWebServer::handlePosition(AsyncWebServerRequest *request) {
    JsonDocument doc;

    // Get actual position from stepper motors (used for display)
    PolarCord_t actualPos = m_polarControl->getActualPosition();
    double maxRho = m_polarControl->getMaxRho();

    // Convert to normalized Cartesian coordinates (0-1 range)
    double x = actualPos.rho * cos(actualPos.theta) / maxRho;
    double y = actualPos.rho * sin(actualPos.theta) / maxRho;

    // Normalize to 0-1 range (center at 0.5,0.5)
    doc["current"]["x"] = (x + 1.0) / 2.0;
    doc["current"]["y"] = (y + 1.0) / 2.0;
    doc["current"]["rho"] = actualPos.rho;
    doc["current"]["theta"] = actualPos.theta;

    // Add path history
    xSemaphoreTake(m_pathMutex, portMAX_DELAY);
    JsonArray path = doc["path"].to<JsonArray>();
    for (size_t i = 0; i < m_pathX.size(); ++i) {
        JsonObject point = path.add<JsonObject>();
        point["x"] = m_pathX[i];
        point["y"] = m_pathY[i];
    }
    xSemaphoreGive(m_pathMutex);

    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
}

void SisyphusWebServer::recordPosition() {
    // Get actual position from stepper motors
    PolarCord_t actualPos = m_polarControl->getActualPosition();
    double maxRho = m_polarControl->getMaxRho();

    // Only record if position has changed significantly
    if (!m_pathInitialized ||
        abs(actualPos.rho - m_lastRecordedPos.rho) > 2.0 ||
        abs(actualPos.theta - m_lastRecordedPos.theta) > 0.02) {

        // Convert to normalized Cartesian coordinates
        double x = actualPos.rho * cos(actualPos.theta) / maxRho;
        double y = actualPos.rho * sin(actualPos.theta) / maxRho;

        // Normalize to 0-1 range (center at 0.5,0.5)
        float normX = (x + 1.0f) / 2.0f;
        float normY = (y + 1.0f) / 2.0f;

        // Add to path history
        xSemaphoreTake(m_pathMutex, portMAX_DELAY);
        m_pathX.push_back(normX);
        m_pathY.push_back(normY);

        // Limit path history size
        if (m_pathX.size() > MAX_PATH_POINTS) {
            m_pathX.erase(m_pathX.begin());
            m_pathY.erase(m_pathY.begin());
        }
        xSemaphoreGive(m_pathMutex);

        m_lastRecordedPos = actualPos;
        m_pathInitialized = true;
    }
}

void SisyphusWebServer::clearPathHistory() {
    xSemaphoreTake(m_pathMutex, portMAX_DELAY);
    m_pathX.clear();
    m_pathY.clear();
    xSemaphoreGive(m_pathMutex);
    m_pathInitialized = false;
    m_lastRecordedPos = {0.0, 0.0};
}
