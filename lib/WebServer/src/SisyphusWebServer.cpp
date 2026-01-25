#include "WiFi.h"
#include "SisyphusWebServer.hpp"
#include "WebUI.h"
#include "TuningUI.h"
#include "FileUI.h"
#include "JsonHelpers.hpp"
#include "PolarUtils.hpp"
#include "MakeUnique.hpp"
#include <SDCard.hpp>
#include <ClearingPatternGen.hpp>

static constexpr size_t kResponseBufferSize = 256;

// Helper to resolve pattern file path
static String resolvePatternPath(const String& filename) {
    String basename = filename;
    if (basename.endsWith(".thr")) basename = basename.substring(0, basename.length() - 4);
    String path = "/patterns/" + basename + "/" + filename;
    if (!SD.exists(path)) path = "/patterns/" + filename;
    return path;
}

static constexpr bool kEnablePatternImages = true;

SisyphusWebServer::SisyphusWebServer(uint16_t port)
    : m_server(port),
      m_events("/api/stream"),
      m_polarControl(nullptr),
      m_ledController(nullptr),
      m_hasQueuedPattern(false),
      m_singlePatternClearing(false),
      m_selectedClearing(CLEARING_NONE),
      m_playlistMode(false),
      m_runningClearing(false),
      m_firstPointCleared(false),
      m_lastUploadTime(0),
      m_fileListDirty(true) {
    m_cacheMutex = xSemaphoreCreateMutex();
}

void SisyphusWebServer::getRequestStats(uint32_t& total, uint32_t& inflight) const {
    total = m_requestTotal.load();
    inflight = m_requestInflight.load();
}

void SisyphusWebServer::noteRequest(AsyncWebServerRequest *request) {
    m_requestTotal.fetch_add(1);
    m_requestInflight.fetch_add(1);
    request->onDisconnect([this]() {
        uint32_t current = m_requestInflight.load();
        while (current > 0 && !m_requestInflight.compare_exchange_weak(current, current - 1)) {
        }
    });
}

void SisyphusWebServer::begin(PolarControl *polarControl, LEDController *ledController) {
    m_polarControl = polarControl;
    m_ledController = ledController;

    // Enable CORS
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");
    DefaultHeaders::Instance().addHeader("Connection", "close");

    // Handle OPTIONS requests (preflight)
    m_server.onNotFound([](AsyncWebServerRequest *request) {
        if (request->method() == HTTP_OPTIONS) {
            request->send(200);
        } else {
            request->send(404);
        }
    });

    // Web interface routes
    m_server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleRoot(request);
    });

    m_server.on("/tuning", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        request->send(200, "text/html",
                      reinterpret_cast<const uint8_t *>(TUNING_UI_HTML),
                      sizeof(TUNING_UI_HTML) - 1);
    });

    m_server.on("/files", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        request->send(200, "text/html",
                      reinterpret_cast<const uint8_t *>(FILE_UI_HTML),
                      sizeof(FILE_UI_HTML) - 1);
    });

    m_server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleStatus(request);
    });

    m_server.on("/api/pattern/start", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePatternStart(request);
    });

    m_server.on("/api/pattern/stop", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePatternStop(request);
    });

    m_server.on("/api/pattern/pause", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePatternPause(request);
    });

    m_server.on("/api/pattern/resume", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePatternResume(request);
    });

    m_server.on("/api/home", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleHome(request);
    });

    m_server.on("/api/position", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePosition(request);
    });

    m_server.on("/api/files", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleFileList(request);
    });

    if (kEnablePatternImages) {
        m_server.on("/api/pattern/image", HTTP_GET, [this](AsyncWebServerRequest *request) {
            noteRequest(request);
            
            // Limit concurrent file operations to prevent FD exhaustion (FATFS limit is low)
            if (m_requestInflight > 3) {
                request->send(503);
                return;
            }

            if (!request->hasParam("file")) {
                request->send(400);
                return;
            }
            String file = request->getParam("file")->value();
            if (file.endsWith(".thr")) {
                file = file.substring(0, file.length() - 4);
            }

            // Lookup in cache
            bool found = false;
            time_t imgTime = 0;
            String pngPath;
            
            // Acquire lock briefly to look up file
            if (m_cacheMutex) {
                xSemaphoreTake(m_cacheMutex, portMAX_DELAY);
                for (const auto& entry : m_fileCache) {
                    // Entry name includes extension .thr
                    String entryBase = entry.name;
                    if (entryBase.endsWith(".thr")) entryBase = entryBase.substring(0, entryBase.length() - 4);
                    
                    if (entryBase == file) {
                        if (entry.hasImage) {
                            found = true;
                            imgTime = entry.imageTime;
                            
                            // Reconstruct path based on directory structure flag
                            if (entry.isDirectory) {
                                pngPath = "/patterns/" + file + "/" + file + ".png";
                            } else {
                                pngPath = "/patterns/" + file + ".png";
                            }
                        }
                        break;
                    }
                }
                xSemaphoreGive(m_cacheMutex);
            }

            if (found) {
                AsyncWebServerResponse *response = request->beginResponse(SD, pngPath, "image/png");
                response->addHeader("Cache-Control", "public, max-age=31536000, immutable");
                
                if (imgTime > 0) {
                    char timeStr[32];
                    struct tm * tmstruct = gmtime((time_t*)&imgTime);
                    strftime(timeStr, sizeof(timeStr), "%a, %d %b %Y %H:%M:%S GMT", tmstruct);
                    response->addHeader("Last-Modified", timeStr);
                }
                
                request->send(response);
            } else {
                request->send(404);
            }
        });
    } else {
        m_server.on("/api/pattern/image", HTTP_GET, [this](AsyncWebServerRequest *request) {
            noteRequest(request);
            request->send(404);
        });
    }

    m_server.on("/api/files/upload", HTTP_POST,
        [this](AsyncWebServerRequest *request) {
            noteRequest(request);
            request->send(200);
        },
        [this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
            handleFileUpload(request, filename, index, data, len, final);
        }
    );

    m_server.on("/api/files/delete", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleFileDelete(request);
    });

    m_server.on("/api/led/brightness", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleLEDBrightnessGet(request);
    });

    m_server.on("/api/led/brightness", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleLEDBrightnessSet(request);
    });

    m_server.on("/api/speed", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleSpeedGet(request);
    });

    m_server.on("/api/speed", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleSpeedSet(request);
    });

    m_server.on("/api/system/info", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleSystemInfo(request);
    });

    // Playlist routes
    m_server.on("/api/playlist/add", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistAdd(request);
    });

    m_server.on("/api/playlist/addall", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistAddAll(request);
    });

    m_server.on("/api/playlist/remove", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistRemove(request);
    });

    m_server.on("/api/playlist/clear", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistClear(request);
    });

    m_server.on("/api/playlist", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistGet(request);
    });

    m_server.on("/api/playlist/start", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistStart(request);
    });

    m_server.on("/api/playlist/stop", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistStop(request);
    });

    m_server.on("/api/playlist/loop", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistLoop(request);
    });

    m_server.on("/api/playlist/shuffle", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistShuffle(request);
    });

    m_server.on("/api/playlist/save", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistSave(request);
    });

    m_server.on("/api/playlist/load", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistLoad(request);
    });

    m_server.on("/api/playlist/list", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistList(request);
    });

    m_server.on("/api/playlist/clearing", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistClearing(request);
    });

    m_server.on("/api/playlist/move", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistMove(request);
    });

    m_server.on("/api/playlist/skipto", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistSkipTo(request);
    });

    m_server.on("/api/playlist/next", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistNext(request);
    });

    m_server.on("/api/playlist/prev", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handlePlaylistPrev(request);
    });

    // Tuning routes
    m_server.on("/api/tuning", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleTuningGet(request);
    });

    m_server.on("/api/tuning/motion", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleTuningMotionSet(request);
    });

    m_server.on("/api/tuning/theta", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleTuningThetaDriverSet(request);
    });

    m_server.on("/api/tuning/rho", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleTuningRhoDriverSet(request);
    });

    m_server.on("/api/tuning/test/theta/continuous", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleTuningTestThetaContinuous(request);
    });

    m_server.on("/api/tuning/test/theta/stress", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleTuningTestThetaStress(request);
    });

    m_server.on("/api/tuning/test/rho/continuous", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleTuningTestRhoContinuous(request);
    });

    m_server.on("/api/tuning/test/rho/stress", HTTP_POST, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        handleTuningTestRhoStress(request);
    });

    // Driver dump routes (for diagnostics)
    m_server.on("/api/tuning/dump/theta", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
        m_polarControl->writeThetaDriverSettings(*response);
        request->send(response);
    });

    m_server.on("/api/tuning/dump/rho", HTTP_GET, [this](AsyncWebServerRequest *request) {
        noteRequest(request);
        AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
        m_polarControl->writeRhoDriverSettings(*response);
        request->send(response);
    });

    // SSE for position stream
    m_events.onConnect([this](AsyncEventSourceClient *client) {
        this->broadcastSinglePosition(client); // Send initial position
    });
    m_server.addHandler(&m_events);

    m_server.begin();
    Serial.println("Web server started on port 80");
}

void SisyphusWebServer::updateFileListCache() {
    Serial.println("Updating file list cache in memory...");
    
    std::vector<FileEntry> newCache;
    
    File root = SD.open("/patterns");
    if (!root) {
        Serial.println("ERROR: Failed to open patterns directory");
        m_fileListDirty = false; // Prevent infinite retry loop if SD fails
        return;
    }

    File file = root.openNextFile();
    while (file) {
        String name = String(file.name());
        if (name.startsWith("/")) name = name.substring(1);

        if (!file.isDirectory()) {
            if (name.endsWith(".thr")) {
                String baseName = name.substring(0, name.length() - 4);
                String pngPath = "/patterns/" + baseName + ".png";
                bool hasImg = false;
                time_t imgTime = 0;
                
                if (SD.exists(pngPath)) {
                    hasImg = true;
                    File img = SD.open(pngPath);
                    if (img) {
                        imgTime = img.getLastWrite();
                        img.close();
                    }
                }
                newCache.push_back({name, file.size(), file.getLastWrite(), hasImg, imgTime, false});
            }
        } else {
            // Check for pattern inside directory
            String patternFile = name + ".thr";
            String innerPath = "/patterns/" + name + "/" + patternFile;
            if (SD.exists(innerPath)) {
                File innerFile = SD.open(innerPath);
                if (innerFile) {
                    // Check for image inside directory
                    String pngPath = "/patterns/" + name + "/" + name + ".png";
                    bool hasImg = false;
                    time_t imgTime = 0;
                    
                    if (SD.exists(pngPath)) {
                        hasImg = true;
                        File img = SD.open(pngPath);
                        if (img) {
                            imgTime = img.getLastWrite();
                            img.close();
                        }
                    }
                    
                    newCache.push_back({patternFile, innerFile.size(), innerFile.getLastWrite(), hasImg, imgTime, true});
                    innerFile.close();
                }
            }
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
    
    if (m_cacheMutex) {
        xSemaphoreTake(m_cacheMutex, portMAX_DELAY);
        m_fileCache = std::move(newCache);
        xSemaphoreGive(m_cacheMutex);
    }
    
    m_fileListDirty = false;
    Serial.printf("File list cache updated: %u files\n", m_fileCache.size());
}

void SisyphusWebServer::loop() {
    if (m_fileListDirty) {
        updateFileListCache();
    }
    processPatternQueue();
    broadcastPosition();
}

void SisyphusWebServer::broadcastSinglePosition(AsyncEventSourceClient *client) {
    // Get actual position (thread-safe now)
    PolarCord_t actualPos = m_polarControl->getActualPosition();
    float maxRho = m_polarControl->getMaxRho();

    // Convert to normalized Cartesian coordinates (0-1 range)
    CartesianCord_t norm = PolarUtils::toNormalizedCartesian(actualPos, maxRho);

    // Create compact JSON manually to save heap
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "{\"x\":%.4f,\"y\":%.4f,\"r\":%.1f,\"t\":%.2f}", 
             norm.x, norm.y, actualPos.rho, actualPos.theta);

    try {
        if (client) {
            client->send(buffer, "pos", millis());
        } else {
            m_events.send(buffer, "pos", millis());
        }
    } catch (...) {
        // Drop the event on allocation failure to avoid crashing Core 0.
    }
}

void SisyphusWebServer::broadcastPosition() {
    // Check if we should broadcast based on machine state
    auto state = m_polarControl->getState();
    uint8_t stateValue = static_cast<uint8_t>(state);

    // Only broadcast in RUNNING (3), STOPPING (5), or CLEARING (6) states
    if (stateValue != 3 && stateValue != 5 && stateValue != 6) {
        return;
    }

    // Broadcast position check every ~66ms (~15Hz)
    unsigned long now = millis();
    if (now - m_lastPosBroadcast < 66) return;
    m_lastPosBroadcast = now;

    if (m_events.count() == 0) return;

    // Get actual position
    PolarCord_t actualPos = m_polarControl->getActualPosition();
    float maxRho = m_polarControl->getMaxRho();
    CartesianCord_t norm = PolarUtils::toNormalizedCartesian(actualPos, maxRho);

    // Only broadcast if changed significantly to reduce network load
    if (abs(norm.x - m_lastBroadcastX) < 0.0001 && abs(norm.y - m_lastBroadcastY) < 0.0001) {
        return;
    }

    m_lastBroadcastX = norm.x;
    m_lastBroadcastY = norm.y;

    // Check if we should signal a clear (reached first point of pattern)
    bool shouldClear = false;
    if (!m_firstPointCleared && m_polarControl->getSegmentsCompleted() >= 1) {
        shouldClear = true;
        m_firstPointCleared = true;
    }

    if (shouldClear) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "{\"x\":%.4f,\"y\":%.4f,\"r\":%.1f,\"t\":%.2f,\"clear\":1}", 
                 norm.x, norm.y, actualPos.rho, actualPos.theta);
        try {
            m_events.send(buffer, "pos", millis());
        } catch (...) {
            // Drop the event on allocation failure to avoid crashing Core 0.
        }
    } else {
        broadcastSinglePosition();
    }
}

void SisyphusWebServer::processPatternQueue() {
    auto state = m_polarControl->getState();
    uint8_t stateValue = static_cast<uint8_t>(state);

    if (stateValue != 2) return; // Only process when IDLE

    // Handle clearing completion for single pattern mode (not playlist)
    if (!m_playlistMode && m_runningClearing && m_pendingPattern.length() > 0) {
        Serial.printf("Single pattern: Starting %s after clearing\r\n", m_pendingPattern.c_str());
        m_polarControl->resetTheta();
        m_currentPattern = m_pendingPattern;
        m_firstPointCleared = false;
        
        m_polarControl->loadAndRunFile(resolvePatternPath(m_pendingPattern));
        m_pendingPattern = "";
        m_runningClearing = false;
        m_singlePatternClearing = false;
        return;
    }

    // Handle single pattern queue
    if (m_hasQueuedPattern) {
        if (m_singlePatternClearing && m_selectedClearing != CLEARING_NONE) {
            Serial.printf("Single pattern: Running clearing before %s\r\n", m_queuedPattern.c_str());
            m_pendingPattern = m_queuedPattern;
            m_runningClearing = true;
            m_hasQueuedPattern = false;
            m_queuedPattern = "";

            m_polarControl->resetTheta();

            ClearingPattern pattern = m_selectedClearing;
            if (pattern == CLEARING_RANDOM) pattern = getRandomClearingPattern();
            
            if (!m_polarControl->startClearing(std_patch::make_unique<ClearingPatternGen>(pattern, m_polarControl->getMaxRho()))) {
                m_runningClearing = false;
                m_singlePatternClearing = false;
                m_polarControl->loadAndRunFile(resolvePatternPath(m_pendingPattern));
                m_currentPattern = m_pendingPattern;
                m_pendingPattern = "";
            }
            return;
        }

        m_currentPattern = m_queuedPattern;
        m_firstPointCleared = false;
        m_polarControl->resetTheta();
        m_polarControl->loadAndRunFile(resolvePatternPath(m_queuedPattern));
        m_hasQueuedPattern = false;
        m_queuedPattern = "";
        return;
    }

    // Handle playlist mode
    if (m_playlistMode) {
        if (m_runningClearing && m_pendingPattern.length() > 0) {
            LOG("Playlist: Starting pattern after clearing: %s\r\n", m_pendingPattern.c_str());
            m_polarControl->resetTheta();
            m_currentPattern = m_pendingPattern;
            m_firstPointCleared = false;
            m_polarControl->loadAndRunFile(resolvePatternPath(m_pendingPattern));
            m_pendingPattern = "";
            m_runningClearing = false;
            return;
        }

        if (m_playlist.hasNext()) {
            NextPatternResult next = m_playlist.getNextPattern();
            if (next.filename.length() > 0) {
                if (next.needsClearing && next.clearingPattern != CLEARING_NONE) {
                    m_pendingPattern = next.filename;
                    m_runningClearing = true;
                    m_polarControl->resetTheta();

                    ClearingPattern pattern = next.clearingPattern;
                    if (pattern == CLEARING_RANDOM) pattern = getRandomClearingPattern();
                    
                    if (!m_polarControl->startClearing(std_patch::make_unique<ClearingPatternGen>(pattern, m_polarControl->getMaxRho()))) {
                        m_runningClearing = false;
                        m_polarControl->loadAndRunFile(resolvePatternPath(next.filename));
                    }
                } else {
                    LOG("Playlist: Starting pattern: %s\r\n", next.filename.c_str());
                    m_polarControl->resetTheta();
                    m_currentPattern = next.filename;
                    m_firstPointCleared = false;
                    m_polarControl->loadAndRunFile(resolvePatternPath(next.filename));
                }
            }
        } else {
            LOG("Playlist complete\r\n");
            m_playlistMode = false;
            m_runningClearing = false;
            m_pendingPattern = "";
        }
    }
}

String SisyphusWebServer::getStateString() {
    return JsonHelpers::getStateString(m_polarControl->getState());
}

void SisyphusWebServer::writeStatusJSON(Print& out) {
    JsonHelpers::writeStatusJSON(out, m_polarControl, m_ledController, m_currentPattern);
}

void SisyphusWebServer::writeFileListJSON(Print& out) {
    JsonHelpers::writeFileListJSON(out);
}

void SisyphusWebServer::writeSystemInfoJSON(Print& out) {
    JsonHelpers::writeSystemInfoJSON(out);
}

void SisyphusWebServer::handleRoot(AsyncWebServerRequest *request) {
    request->send(200, "text/html",
                  reinterpret_cast<const uint8_t *>(WEB_UI_HTML),
                  sizeof(WEB_UI_HTML) - 1);
}

void SisyphusWebServer::handleStatus(AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
    writeStatusJSON(*response);
    request->send(response);
}

void SisyphusWebServer::handlePatternStart(AsyncWebServerRequest *request) {
    auto state = m_polarControl->getState();
    if (state != PolarControl::IDLE) {
        request->send(409, "application/json", "{\"success\":false,\"message\":\"System not idle\"}");
        return;
    }

    if (!request->hasParam("file", true)) {
        request->send(400, "application/json", "{\"success\":false,\"message\":\"Missing parameters\"}");
        return;
    }

    String file = request->getParam("file", true)->value();
    String filePath = resolvePatternPath(file);

    if (!SD.exists(filePath)) {
        request->send(404, "application/json", "{\"success\":false,\"message\":\"File not found\"}");
        return;
    }

    // Check for clearing parameter
    m_singlePatternClearing = false;
    m_selectedClearing = CLEARING_NONE;
    if (request->hasParam("clearing", true)) {
        int clearingType = request->getParam("clearing", true)->value().toInt();
        if (clearingType > 0) {
            m_singlePatternClearing = true;
            m_selectedClearing = static_cast<ClearingPattern>(clearingType);
        }
    }

    m_queuedPattern = file;
    m_hasQueuedPattern = true;

    request->send(200, "application/json", "{\"success\":true,\"message\":\"Pattern started\"}");
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
    bool isEmpty = true;
    if (m_cacheMutex) {
        xSemaphoreTake(m_cacheMutex, portMAX_DELAY);
        isEmpty = m_fileCache.empty();
        xSemaphoreGive(m_cacheMutex);
    }

    // If cache is empty and we are marked dirty, it means we are likely initializing
    if (isEmpty && m_fileListDirty) {
        AsyncWebServerResponse *response = request->beginResponse(503, "application/json", "{\"success\":false,\"message\":\"File list generating...\"}");
        response->addHeader("Retry-After", "1");
        request->send(response);
        return;
    }

    AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
    response->print("{\"files\":[");
    
    if (m_cacheMutex) {
        xSemaphoreTake(m_cacheMutex, portMAX_DELAY);
        for (size_t i = 0; i < m_fileCache.size(); i++) {
            if (i > 0) response->print(",");
            // Manual JSON construction for efficiency
            response->printf("{\"name\":\"%s\",\"size\":%u,\"time\":%u}", 
                m_fileCache[i].name.c_str(), m_fileCache[i].size, m_fileCache[i].time);
        }
        xSemaphoreGive(m_cacheMutex);
    }
    
    response->print("]}");
    request->send(response);
}

void SisyphusWebServer::handleFileUpload(AsyncWebServerRequest *request, String filename,
                                        size_t index, uint8_t *data, size_t len, bool final) {
    if (index == 0) {
        unsigned long now = millis();
        // Allow burst uploads for pattern+image
        if (now - m_lastUploadTime < 100) { 
            // Decrease timeout check to allow simultaneous uploads
        }
        m_lastUploadTime = now;

        // Determine directory name from filename (remove extension)
        String basename = filename;
        int lastDot = basename.lastIndexOf('.');
        if (lastDot != -1) {
            basename = basename.substring(0, lastDot);
        }
        
        // Clean up basename (remove directories if sent in filename)
        int lastSlash = basename.lastIndexOf('/');
        if (lastSlash != -1) {
            basename = basename.substring(lastSlash + 1);
        }
        
        String dirPath = "/patterns/" + basename;
        
        // Delete existing directory only if this is a "new" upload session for this pattern
        // (to avoid deleting it again when the companion image is uploaded right after)
        if (SD.exists(dirPath)) {
            if (now - m_lastUploadTime > 2000) { 
                Serial.print("Deleting existing pattern directory: ");
                Serial.println(dirPath);
                removeDirectoryRecursive(dirPath.c_str());
                SD.mkdir(dirPath);
            }
        } else {
            SD.mkdir(dirPath);
        }

        m_lastUploadTime = now;

        Serial.print("Upload start: ");
        Serial.println(filename);

        String path = dirPath + "/" + filename;
        // No need to SD.remove(path) here anymore if directory was recreated

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
            
            m_fileListDirty = true;

            AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
            JsonDocument doc;
            doc["success"] = true;
            doc["filename"] = filename;
            doc["size"] = index + len;
            serializeJson(doc, *response);
            request->send(response);
        } else {
            request->send(500, "application/json",
                "{\"success\":false,\"message\":\"Upload failed\"}");
        }
    }
}

void SisyphusWebServer::handleFileDelete(AsyncWebServerRequest *request) {
    if (!request->hasParam("file", true)) {
        request->send(400, "application/json", "{\"success\":false,\"message\":\"Missing filename\"}");
        return;
    }

    String filename = request->getParam("file", true)->value();
    if (filename.indexOf("..") != -1) {
        request->send(400, "application/json", "{\"success\":false,\"message\":\"Invalid filename\"}");
        return;
    }

    String basename = filename;
    if (basename.endsWith(".thr")) basename = basename.substring(0, basename.length() - 4);

    String dirPath = "/patterns/" + basename;
    String thrPath = dirPath + "/" + filename;
    String pngPath = dirPath + "/" + basename + ".png";
    bool deleted = false;

    if (SD.exists(dirPath) && SD.open(dirPath).isDirectory()) {
        // Delete entire pattern directory recursively
        deleted = removeDirectoryRecursive(dirPath.c_str());
    } else {
        // Fallback: try flat file structure
        String flatPath = "/patterns/" + filename;
        if (SD.exists(flatPath)) deleted = SD.remove(flatPath);
    }

    if (deleted) {
        m_fileListDirty = true;
        request->send(200, "application/json", "{\"success\":true}");
    } else {
        request->send(404, "application/json", "{\"success\":false,\"message\":\"File not found\"}");
    }
}

void SisyphusWebServer::handleLEDBrightnessGet(AsyncWebServerRequest *request) {
    JsonDocument doc;
    uint8_t brightness = m_ledController->getBrightness();
    doc["brightness"] = map(brightness, 0, 255, 0, 100);

    AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
    serializeJson(doc, *response);
    request->send(response);
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
    doc["speed"] = m_polarControl->getSpeed();

    AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
    serializeJson(doc, *response);
    request->send(response);
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
    AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
    writeSystemInfoJSON(*response);
    request->send(response);
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
    File root = SD.open("/patterns");
    if (!root) {
        request->send(500, "application/json", "{\"success\":false,\"message\":\"Failed to open patterns directory\"}");
        return;
    }

    int count = 0;
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            String filename = String(file.name());
            if (filename.startsWith("/")) filename = filename.substring(1);

            if (filename.endsWith(".thr")) {
                m_playlist.addPattern(filename);
                count++;
            }
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();

    AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
    response->printf("{\"success\":true,\"count\":%d}", count);
    request->send(response);
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
    AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
    response->print("{\"loop\":");
    response->print(m_playlist.isLoop() ? "true" : "false");
    response->print(",\"playing\":");
    response->print(m_playlistMode ? "true" : "false");
    response->printf(",\"currentIndex\":%d", m_playlist.getCurrentIndex());
    response->printf(",\"count\":%d", m_playlist.count());
    response->print(",\"clearingEnabled\":");
    response->print(m_playlist.isClearingEnabled() ? "true" : "false");
    response->print(",\"items\":[");

    bool first = true;
    for (int i = 0; i < m_playlist.count(); i++) {
        const PlaylistItem& item = m_playlist.getItem(i);
        JsonDocument entry;
        entry["filename"] = item.filename;
        if (!first) response->print(',');
        serializeJson(entry, *response);
        first = false;
    }

    response->print("]}");
    request->send(response);
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

void SisyphusWebServer::handlePlaylistLoop(AsyncWebServerRequest *request) {
    if (!request->hasParam("enabled", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing enabled parameter\"}");
        return;
    }

    String enabledStr = request->getParam("enabled", true)->value();
    bool enabled = (enabledStr == "true" || enabledStr == "1");

    m_playlist.setLoop(enabled);

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handlePlaylistShuffle(AsyncWebServerRequest *request) {
    m_playlist.shuffle();
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
    AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
    response->print("{\"playlists\":[");
    bool first = true;

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
                    JsonDocument entry;
                    entry.set(filename);
                    if (!first) response->print(',');
                    serializeJson(entry, *response);
                    first = false;
                }
            }
            file.close();
            file = root.openNextFile();
        }
        root.close();
    }

    response->print("]}");
    request->send(response);
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

void SisyphusWebServer::handlePlaylistMove(AsyncWebServerRequest *request) {
    if (!request->hasParam("from", true) || !request->hasParam("to", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing from/to parameters\"}");
        return;
    }

    int fromIndex = request->getParam("from", true)->value().toInt();
    int toIndex = request->getParam("to", true)->value().toInt();

    m_playlist.movePattern(fromIndex, toIndex);

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handlePlaylistSkipTo(AsyncWebServerRequest *request) {
    if (!request->hasParam("index", true)) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Missing index parameter\"}");
        return;
    }

    int index = request->getParam("index", true)->value().toInt();

    // Stop current pattern and skip to the requested one
    m_polarControl->stop();
    m_playlist.skipToIndex(index);

    // If playlist mode is active, the next pattern will be picked up automatically
    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handlePlaylistNext(AsyncWebServerRequest *request) {
    if (m_playlist.count() == 0) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Playlist is empty\"}");
        return;
    }

    // Stop current pattern and skip to next
    m_polarControl->stop();
    m_playlist.skipNext();

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handlePlaylistPrev(AsyncWebServerRequest *request) {
    if (m_playlist.count() == 0) {
        request->send(400, "application/json",
            "{\"success\":false,\"message\":\"Playlist is empty\"}");
        return;
    }

    // Stop current pattern and skip to previous
    m_polarControl->stop();
    m_playlist.skipPrevious();

    request->send(200, "application/json", "{\"success\":true}");
}

// ============================================================================
// Position and Path Tracking
// ============================================================================

void SisyphusWebServer::handlePosition(AsyncWebServerRequest *request) {
    // Get actual position from stepper motors (used for display)
    PolarCord_t actualPos = m_polarControl->getActualPosition();
    float maxRho = m_polarControl->getMaxRho();

    // Debug log (throttled)
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 5000) {
        lastLog = millis();
        Serial.printf("API Position: rho=%.2f theta=%.2f\r\n", actualPos.rho, actualPos.theta);
    }

    AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);

    if (isnan(actualPos.rho) || isnan(actualPos.theta)) {
        response->print("{\"current\":null}");
    } else {
        // Convert to normalized Cartesian coordinates (0-1 range)
        CartesianCord_t norm = PolarUtils::toNormalizedCartesian(actualPos, maxRho);

        // Normalize to 0-1 range (center at 0.5,0.5)
        response->printf("{\"current\":{\"x\":%.4f,\"y\":%.4f,\"rho\":%.2f,\"theta\":%.2f}}",
            norm.x, norm.y, actualPos.rho, actualPos.theta);
    }

    request->send(response);
}

// ============================================================================
// Tuning Handlers
// ============================================================================

// Helper to serialize DriverSettings to JSON
static void driverSettingsToJson(JsonObject& obj, const DriverSettings& settings) {
    // Current settings (mA)
    obj["runCurrent"] = settings.runCurrent;
    obj["holdCurrent"] = settings.holdCurrent;
    obj["holdDelay"] = settings.holdDelay;

    // Microstepping
    obj["microsteps"] = settings.microsteps;

    // StealthChop settings
    obj["stealthChopEnabled"] = settings.stealthChopEnabled;
    obj["stealthChopThreshold"] = settings.stealthChopThreshold;

    // CoolStep settings
    obj["coolStepEnabled"] = settings.coolStepEnabled;
    obj["coolStepLowerThreshold"] = settings.coolStepLowerThreshold;
    obj["coolStepUpperThreshold"] = settings.coolStepUpperThreshold;
    obj["coolStepCurrentIncrement"] = settings.coolStepCurrentIncrement;
    obj["coolStepMeasurementCount"] = settings.coolStepMeasurementCount;
    obj["coolStepThreshold"] = settings.coolStepThreshold;
}

// Helper to parse DriverSettings from request
static void parseDriverSettings(AsyncWebServerRequest *request, DriverSettings& settings) {
    // Current settings (mA)
    if (request->hasParam("runCurrent", true))
        settings.runCurrent = request->getParam("runCurrent", true)->value().toInt();
    if (request->hasParam("holdCurrent", true))
        settings.holdCurrent = request->getParam("holdCurrent", true)->value().toInt();
    if (request->hasParam("holdDelay", true))
        settings.holdDelay = request->getParam("holdDelay", true)->value().toInt();

    // Microstepping
    if (request->hasParam("microsteps", true))
        settings.microsteps = request->getParam("microsteps", true)->value().toInt();

    // StealthChop settings
    if (request->hasParam("stealthChopEnabled", true))
        settings.stealthChopEnabled = request->getParam("stealthChopEnabled", true)->value() == "true";
    if (request->hasParam("stealthChopThreshold", true))
        settings.stealthChopThreshold = request->getParam("stealthChopThreshold", true)->value().toInt();

    // CoolStep settings
    if (request->hasParam("coolStepEnabled", true))
        settings.coolStepEnabled = request->getParam("coolStepEnabled", true)->value() == "true";
    if (request->hasParam("coolStepLowerThreshold", true))
        settings.coolStepLowerThreshold = request->getParam("coolStepLowerThreshold", true)->value().toInt();
    if (request->hasParam("coolStepUpperThreshold", true))
        settings.coolStepUpperThreshold = request->getParam("coolStepUpperThreshold", true)->value().toInt();
    if (request->hasParam("coolStepCurrentIncrement", true))
        settings.coolStepCurrentIncrement = request->getParam("coolStepCurrentIncrement", true)->value().toInt();
    if (request->hasParam("coolStepMeasurementCount", true))
        settings.coolStepMeasurementCount = request->getParam("coolStepMeasurementCount", true)->value().toInt();
    if (request->hasParam("coolStepThreshold", true))
        settings.coolStepThreshold = request->getParam("coolStepThreshold", true)->value().toInt();
}

void SisyphusWebServer::handleTuningGet(AsyncWebServerRequest *request) {
    JsonDocument doc;

    // Motion settings
    const MotionSettings& motion = m_polarControl->getMotionSettings();
    JsonObject motionObj = doc["motion"].to<JsonObject>();
    motionObj["rMaxVelocity"] = motion.rMaxVelocity;
    motionObj["rMaxAccel"] = motion.rMaxAccel;
    motionObj["rMaxJerk"] = motion.rMaxJerk;
    motionObj["tMaxVelocity"] = motion.tMaxVelocity;
    motionObj["tMaxAccel"] = motion.tMaxAccel;
    motionObj["tMaxJerk"] = motion.tMaxJerk;

    // Driver settings
    const DriverSettings& theta = m_polarControl->getThetaDriverSettings();
    JsonObject thetaObj = doc["thetaDriver"].to<JsonObject>();
    driverSettingsToJson(thetaObj, theta);

    const DriverSettings& rho = m_polarControl->getRhoDriverSettings();
    JsonObject rhoObj = doc["rhoDriver"].to<JsonObject>();
    driverSettingsToJson(rhoObj, rho);

    AsyncResponseStream *response = request->beginResponseStream("application/json", kResponseBufferSize);
    serializeJson(doc, *response);
    request->send(response);
}

void SisyphusWebServer::handleTuningMotionSet(AsyncWebServerRequest *request) {
    MotionSettings settings = m_polarControl->getMotionSettings();

    if (request->hasParam("rMaxVelocity", true))
        settings.rMaxVelocity = request->getParam("rMaxVelocity", true)->value().toFloat();
    if (request->hasParam("rMaxAccel", true))
        settings.rMaxAccel = request->getParam("rMaxAccel", true)->value().toFloat();
    if (request->hasParam("rMaxJerk", true))
        settings.rMaxJerk = request->getParam("rMaxJerk", true)->value().toFloat();
    if (request->hasParam("tMaxVelocity", true))
        settings.tMaxVelocity = request->getParam("tMaxVelocity", true)->value().toFloat();
    if (request->hasParam("tMaxAccel", true))
        settings.tMaxAccel = request->getParam("tMaxAccel", true)->value().toFloat();
    if (request->hasParam("tMaxJerk", true))
        settings.tMaxJerk = request->getParam("tMaxJerk", true)->value().toFloat();

    m_polarControl->setMotionSettings(settings);
    m_polarControl->saveTuningSettings();

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handleTuningThetaDriverSet(AsyncWebServerRequest *request) {
    DriverSettings settings = m_polarControl->getThetaDriverSettings();
    parseDriverSettings(request, settings);

    m_polarControl->setThetaDriverSettings(settings);
    m_polarControl->saveTuningSettings();

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handleTuningRhoDriverSet(AsyncWebServerRequest *request) {
    DriverSettings settings = m_polarControl->getRhoDriverSettings();
    parseDriverSettings(request, settings);

    m_polarControl->setRhoDriverSettings(settings);
    m_polarControl->saveTuningSettings();

    request->send(200, "application/json", "{\"success\":true}");
}

void SisyphusWebServer::handleTuningTestThetaContinuous(AsyncWebServerRequest *request) {
    auto state = m_polarControl->getState();
    if (state != PolarControl::IDLE) {
        request->send(409, "application/json", "{\"success\":false,\"message\":\"System must be idle\"}");
        return;
    }
    m_currentPattern = "";
    request->send(200, "application/json", "{\"success\":true,\"message\":\"Test started\"}");
    m_polarControl->testThetaContinuous();
}

void SisyphusWebServer::handleTuningTestThetaStress(AsyncWebServerRequest *request) {
    auto state = m_polarControl->getState();
    if (state != PolarControl::IDLE) {
        request->send(409, "application/json", "{\"success\":false,\"message\":\"System must be idle\"}");
        return;
    }
    m_currentPattern = "";
    request->send(200, "application/json", "{\"success\":true,\"message\":\"Test started\"}");
    m_polarControl->testThetaStress();
}

void SisyphusWebServer::handleTuningTestRhoContinuous(AsyncWebServerRequest *request) {
    auto state = m_polarControl->getState();
    if (state != PolarControl::IDLE) {
        request->send(409, "application/json", "{\"success\":false,\"message\":\"System must be idle\"}");
        return;
    }
    m_currentPattern = "";
    request->send(200, "application/json", "{\"success\":true,\"message\":\"Test started\"}");
    m_polarControl->testRhoContinuous();
}

void SisyphusWebServer::handleTuningTestRhoStress(AsyncWebServerRequest *request) {
    auto state = m_polarControl->getState();
    if (state != PolarControl::IDLE) {
        request->send(409, "application/json", "{\"success\":false,\"message\":\"System must be idle\"}");
        return;
    }
    m_currentPattern = "";
    request->send(200, "application/json", "{\"success\":true,\"message\":\"Test started\"}");
    m_polarControl->testRhoStress();
}
