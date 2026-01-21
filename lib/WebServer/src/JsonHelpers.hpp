#pragma once
#include <ArduinoJson.h>
#include <SD.h>
#include "PolarControl.hpp"
#include "LEDController.hpp"

class JsonHelpers {
public:
    static String buildStatusJSON(PolarControl* polarControl, LEDController* ledController, const String& currentPattern) {
        JsonDocument doc;
        doc["state"] = getStateString(polarControl->getState());
        doc["currentPattern"] = currentPattern;
        uint8_t brightness = ledController->getBrightness();
        doc["ledBrightness"] = map(brightness, 0, 255, 0, 100);
        doc["heap"] = ESP.getFreeHeap();
        doc["uptime"] = millis() / 1000;

        String output;
        serializeJson(doc, output);
        return output;
    }

    static String buildFileListJSON() {
        JsonDocument doc;
        JsonArray files = doc["files"].to<JsonArray>();

        File root = SD.open("/patterns");
        if (root) {
            File file = root.openNextFile();
            while (file) {
                String name = String(file.name());
                if (name.startsWith("/")) name = name.substring(1);

                if (!file.isDirectory()) {
                    if (name.endsWith(".thr")) {
                        JsonObject fileObj = files.add<JsonObject>();
                        fileObj["name"] = name;
                        fileObj["size"] = file.size();
                    }
                } else {
                    String patternFile = name + ".thr";
                    String innerPath = "/patterns/" + name + "/" + patternFile;
                    if (SD.exists(innerPath)) {
                        JsonObject fileObj = files.add<JsonObject>();
                        fileObj["name"] = patternFile;
                        fileObj["size"] = 0; 
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

    static String buildSystemInfoJSON() {
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

    static String getStateString(PolarControl::State_t state) {
        switch (static_cast<uint8_t>(state)) {
            case 0: return "UNINITIALIZED";
            case 1: return "INITIALIZED";
            case 2: return "IDLE";
            case 3: return "RUNNING";
            case 4: return "PAUSED";
            case 5: return "STOPPING";
            case 6: return "CLEARING";
            default: return "UNKNOWN";
        }
    }
};
