#pragma once
#include <ArduinoJson.h>
#include <SD.h>
#include <Print.h>
#include "PolarControl.hpp"
#include "LEDController.hpp"

class JsonHelpers {
public:
    static void writeStatusJSON(Print& out, PolarControl* polarControl, LEDController* ledController, const String& currentPattern) {
        String state = getStateString(polarControl->getState());
        int progress = polarControl->getProgressPercent();
        uint8_t brightness = ledController->getBrightness();
        int brightnessPercent = map(brightness, 0, 255, 0, 100);
        uint32_t heap = ESP.getFreeHeap();
        uint32_t uptime = millis() / 1000;

        out.print("{\"state\":\"");
        out.print(state);
        out.print("\",\"currentPattern\":\"");
        out.print(currentPattern);
        out.print("\",\"progress\":");
        out.print(progress);
        out.print(",\"ledBrightness\":");
        out.print(brightnessPercent);
        out.print(",\"heap\":");
        out.print(heap);
        out.print(",\"uptime\":");
        out.print(uptime);
        out.print("}");
    }

    static void writeFileListJSON(Print& out) {
        out.print("{\"files\":[");
        bool first = true;

        File root = SD.open("/patterns");
        if (root) {
            File file = root.openNextFile();
            while (file) {
                String name = String(file.name());
                if (name.startsWith("/")) name = name.substring(1);

                if (!file.isDirectory()) {
                    if (name.endsWith(".thr")) {
                        JsonDocument entry;
                        entry["name"] = name;
                        entry["size"] = file.size();
                        entry["time"] = file.getLastWrite();
                        if (!first) out.print(',');
                        serializeJson(entry, out);
                        first = false;
                    }
                } else {
                    String patternFile = name + ".thr";
                    String innerPath = "/patterns/" + name + "/" + patternFile;
                    if (SD.exists(innerPath)) {
                        File innerFile = SD.open(innerPath);
                        JsonDocument entry;
                        entry["name"] = patternFile;
                        entry["size"] = innerFile.size();
                        entry["time"] = innerFile.getLastWrite();
                        if (!first) out.print(',');
                        serializeJson(entry, out);
                        first = false;
                        innerFile.close();
                    }
                }
                file.close();
                file = root.openNextFile();
            }
            root.close();
        }
        out.print("]}");
    }

    static void writeSystemInfoJSON(Print& out) {
        JsonDocument doc;
        doc["heap"] = ESP.getFreeHeap();
        doc["uptime"] = millis() / 1000;

        JsonObject wifi = doc["wifi"].to<JsonObject>();
        wifi["ssid"] = WiFi.SSID();
        wifi["ip"] = WiFi.localIP().toString();
        wifi["rssi"] = WiFi.RSSI();
        serializeJson(doc, out);
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
            case 7: return "PREPARING";
            default: return "UNKNOWN";
        }
    }
};
