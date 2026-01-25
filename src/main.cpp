#include "Arduino.h"
#include <SDCard.hpp>

#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <PolarControl.hpp>
#include <SisyphusWebServer.hpp>
#include <LEDController.hpp>
#include <ClearingPatternGen.hpp>

#define AP_SSID "SisyphusTable"
#define AP_PWD "sandpatterns"

#define MOTOR_CORE 1
#define WEB_CORE 0

// Static IP configuration
#define STATIC_IP_BASE IPAddress(100, 76, 149, 200)
#define STATIC_GATEWAY IPAddress(100, 76, 149, 1)
#define STATIC_SUBNET IPAddress(255, 255, 255, 0)
#define STATIC_DNS IPAddress(100, 76, 149, 1)

PolarControl polarControl;
SisyphusWebServer webServer(80);
LEDController ledController(2);

TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;

volatile unsigned long g_motorLoopCount = 0;
volatile unsigned long g_lastPositionPrint = 0;

void motorTask(void *parameter) {
    LOG("Motor task started on Core %d\r\n", xPortGetCoreID());

    unsigned long lastPrint = millis();
    unsigned long loopCount = 0;

    while (true) {
        bool isBusy = polarControl.processNextMove();
        loopCount++;

        unsigned long now = millis();
        if (now - lastPrint >= 1000) {
            PolarCord_t pos = polarControl.getCurrentPosition();
            float loopsPerSec = (float)loopCount * 1000.0 / (now - lastPrint);
            auto state = polarControl.getState();
            
            uint32_t queueDepth, underruns;
            polarControl.getDiagnostics(queueDepth, underruns);

            uint32_t maxProc, maxInt, avgGen;
            polarControl.getProfileData(maxProc, maxInt, avgGen);

            if (state == PolarControl::RUNNING || state == PolarControl::CLEARING || state == PolarControl::PREPARING) {
                LOG("[MOTOR Core%d] Pos: ρ=%.0f θ=%.0f° | Q: %u | UR: %u | Loop: %.0f/s | MaxProc: %uus | MaxInt: %uus | AvgGen: %uus\r\n",
                              xPortGetCoreID(), pos.rho, pos.theta * 180.0 / PI, queueDepth, underruns, loopsPerSec, maxProc, maxInt, avgGen);
            }

            loopCount = 0;
            lastPrint = now;
        }

        if (!isBusy) {
            vTaskDelay(1);
        } else {
            vTaskDelay(0); // Yield to same-priority tasks on Core 1
        }
    }
}

void webTask(void *parameter) {
    LOG("Web logic task started on Core %d\r\n", xPortGetCoreID());
    while (true) {
        webServer.loop();
        ArduinoOTA.handle();
        vTaskDelay(10); // Run at ~100Hz, sufficient for UI updates
    }
}

void setup() {
    // ... (rest of setup unchanged until task creation)
    Serial.begin(115200);
    delay(500);

    LOG("\n\n=== Sisyphus Table Starting ===\r\n");

    // Initialize SD Card
    if (!initSDCard()) {
        LOG("WARNING: Running without SD card storage!\r\n");
    } else {
        listSDFiles();
    }

    // WiFi Setup
    LOG("Setting up WiFi...\r\n");
    WiFi.mode(WIFI_STA);
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);

    // Configure static IP
    wm.setSTAStaticIPConfig(STATIC_IP_BASE, STATIC_GATEWAY, STATIC_SUBNET, STATIC_DNS);
    LOG("Requesting static IP: %s\r\n", STATIC_IP_BASE.toString().c_str());

    if (!wm.autoConnect(AP_SSID, AP_PWD)) {
        LOG("Failed to connect to WiFi\r\n");
        ESP.restart();
    }

    LOG("WiFi connected!\r\n");
    LOG("IP Address: %s\r\n", WiFi.localIP().toString().c_str());
    LOG("SSID: %s\r\n", WiFi.SSID().c_str());

    // Setup OTA updates
    ArduinoOTA.setHostname("sisyphus");
    ArduinoOTA.setPassword("sandpatterns");
    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        LOG("OTA Start: %s\r\n", type.c_str());
        // Stop motors during OTA update
        polarControl.stop();
    });
    ArduinoOTA.onEnd([]() {
        LOG("\nOTA End - Rebooting...\r\n");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
        // Don't LOG progress to web, too much traffic
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error); // Keep Serial for errors if web logger fails
        if (error == OTA_AUTH_ERROR) LOG("OTA Auth Failed\r\n");
        else if (error == OTA_BEGIN_ERROR) LOG("OTA Begin Failed\r\n");
        else if (error == OTA_CONNECT_ERROR) LOG("OTA Connect Failed\r\n");
        else if (error == OTA_RECEIVE_ERROR) LOG("OTA Receive Failed\r\n");
        else if (error == OTA_END_ERROR) LOG("OTA End Failed\r\n");
    });
    ArduinoOTA.begin();
    LOG("OTA updates enabled\r\n");

    // Initialize hardware
    LOG("Initializing motors...\r\n");
    polarControl.begin();
    polarControl.setupDrivers();
    polarControl.home();

    // Initialize LED controller
    LOG("Initializing LED controller...\r\n");
    ledController.begin();

    // Start web server
    LOG("Starting web server...\r\n");
    webServer.begin(&polarControl, &ledController);

    LOG("\n=== System Ready ===\r\n");
    LOG("Access web interface at: http://%s\r\n", WiFi.localIP().toString().c_str());

    // Create motor task on Core 1
    LOG("Starting motor task on Core 1...\r\n");
    xTaskCreatePinnedToCore(
        motorTask,
        "MotorTask",
        16384,
        NULL,
        1,
        &motorTaskHandle,
        MOTOR_CORE
    );

    // Create web logic task on Core 0
    LOG("Starting web logic task on Core 0...\r\n");
    xTaskCreatePinnedToCore(
        webTask,
        "WebTask",
        16384,
        NULL,
        1,
        &webTaskHandle,
        WEB_CORE
    );

    // Initial speed
    polarControl.setSpeed(5);

    LOG("Main setup done on Core %d\r\n", xPortGetCoreID());
}

void loop() {
    vTaskDelete(NULL); // Delete the default loop task
}
