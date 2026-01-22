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
    Serial.printf("Motor task started on Core %d\r\n", xPortGetCoreID());

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

            Serial.printf("[MOTOR Core%d] Position: ρ=%.1fmm θ=%.1f° | State: %d | Loops/s: %.1f | Q: %u | UR: %u\r\n",
                          xPortGetCoreID(), pos.rho, pos.theta * 180.0 / PI, static_cast<uint8_t>(state), loopsPerSec, queueDepth, underruns);

            loopCount = 0;
            lastPrint = now;
        }

        if (!isBusy) {
            vTaskDelay(1);
        }
    }
}

void webTask(void *parameter) {
    Serial.printf("Web logic task started on Core %d\r\n", xPortGetCoreID());
    while (true) {
        webServer.loop();
        ArduinoOTA.handle();
        vTaskDelay(10); // Run at ~100Hz, sufficient for UI updates
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n\n=== Sisyphus Table Starting ===");

    // Initialize SD Card
    if (!initSDCard()) {
        Serial.println("WARNING: Running without SD card storage!");
    } else {
        listSDFiles();
    }

    // WiFi Setup
    Serial.println("Setting up WiFi...");
    WiFi.mode(WIFI_STA);
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);

    // Configure static IP
    wm.setSTAStaticIPConfig(STATIC_IP_BASE, STATIC_GATEWAY, STATIC_SUBNET, STATIC_DNS);
    Serial.printf("Requesting static IP: %s\r\n", STATIC_IP_BASE.toString().c_str());

    if (!wm.autoConnect(AP_SSID, AP_PWD)) {
        Serial.println("Failed to connect to WiFi");
        ESP.restart();
    }

    Serial.println("WiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // Setup OTA updates
    ArduinoOTA.setHostname("sisyphus");
    ArduinoOTA.setPassword("sandpatterns");
    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("OTA Start: " + type);
        // Stop motors during OTA update
        polarControl.stop();
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End - Rebooting...");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("OTA updates enabled");

    // Initialize hardware
    Serial.println("Initializing motors...");
    polarControl.begin();
    polarControl.setupDrivers();
    polarControl.home();

    // Initialize LED controller
    Serial.println("Initializing LED controller...");
    ledController.begin();

    // Start web server
    Serial.println("Starting web server...");
    webServer.begin(&polarControl, &ledController);

    Serial.println("\n=== System Ready ===");
    Serial.println("Access web interface at: http://" + WiFi.localIP().toString());

    // Create motor task on Core 1
    Serial.println("Starting motor task on Core 1...");
    xTaskCreatePinnedToCore(
        motorTask,
        "MotorTask",
        8192,
        NULL,
        1,
        &motorTaskHandle,
        MOTOR_CORE
    );

    // Create web logic task on Core 0
    Serial.println("Starting web logic task on Core 0...");
    xTaskCreatePinnedToCore(
        webTask,
        "WebTask",
        4096,
        NULL,
        1,
        &webTaskHandle,
        WEB_CORE
    );

    // Initial speed
    polarControl.setSpeed(5);

    Serial.printf("Main setup done on Core %d\r\n", xPortGetCoreID());
}

void loop() {
    vTaskDelete(NULL); // Delete the default loop task
}
