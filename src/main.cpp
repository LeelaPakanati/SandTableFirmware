#include "Arduino.h"
#include <SDCard.hpp>

#include <WiFiManager.h>
#include <PolarControl.hpp>
#include <SisyphusWebServer.hpp>
#include <LEDController.hpp>
#include <ClearingPatternGen.hpp>

#define AP_SSID "SisyphusTable"
#define AP_PWD "sandpatterns"

#define MOTOR_CORE 1
#define WEB_CORE 0

PolarControl polarControl;
SisyphusWebServer webServer(80);
LEDController ledController(2);

TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;

volatile unsigned long g_motorLoopCount = 0;
volatile unsigned long g_lastPositionPrint = 0;

void motorTask(void *parameter) {
    Serial.printf("Motor task started on Core %d\n", xPortGetCoreID());

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

            Serial.printf("[MOTOR Core%d] Position: ρ=%.1fmm θ=%.1f° | State: %d | Loops/s: %.1f\r\n",
                          xPortGetCoreID(), pos.rho, pos.theta * 180.0 / PI, static_cast<uint8_t>(state), loopsPerSec);

            loopCount = 0;
            lastPrint = now;
        }

        if (!isBusy) {
            vTaskDelay(1);
        }
    }
}

void webTask(void *parameter) {
    Serial.printf("Web logic task started on Core %d\n", xPortGetCoreID());
    while (true) {
        webServer.loop();
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

    if (!wm.autoConnect(AP_SSID, AP_PWD)) {
        Serial.println("Failed to connect to WiFi");
        ESP.restart();
    }

    Serial.println("WiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

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

    Serial.printf("Main setup done on Core %d\n", xPortGetCoreID());
}

void loop() {
    vTaskDelete(NULL); // Delete the default loop task
}
