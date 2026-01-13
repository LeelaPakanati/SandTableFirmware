#include "Arduino.h"
#include <SDCard.hpp>

#include <WiFiManager.h>
#include <PolarControl.hpp>
#include <SisyphusWebServer.hpp>
#include <LEDController.hpp>
#include <ClearingPatternGen.hpp>

#define AP_SSID "SisyphusTable"
#define AP_PWD "sandpatterns"

// Core assignments
#define MOTOR_CORE 1    // Dedicated core for motor processing
#define WEB_CORE 0      // Core for web server (shares with WiFi)

PolarControl polarControl;
SisyphusWebServer webServer(80);
LEDController ledController(2);

// Motor task handle
TaskHandle_t motorTaskHandle = NULL;

// Position tracking variables (volatile for cross-core access)
volatile unsigned long g_motorLoopCount = 0;
volatile unsigned long g_lastPositionPrint = 0;

// Motor processing task - runs on Core 1
void motorTask(void *parameter) {
    Serial.printf("Motor task started on Core %d\n", xPortGetCoreID());

    unsigned long lastPrint = millis();
    unsigned long loopCount = 0;

    while (true) {
        // Process motor moves
        bool isRunning = polarControl.processNextMove();
        if (loopCount % 1000 == 0) {
            Serial.printf("DEBUG: processNextMove() returned: %d\n", isRunning);
        }

        loopCount++;

        // Print position every second
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

        vTaskDelay(1);
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
    polarControl.setupDrivers();
    polarControl.home();

    // Initialize LED controller
    Serial.println("Initializing LED controller...");
    ledController.begin();

    // Start web server BEFORE clearing so user can monitor
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

    // Run startup clearing pattern
    Serial.println("Running startup clearing pattern...");
    ClearingPatternGen *startupClearing = new ClearingPatternGen(SPIRAL_OUTWARD, 450.0);
    if (polarControl.start(startupClearing)) {
        Serial.println("Startup clearing pattern started");
        while (polarControl.getState() != 2) {
            webServer.loop();
            delay(10);
        }
        Serial.println("\nStartup clearing pattern complete");
    } else {
        delete startupClearing;
        Serial.println("Failed to start clearing pattern");
    }

    Serial.printf("Main loop running on Core %d\n", xPortGetCoreID());
}

void loop() {
    webServer.loop();
    delay(1);
}
