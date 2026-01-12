#include "Arduino.h"
#include "LittleFS.h"

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
LEDController ledController(18);

// Motor task handle
TaskHandle_t motorTaskHandle = NULL;

// Profiling variables (volatile for cross-core access)
volatile unsigned long g_motorLoopCount = 0;
volatile unsigned long g_motorTimeTotal = 0;
volatile unsigned long g_motorTimeMax = 0;
volatile unsigned long g_lastMotorProfilePrint = 0;

// Motor processing task - runs on Core 1
void motorTask(void *parameter) {
  Serial.printf("Motor task started on Core %d\n", xPortGetCoreID());

  unsigned long lastPrint = millis();
  unsigned long loopCount = 0;
  unsigned long timeTotal = 0;
  unsigned long timeMax = 0;

  while (true) {
    unsigned long start = micros();

    // Process motor moves
    polarControl.processNextMove();

    unsigned long elapsed = micros() - start;

    // Accumulate stats
    loopCount++;
    timeTotal += elapsed;
    if (elapsed > timeMax) timeMax = elapsed;

    // Print stats every second
    unsigned long now = millis();
    if (now - lastPrint >= 1000) {
      float avgTime = (float)timeTotal / loopCount;
      float loopsPerSec = (float)loopCount * 1000.0 / (now - lastPrint);

      Serial.printf("\r[MOTOR Core%d] Loops/s: %.1f | avg=%.0fus max=%luus                    ",
        xPortGetCoreID(), loopsPerSec, avgTime, timeMax);

      // Reset stats
      loopCount = 0;
      timeTotal = 0;
      timeMax = 0;
      lastPrint = now;
    }

    // Small delay to prevent watchdog issues
    vTaskDelay(1); // 1 tick = ~1ms
  }
}

void listFiles() {
  Serial.println("\n=== Files on LittleFS ===");
  File root = LittleFS.open("/");
  if (!root) {
    Serial.println("ERROR: Failed to open root directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("ERROR: Root is not a directory");
    return;
  }

  File file = root.openNextFile();
  int count = 0;
  while (file) {
    if (!file.isDirectory()) {
      Serial.print("  ");
      Serial.print(file.name());
      Serial.print(" (");
      Serial.print(file.size());
      Serial.println(" bytes)");
      count++;
    }
    file = root.openNextFile();
  }
  Serial.print("Total files: ");
  Serial.println(count);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);  // Wait for serial to initialize

  Serial.println("\n\n=== Sisyphus Table Starting ===");

  // Initialize LittleFS
  if (!LittleFS.begin(true)) {  // true = format on failure
    Serial.println("ERROR: LittleFS mount failed!");
  } else {
    Serial.println("LittleFS mounted successfully");
    Serial.print("Total space: ");
    Serial.print(LittleFS.totalBytes() / 1024);
    Serial.println(" KB");
    Serial.print("Used space: ");
    Serial.print(LittleFS.usedBytes() / 1024);
    Serial.println(" KB");

    // List all files
    listFiles();
  }

  // WiFi Setup
  Serial.println("Setting up WiFi...");
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setConfigPortalTimeout(180); // 3 minute timeout

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
  polarControl.verifyDriverConfig();

  // Initialize LED controller
  Serial.println("Initializing LED controller...");
  ledController.begin();

  // Start web server BEFORE clearing so user can monitor
  Serial.println("Starting web server...");
  webServer.begin(&polarControl, &ledController);

  Serial.println("\n=== System Ready ===");
  Serial.println("Access web interface at: http://" + WiFi.localIP().toString());

  // Create motor task on Core 1 (dedicated core for motor processing)
  Serial.println("Starting motor task on Core 1...");
  xTaskCreatePinnedToCore(
    motorTask,          // Task function
    "MotorTask",        // Task name
    8192,               // Stack size (bytes)
    NULL,               // Parameters
    1,                  // Priority (1 = low, higher = more priority)
    &motorTaskHandle,   // Task handle
    MOTOR_CORE          // Core to run on
  );

  // Run startup clearing pattern (motor task handles processing)
  Serial.println("Running startup clearing pattern...");
  ClearingPatternGen *startupClearing = new ClearingPatternGen(SPIRAL_OUTWARD, 450.0);
  if (polarControl.start(startupClearing)) {
    Serial.println("Startup clearing pattern started");
    // Wait for clearing to complete, web server handles requests
    while (polarControl.getState() != 2) { // 2 = IDLE
      webServer.loop(); // Allow web interface to work during clearing
      delay(10);
    }
    Serial.println("\nStartup clearing pattern complete");
  } else {
    delete startupClearing;
    Serial.println("Failed to start clearing pattern");
  }

  Serial.printf("Main loop running on Core %d\n", xPortGetCoreID());
}

// Web server loop - runs on Core 0 (with WiFi)
void loop() {
  // Only handle web server - motor processing is on Core 1
  webServer.loop();

  // Small delay to prevent watchdog issues
  delay(1);
}
