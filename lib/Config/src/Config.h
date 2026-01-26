#pragma once

#include "Arduino.h"

namespace Config {
// WiFi AP fallback credentials for captive portal mode.
static constexpr const char kApSsid[] = "SisyphusTable";
static constexpr const char kApPassword[] = "sandpatterns";

// Core affinity for the FreeRTOS tasks.
static constexpr int kMotorCore = 1;
static constexpr int kWebCore = 0;

// Config portal timeout for WiFiManager in seconds.
static constexpr uint32_t kWifiPortalTimeoutSec = 180;

// Static IP defaults for STA mode.
static const IPAddress kStaticIpBase(100, 76, 149, 200);
static const IPAddress kStaticGateway(100, 76, 149, 1);
static const IPAddress kStaticSubnet(255, 255, 255, 0);
static const IPAddress kStaticDns(100, 76, 149, 1);

// OTA identity and auth.
static constexpr const char kOtaHostname[] = "sisyphus";
static constexpr const char kOtaPassword[] = "sandpatterns";

// Web server port and attached LED count.
static constexpr uint16_t kWebServerPort = 80;
static constexpr uint8_t kLedCount = 2;

// Step/dir pin mapping.
static constexpr uint8_t kRhoStepPin = 33;
static constexpr uint8_t kRhoDirPin = 25;
static constexpr uint8_t kThetaStepPin = 32;
static constexpr uint8_t kThetaDirPin = 22;

// TMC2209 UART pins and driver addresses.
static constexpr uint8_t kUartRxPin = 27;
static constexpr uint8_t kUartTxPin = 26;
static constexpr uint8_t kRhoDriverAddress = 1;
static constexpr uint8_t kRhoCDriverAddress = 0;
static constexpr uint8_t kThetaDriverAddress = 2;
}
