#pragma once

#include "Arduino.h"

namespace Config {
// WiFi AP fallback credentials for captive portal mode.
inline constexpr const char kApSsid[] = "SisyphusTable";
inline constexpr const char kApPassword[] = "sandpatterns";

// Core affinity for the FreeRTOS tasks.
inline constexpr int kMotorCore = 1;
inline constexpr int kWebCore = 0;

// Config portal timeout for WiFiManager in seconds.
inline constexpr uint32_t kWifiPortalTimeoutSec = 180;

// Static IP defaults for STA mode.
inline const IPAddress kStaticIpBase(100, 76, 149, 200);
inline const IPAddress kStaticGateway(100, 76, 149, 1);
inline const IPAddress kStaticSubnet(255, 255, 255, 0);
inline const IPAddress kStaticDns(100, 76, 149, 1);

// OTA identity and auth.
inline constexpr const char kOtaHostname[] = "sisyphus";
inline constexpr const char kOtaPassword[] = "sandpatterns";

// Web server port and attached LED count.
inline constexpr uint16_t kWebServerPort = 80;
inline constexpr uint8_t kLedCount = 2;

// Step/dir pin mapping.
inline constexpr uint8_t kRhoStepPin = 33;
inline constexpr uint8_t kRhoDirPin = 25;
inline constexpr uint8_t kThetaStepPin = 32;
inline constexpr uint8_t kThetaDirPin = 22;

// TMC2209 UART pins and driver addresses.
inline constexpr uint8_t kUartRxPin = 27;
inline constexpr uint8_t kUartTxPin = 26;
inline constexpr uint8_t kRhoDriverAddress = 1;
inline constexpr uint8_t kRhoCDriverAddress = 0;
inline constexpr uint8_t kThetaDriverAddress = 2;
}
