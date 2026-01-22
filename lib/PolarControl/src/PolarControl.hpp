#pragma once
#include <TMC2209.h>
#include "MotionPlanner.hpp"
#include <PosGen.hpp>
#include <freertos/semphr.h>
#include <memory>

// Pin definitions
#define R_STEP_PIN 33
#define R_DIR_PIN 25
#define T_STEP_PIN 32
#define T_DIR_PIN 22
#define RX_PIN 27
#define TX_PIN 26
#define R_ADDR 1
#define RC_ADDR 0
#define T_ADDR 2

// Tuning settings structures
struct MotionSettings {
  double rMaxVelocity = 30.0;   // mm/s
  double rMaxAccel = 20.0;      // mm/s²
  double rMaxJerk = 100.0;      // mm/s³
  double tMaxVelocity = 1.0;    // rad/s
  double tMaxAccel = 2.0;       // rad/s²
  double tMaxJerk = 10.0;       // rad/s³
};

struct DriverSettings {
  // Current settings (in mA)
  uint16_t runCurrent = 800;          // Run current in mA
  uint16_t holdCurrent = 400;         // Hold current in mA
  uint8_t holdDelay = 8;              // Delay before switching to hold current (0-15)

  // Microstepping
  uint16_t microsteps = 2;            // Microsteps per full step (1,2,4,8,16,32,64,128,256)

  // StealthChop settings
  bool stealthChopEnabled = true;     // true=StealthChop, false=SpreadCycle
  uint32_t stealthChopThreshold = 0;  // Velocity threshold for StealthChop (0=always)

  // CoolStep settings (current reduction at low load)
  bool coolStepEnabled = false;       // Enable CoolStep
  uint8_t coolStepLowerThreshold = 1; // Lower StallGuard threshold (0-15)
  uint8_t coolStepUpperThreshold = 0; // Upper StallGuard threshold (0-15)
  uint8_t coolStepCurrentIncrement = 0;  // Current increment (0-3: 1,2,4,8)
  uint8_t coolStepMeasurementCount = 0;  // Measurement count (0-3: 32,8,2,1)
  uint32_t coolStepThreshold = 0;     // Velocity threshold for CoolStep
};

class PolarControl {
public:
  enum State_t : uint8_t {
    UNINITIALIZED,
    INITIALIZED,
    IDLE,
    RUNNING,
    PAUSED,
    STOPPING,
    CLEARING
  };

  PolarControl();
  ~PolarControl();

  // Lifecycle
  void begin();
  void setupDrivers();
  void home();

  // Pattern control
  bool start(std::unique_ptr<PosGen> posGen);
  bool startClearing(std::unique_ptr<PosGen> posGen);
  bool loadAndRunFile(String filePath);
  bool loadAndRunFile(String filePath, double maxRho);
  bool pause();
  bool resume();
  bool stop();

  // Main processing loop - call from motor task
  bool processNextMove();

  // Getters/Setters
  State_t getState();
  void setSpeed(uint8_t speed);
  uint8_t getSpeed() const { return m_speed; }
  PolarCord_t getCurrentPosition() const;
  PolarCord_t getActualPosition();
  uint32_t getSegmentsCompleted() const { return m_planner.getCompletedCount(); }
  double getMaxRho() const { return R_MAX; }
  int getProgressPercent() const;

  // Reset theta to zero (current position becomes new origin)
  void resetTheta();

  // Tuning getters/setters
  const MotionSettings& getMotionSettings() const { return m_motionSettings; }
  void setMotionSettings(const MotionSettings& settings);

  const DriverSettings& getThetaDriverSettings() const { return m_tDriverSettings; }
  const DriverSettings& getRhoDriverSettings() const { return m_rDriverSettings; }
  void setThetaDriverSettings(const DriverSettings& settings);
  void setRhoDriverSettings(const DriverSettings& settings);

  // Settings persistence
  bool saveTuningSettings();
  bool loadTuningSettings();

  // Motor stress tests (blocking calls - run from main task)
  void testThetaContinuous();
  void testThetaStress();
  void testRhoContinuous();
  void testRhoStress();

  // Driver diagnostics - returns JSON string with all driver register values
  String dumpThetaDriverSettings();
  String dumpRhoDriverSettings();

private:
  // Physical constants
  static constexpr double R_MAX = 450.0;
  static constexpr float R_SENSE = 0.12f;  // Sense resistor in ohms

  // These are calculated based on current microstep settings
  int getStepsPerMm() const { return 50 * m_rDriverSettings.microsteps; }
  int getStepsPerRadian() const { return (int)((200.0 * m_tDriverSettings.microsteps / (2.0 * PI)) * (60.0 / 16.0)); }

  // Tuning settings (runtime changeable)
  MotionSettings m_motionSettings;
  DriverSettings m_tDriverSettings;  // Theta driver
  DriverSettings m_rDriverSettings;  // Rho driver

  // Motion planner with lookahead and S-curves
  MotionPlanner m_planner;

  // TMC2209 drivers for configuration and homing
  TMC2209 m_tDriver;
  TMC2209 m_rDriver;
  TMC2209 m_rCDriver;

  SemaphoreHandle_t m_mutex = NULL;

  State_t m_state = UNINITIALIZED;
  std::unique_ptr<PosGen> m_posGen;

  // Speed setting: 1-10
  uint8_t m_speed = 5;

  // Helpers
  void forceStop();
  void updateSpeedSettings();
  void feedPlanner();

  // Driver setup and homing
  void applyDriverSettings(TMC2209 &driver, const DriverSettings &settings);
  void homeDriver(TMC2209 &driver, int speed);
  void homeDriver(TMC2209 &driver);
};
