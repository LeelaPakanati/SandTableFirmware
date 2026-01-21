#pragma once
#include <TMCStepper.h>
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
  uint16_t current = 800;       // mA
  uint8_t toff = 3;             // Chopper off-time (1-15)
  uint8_t blankTime = 16;       // Comparator blank time (16,24,36,54)
  bool spreadCycle = false;     // true=SpreadCycle, false=StealthChop
  uint8_t pwmFreq = 3;          // PWM frequency (0-3)
  uint8_t pwmReg = 2;           // PWM regulation
  uint8_t pwmLim = 8;           // PWM limit
  uint32_t tpwmthrs = 500;      // StealthChop threshold
  uint8_t hystStart = 4;        // SpreadCycle hysteresis start
  int8_t hystEnd = 0;           // SpreadCycle hysteresis end
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
  void testThetaMotor();
  void testRhoMotor();

private:
  // Physical constants
  static constexpr double R_MAX = 450.0;
  static constexpr int T_MICROSTEPS = 2;
  static constexpr int R_MICROSTEPS = 2;

  static constexpr int STEPS_PER_MM = 50 * R_MICROSTEPS;
  static constexpr int STEPS_PER_RADIAN = (int)((200.0 * T_MICROSTEPS / (2.0 * PI)) * (60.0 / 16.0));

  // Tuning settings (runtime changeable)
  MotionSettings m_motionSettings;
  DriverSettings m_tDriverSettings;  // Theta driver
  DriverSettings m_rDriverSettings;  // Rho driver

  // Motion planner with lookahead and S-curves
  MotionPlanner m_planner;

  // TMC2209 drivers for configuration and homing
  TMC2209Stepper m_tDriver;
  TMC2209Stepper m_rDriver;
  TMC2209Stepper m_rCDriver;

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
  void applyDriverSettings(TMC2209Stepper &driver, const DriverSettings &settings);
  void homeDriver(TMC2209Stepper &driver, int speed);
  void homeDriver(TMC2209Stepper &driver);
};
