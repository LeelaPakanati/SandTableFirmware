#include "PolarControl.hpp"
#include "PolarUtils.hpp"
#include "MakeUnique.hpp"
#include "Logger.hpp"
#include <cmath>
#include <SD.h>
#include <ArduinoJson.h>

// ============================================================================
// Constructor / Destructor
// ============================================================================

PolarControl::PolarControl() :
  m_tDriver(&Serial1, 0.12f, T_ADDR),
  m_rDriver(&Serial1, 0.12f, R_ADDR),
  m_rCDriver(&Serial1, 0.12f, RC_ADDR)
{
  // Initialize default driver settings
  m_tDriverSettings.current = 1200;  // Theta motor
  m_rDriverSettings.current = 500;   // Rho motor
}

PolarControl::~PolarControl() {
}

// ============================================================================
// Lifecycle
// ============================================================================

void PolarControl::begin() {
  if (m_mutex == NULL) {
    m_mutex = xSemaphoreCreateMutex();
  }

  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Load saved tuning settings (if any)
  loadTuningSettings();

  // Initialize motion planner with separate axis limits
  m_planner.init(
    STEPS_PER_MM,
    STEPS_PER_RADIAN,
    R_MAX,
    // Rho limits (mm)
    m_motionSettings.rMaxVelocity,
    m_motionSettings.rMaxAccel,
    m_motionSettings.rMaxJerk,
    // Theta limits (rad)
    m_motionSettings.tMaxVelocity,
    m_motionSettings.tMaxAccel,
    m_motionSettings.tMaxJerk
  );

  updateSpeedSettings();

  delay(10);
  LOG("Motor Setup Complete\r\n");
}

void PolarControl::updateSpeedSettings() {
  double speedFactor = m_speed / 10.0;
  m_planner.setSpeedMultiplier(speedFactor);
}

void PolarControl::setupDrivers() {
  if (m_state != UNINITIALIZED) {
    LOG("Setup was already done\r\n");
    return;
  }

  applyDriverSettings(m_tDriver, m_tDriverSettings);
  applyDriverSettings(m_rDriver, m_rDriverSettings);
  applyDriverSettings(m_rCDriver, m_rDriverSettings);

  m_rDriver.microsteps(R_MICROSTEPS);
  m_rDriver.rms_current(m_rDriverSettings.current);
  m_rCDriver.microsteps(R_MICROSTEPS);
  m_rCDriver.rms_current(m_rDriverSettings.current);
  m_tDriver.microsteps(T_MICROSTEPS);
  m_tDriver.rms_current(m_tDriverSettings.current);

  m_state = INITIALIZED;
}

void PolarControl::home() {
  xSemaphoreTake(m_mutex, portMAX_DELAY);
  homeDriver(m_rDriver);
  LOG("R Homing Done\r\n");

  // Planner position will be at 0,0 after init
  m_state = IDLE;
  xSemaphoreGive(m_mutex);
}

void PolarControl::applyDriverSettings(TMC2209Stepper &driver, const DriverSettings &settings) {
  driver.begin();
  driver.toff(settings.toff);
  driver.blank_time(settings.blankTime);
  driver.en_spreadCycle(settings.spreadCycle);

  if (settings.spreadCycle) {
    // SpreadCycle mode settings
    driver.hysteresis_start(settings.hystStart);
    driver.hysteresis_end(settings.hystEnd);
  } else {
    // StealthChop mode settings
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    driver.pwm_freq(settings.pwmFreq);
    driver.pwm_reg(settings.pwmReg);
    driver.pwm_lim(settings.pwmLim);
    driver.TPWMTHRS(settings.tpwmthrs);
  }

  driver.intpol(true);
  driver.I_scale_analog(false);
  driver.internal_Rsense(false);
  driver.mstep_reg_select(true);
  driver.pdn_disable(true);
  driver.VACTUAL(0);
  driver.shaft(false);
}

void PolarControl::homeDriver(TMC2209Stepper &driver, int speed) {
  driver.VACTUAL(500);
  delay(500);
  driver.VACTUAL(0);
  driver.VACTUAL(speed);

  delay(100);
  int sg_sum = driver.SG_RESULT();
  int cnt = 1;

  while (true) {
    int sg = driver.SG_RESULT();
    if (sg <= (sg_sum / cnt) * 0.75 && cnt > 15) {
      driver.VACTUAL(0);
      LOG("Hit Endstop, sg=%d\r\n", sg);
      break;
    }
    sg_sum += sg;
    cnt++;
    delay(10);
  }
}

void PolarControl::homeDriver(TMC2209Stepper &driver) {
  LOG("Preparing Homing\r\n");
  homeDriver(driver, 1000);
  driver.shaft(!driver.shaft());
  driver.VACTUAL(250);
  delay(1000);
  driver.VACTUAL(0);
  driver.shaft(!driver.shaft());
  homeDriver(driver, 250);
}

// ============================================================================
// Pattern Control
// ============================================================================

bool PolarControl::start(std::unique_ptr<PosGen> posGen) {
  xSemaphoreTake(m_mutex, portMAX_DELAY);

  if (m_state != IDLE) {
    LOG("Not IDLE. Start Failed\r\n");
    xSemaphoreGive(m_mutex);
    return false;
  }

  LOG("Starting new PosGen\r\n");

  m_posGen = std::move(posGen);

  // Reset planner stats for new pattern
  m_planner.resetCompletedCount();

  // Feed initial segments to planner
  feedPlanner();

  // Start the motion planner
  m_planner.start();

  m_state = RUNNING;
  xSemaphoreGive(m_mutex);
  return true;
}

bool PolarControl::startClearing(std::unique_ptr<PosGen> posGen) {
  xSemaphoreTake(m_mutex, portMAX_DELAY);

  if (m_state != IDLE) {
    LOG("Not IDLE. Clearing Start Failed\r\n");
    xSemaphoreGive(m_mutex);
    return false;
  }

  LOG("Starting Clearing Pattern\r\n");

  m_posGen = std::move(posGen);

  // Reset planner stats for new pattern
  m_planner.resetCompletedCount();

  // Feed initial segments to planner
  feedPlanner();

  // Start the motion planner
  m_planner.start();

  m_state = CLEARING;
  xSemaphoreGive(m_mutex);
  return true;
}

bool PolarControl::loadAndRunFile(String filePath) {
  return loadAndRunFile(filePath, R_MAX);
}

bool PolarControl::loadAndRunFile(String filePath, double maxRho) {
  xSemaphoreTake(m_mutex, portMAX_DELAY);

  if (m_state != IDLE) {
    LOG("ERROR: Cannot load file - system not IDLE\r\n");
    xSemaphoreGive(m_mutex);
    return false;
  }

  LOG("Loading pattern file: %s\r\n", filePath.c_str());

  auto newGen = std_patch::make_unique<FilePosGen>(filePath, maxRho);
  
  // Quick check if file is valid/openable
  PolarCord_t firstPos = newGen->getNextPos();
  if (firstPos.isNan()) {
    LOG("ERROR: File is empty or invalid\r\n");
    m_posGen.reset();
    xSemaphoreGive(m_mutex);
    return false;
  }
  
  m_posGen = std::move(newGen);

  // Reset planner stats for new pattern
  m_planner.resetCompletedCount();

  // Feed initial segments to planner
  feedPlanner();

  // Start the motion planner
  m_planner.start();

  m_state = RUNNING;
  xSemaphoreGive(m_mutex);
  return true;
}

bool PolarControl::pause() {
  xSemaphoreTake(m_mutex, portMAX_DELAY);
  if (m_state == RUNNING) {
    m_planner.stop();
    m_state = PAUSED;
    xSemaphoreGive(m_mutex);
    return true;
  }
  xSemaphoreGive(m_mutex);
  return false;
}

bool PolarControl::resume() {
  xSemaphoreTake(m_mutex, portMAX_DELAY);
  if (m_state == PAUSED) {
    // Feed segments and restart
    feedPlanner();
    m_planner.start();
    m_state = RUNNING;
    xSemaphoreGive(m_mutex);
    return true;
  }
  xSemaphoreGive(m_mutex);
  return false;
}

bool PolarControl::stop() {
  xSemaphoreTake(m_mutex, portMAX_DELAY);

  if (m_state == PAUSED || m_state == RUNNING || m_state == CLEARING) {
    m_posGen.reset();

    m_planner.stop();

    m_state = IDLE;
    LOG("Stopped\r\n");

    xSemaphoreGive(m_mutex);
    return true;
  }
  xSemaphoreGive(m_mutex);
  return false;
}

void PolarControl::setSpeed(uint8_t speed) {
  m_speed = speed;
  updateSpeedSettings();
}

void PolarControl::resetTheta() {
  m_planner.resetTheta();
}

PolarControl::State_t PolarControl::getState() {
  return m_state;
}

PolarCord_t PolarControl::getCurrentPosition() const {
  double theta, rho;
  // Note: casting away const for the planner call - it's thread-safe
  const_cast<MotionPlanner&>(m_planner).getCurrentPosition(theta, rho);
  return {theta, rho};
}

PolarCord_t PolarControl::getActualPosition() {
  double theta, rho;
  m_planner.getCurrentPosition(theta, rho);
  return {theta, rho};
}

void PolarControl::forceStop() {
  m_planner.stop();
}

// ============================================================================
// Feed segments to planner
// ============================================================================

void PolarControl::feedPlanner() {
  if (!m_posGen) return;

  // Keep planner buffer reasonably full
  while (m_planner.hasSpace()) {
    PolarCord_t next = m_posGen->getNextPos();

    if (next.isNan()) {
      // End of pattern
      break;
    }

    // Add segment to planner (planner handles theta normalization internally)
    m_planner.addSegment(next.theta, next.rho);
  }
}

// ============================================================================
// Main Processing Loop
// ============================================================================

bool PolarControl::processNextMove() {
  // Let planner process (handles timer internally)
  m_planner.process();

  xSemaphoreTake(m_mutex, portMAX_DELAY);

  if (m_state == RUNNING || m_state == CLEARING) {
    // Feed more segments to the planner
    feedPlanner();

    // Check if pattern is complete
    if (m_planner.isIdle()) {
      m_posGen.reset();
      m_state = IDLE;
      LOG("Pattern Complete\r\n");
      xSemaphoreGive(m_mutex);
      return false;
    }

    xSemaphoreGive(m_mutex);
    return m_planner.isRunning();
  }

  xSemaphoreGive(m_mutex);
  return false;
}

// ============================================================================
// Tuning Settings
// ============================================================================

void PolarControl::setMotionSettings(const MotionSettings& settings) {
  m_motionSettings = settings;

  // Update the motion planner with new limits
  m_planner.init(
    STEPS_PER_MM,
    STEPS_PER_RADIAN,
    R_MAX,
    m_motionSettings.rMaxVelocity,
    m_motionSettings.rMaxAccel,
    m_motionSettings.rMaxJerk,
    m_motionSettings.tMaxVelocity,
    m_motionSettings.tMaxAccel,
    m_motionSettings.tMaxJerk
  );

  LOG("Motion settings updated\r\n");
}

void PolarControl::setThetaDriverSettings(const DriverSettings& settings) {
  m_tDriverSettings = settings;
  applyDriverSettings(m_tDriver, m_tDriverSettings);
  m_tDriver.microsteps(T_MICROSTEPS);
  m_tDriver.rms_current(m_tDriverSettings.current);
  LOG("Theta driver settings updated\r\n");
}

void PolarControl::setRhoDriverSettings(const DriverSettings& settings) {
  m_rDriverSettings = settings;
  applyDriverSettings(m_rDriver, m_rDriverSettings);
  applyDriverSettings(m_rCDriver, m_rDriverSettings);
  m_rDriver.microsteps(R_MICROSTEPS);
  m_rDriver.rms_current(m_rDriverSettings.current);
  m_rCDriver.microsteps(R_MICROSTEPS);
  m_rCDriver.rms_current(m_rDriverSettings.current);
  LOG("Rho driver settings updated\r\n");
}

// ============================================================================
// Settings Persistence
// ============================================================================

static const char* TUNING_FILE = "/tuning.json";

bool PolarControl::saveTuningSettings() {
  JsonDocument doc;

  // Motion settings
  JsonObject motion = doc["motion"].to<JsonObject>();
  motion["rMaxVelocity"] = m_motionSettings.rMaxVelocity;
  motion["rMaxAccel"] = m_motionSettings.rMaxAccel;
  motion["rMaxJerk"] = m_motionSettings.rMaxJerk;
  motion["tMaxVelocity"] = m_motionSettings.tMaxVelocity;
  motion["tMaxAccel"] = m_motionSettings.tMaxAccel;
  motion["tMaxJerk"] = m_motionSettings.tMaxJerk;

  // Theta driver settings
  JsonObject theta = doc["thetaDriver"].to<JsonObject>();
  theta["current"] = m_tDriverSettings.current;
  theta["toff"] = m_tDriverSettings.toff;
  theta["blankTime"] = m_tDriverSettings.blankTime;
  theta["spreadCycle"] = m_tDriverSettings.spreadCycle;
  theta["pwmFreq"] = m_tDriverSettings.pwmFreq;
  theta["pwmReg"] = m_tDriverSettings.pwmReg;
  theta["pwmLim"] = m_tDriverSettings.pwmLim;
  theta["tpwmthrs"] = m_tDriverSettings.tpwmthrs;
  theta["hystStart"] = m_tDriverSettings.hystStart;
  theta["hystEnd"] = m_tDriverSettings.hystEnd;

  // Rho driver settings
  JsonObject rho = doc["rhoDriver"].to<JsonObject>();
  rho["current"] = m_rDriverSettings.current;
  rho["toff"] = m_rDriverSettings.toff;
  rho["blankTime"] = m_rDriverSettings.blankTime;
  rho["spreadCycle"] = m_rDriverSettings.spreadCycle;
  rho["pwmFreq"] = m_rDriverSettings.pwmFreq;
  rho["pwmReg"] = m_rDriverSettings.pwmReg;
  rho["pwmLim"] = m_rDriverSettings.pwmLim;
  rho["tpwmthrs"] = m_rDriverSettings.tpwmthrs;
  rho["hystStart"] = m_rDriverSettings.hystStart;
  rho["hystEnd"] = m_rDriverSettings.hystEnd;

  // Write to file
  File file = SD.open(TUNING_FILE, FILE_WRITE);
  if (!file) {
    LOG("Failed to open tuning file for writing\r\n");
    return false;
  }

  serializeJsonPretty(doc, file);
  file.close();
  LOG("Tuning settings saved to %s\r\n", TUNING_FILE);
  return true;
}

bool PolarControl::loadTuningSettings() {
  if (!SD.exists(TUNING_FILE)) {
    LOG("No tuning file found, using defaults\r\n");
    return false;
  }

  File file = SD.open(TUNING_FILE, FILE_READ);
  if (!file) {
    LOG("Failed to open tuning file for reading\r\n");
    return false;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    LOG("Failed to parse tuning file: %s\r\n", error.c_str());
    return false;
  }

  // Motion settings
  if (doc["motion"].is<JsonObject>()) {
    JsonObject motion = doc["motion"];
    m_motionSettings.rMaxVelocity = motion["rMaxVelocity"] | m_motionSettings.rMaxVelocity;
    m_motionSettings.rMaxAccel = motion["rMaxAccel"] | m_motionSettings.rMaxAccel;
    m_motionSettings.rMaxJerk = motion["rMaxJerk"] | m_motionSettings.rMaxJerk;
    m_motionSettings.tMaxVelocity = motion["tMaxVelocity"] | m_motionSettings.tMaxVelocity;
    m_motionSettings.tMaxAccel = motion["tMaxAccel"] | m_motionSettings.tMaxAccel;
    m_motionSettings.tMaxJerk = motion["tMaxJerk"] | m_motionSettings.tMaxJerk;
  }

  // Theta driver settings
  if (doc["thetaDriver"].is<JsonObject>()) {
    JsonObject theta = doc["thetaDriver"];
    m_tDriverSettings.current = theta["current"] | m_tDriverSettings.current;
    m_tDriverSettings.toff = theta["toff"] | m_tDriverSettings.toff;
    m_tDriverSettings.blankTime = theta["blankTime"] | m_tDriverSettings.blankTime;
    m_tDriverSettings.spreadCycle = theta["spreadCycle"] | m_tDriverSettings.spreadCycle;
    m_tDriverSettings.pwmFreq = theta["pwmFreq"] | m_tDriverSettings.pwmFreq;
    m_tDriverSettings.pwmReg = theta["pwmReg"] | m_tDriverSettings.pwmReg;
    m_tDriverSettings.pwmLim = theta["pwmLim"] | m_tDriverSettings.pwmLim;
    m_tDriverSettings.tpwmthrs = theta["tpwmthrs"] | m_tDriverSettings.tpwmthrs;
    m_tDriverSettings.hystStart = theta["hystStart"] | m_tDriverSettings.hystStart;
    m_tDriverSettings.hystEnd = theta["hystEnd"] | m_tDriverSettings.hystEnd;
  }

  // Rho driver settings
  if (doc["rhoDriver"].is<JsonObject>()) {
    JsonObject rho = doc["rhoDriver"];
    m_rDriverSettings.current = rho["current"] | m_rDriverSettings.current;
    m_rDriverSettings.toff = rho["toff"] | m_rDriverSettings.toff;
    m_rDriverSettings.blankTime = rho["blankTime"] | m_rDriverSettings.blankTime;
    m_rDriverSettings.spreadCycle = rho["spreadCycle"] | m_rDriverSettings.spreadCycle;
    m_rDriverSettings.pwmFreq = rho["pwmFreq"] | m_rDriverSettings.pwmFreq;
    m_rDriverSettings.pwmReg = rho["pwmReg"] | m_rDriverSettings.pwmReg;
    m_rDriverSettings.pwmLim = rho["pwmLim"] | m_rDriverSettings.pwmLim;
    m_rDriverSettings.tpwmthrs = rho["tpwmthrs"] | m_rDriverSettings.tpwmthrs;
    m_rDriverSettings.hystStart = rho["hystStart"] | m_rDriverSettings.hystStart;
    m_rDriverSettings.hystEnd = rho["hystEnd"] | m_rDriverSettings.hystEnd;
  }

  LOG("Tuning settings loaded from %s\r\n", TUNING_FILE);
  return true;
}

// ============================================================================
// Motor Stress Tests (using Motion Planner)
// ============================================================================

// Test pattern generator for theta motor
class TestThetaPosGen : public PosGen {
public:
  TestThetaPosGen(double fixedRho) : m_rho(fixedRho), m_phase(0), m_step(0) {}

  PolarCord_t getNextPos() override {
    // Phase 0: Rotate 5 full turns forward
    // Phase 1: Rotate 5 full turns back
    // Phase 2: Varying size moves (0.5° to 90°)
    // Phase 3: Random quick reversals

    const double STEP_SIZE = PI;  // radians per point
    const double FULL_ROTATION = 2.0 * PI;
    const int ROTATIONS = 5;
    const double DEGREES_TO_RADIANS = PI / 180.0;

    // Varying move sizes in degrees
    static const double MOVE_SIZES[] = {
      0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0,
      90.0, 60.0, 45.0, 30.0, 20.0, 15.0, 10.0, 5.0, 2.0, 1.0, 0.5
    };
    static const int NUM_MOVE_SIZES = sizeof(MOVE_SIZES) / sizeof(MOVE_SIZES[0]);

    if (m_phase == 0) {
      // Forward rotation
      double theta = m_step * STEP_SIZE;
      if (theta >= FULL_ROTATION * ROTATIONS) {
        m_phase = 1;
        m_step = 0;
        m_baseTheta = FULL_ROTATION * ROTATIONS;
      } else {
        m_step++;
        return {theta, m_rho};
      }
    }

    if (m_phase == 1) {
      // Reverse rotation back to start
      double theta = m_baseTheta - m_step * STEP_SIZE;
      if (theta <= 0) {
        m_phase = 2;
        m_step = 0;
        m_currentTheta = 0;
      } else {
        m_step++;
        return {theta, m_rho};
      }
    }

    if (m_phase == 2) {
      // Varying size moves - go out and back for each size
      if (m_step >= NUM_MOVE_SIZES * 2) {
        m_phase = 3;
        m_step = 0;
        m_currentTheta = 0;
      } else {
        int sizeIdx = m_step / 2;
        bool goingOut = (m_step % 2 == 0);
        double moveSize = MOVE_SIZES[sizeIdx] * DEGREES_TO_RADIANS;

        if (goingOut) {
          m_currentTheta = moveSize;
        } else {
          m_currentTheta = 0;
        }
        m_step++;
        return {m_currentTheta, m_rho};
      }
    }

    if (m_phase == 3) {
      // Quick random-ish reversals at different amplitudes
      static const double QUICK_SIZES[] = {5.0, 45.0, 2.0, 90.0, 10.0, 30.0, 1.0, 60.0, 15.0, 0.5};
      static const int NUM_QUICK = sizeof(QUICK_SIZES) / sizeof(QUICK_SIZES[0]);

      if (m_step >= NUM_QUICK * 2) {
        return {std::nan(""), std::nan("")};  // Done
      }

      int sizeIdx = m_step / 2;
      bool goingOut = (m_step % 2 == 0);
      double moveSize = QUICK_SIZES[sizeIdx] * DEGREES_TO_RADIANS;

      if (goingOut) {
        m_currentTheta = moveSize;
      } else {
        m_currentTheta = 0;
      }
      m_step++;
      return {m_currentTheta, m_rho};
    }

    return {std::nan(""), std::nan("")};
  }

private:
  double m_rho;
  double m_baseTheta = 0;
  double m_currentTheta = 0;
  int m_phase;
  int m_step;
};

// Test pattern generator for rho motor
class TestRhoPosGen : public PosGen {
public:
  TestRhoPosGen(double maxRho) : m_maxRho(maxRho), m_phase(0), m_step(0) {}

  PolarCord_t getNextPos() override {
    // Phase 0: Move from center to max
    // Phase 1: Move from max to near-zero
    // Phase 2: Move back to center
    // Phase 3: Varying size moves (1mm to 100mm)
    // Phase 4: Quick random-ish reversals

    const double STEP_SIZE = 5.0;  // mm per point
    const double CENTER = m_maxRho / 2;
    const double MIN_RHO = 20.0;

    // Varying move sizes in mm
    static const double MOVE_SIZES[] = {
      1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 50.0, 75.0, 100.0,
      100.0, 75.0, 50.0, 30.0, 20.0, 10.0, 5.0, 2.0, 1.0
    };
    static const int NUM_MOVE_SIZES = sizeof(MOVE_SIZES) / sizeof(MOVE_SIZES[0]);

    if (m_phase == 0) {
      // Move outward from center to max
      double rho = CENTER + m_step * STEP_SIZE;
      if (rho >= m_maxRho) {
        m_phase = 1;
        m_step = 0;
      } else {
        m_step++;
        return {0, rho};
      }
    }

    if (m_phase == 1) {
      // Move inward from max to min
      double rho = m_maxRho - m_step * STEP_SIZE;
      if (rho <= MIN_RHO) {
        m_phase = 2;
        m_step = 0;
      } else {
        m_step++;
        return {0, rho};
      }
    }

    if (m_phase == 2) {
      // Move back to center
      double rho = MIN_RHO + m_step * STEP_SIZE;
      if (rho >= CENTER) {
        m_phase = 3;
        m_step = 0;
        m_currentRho = CENTER;
      } else {
        m_step++;
        return {0, rho};
      }
    }

    if (m_phase == 3) {
      // Varying size moves - go out and back for each size
      if (m_step >= NUM_MOVE_SIZES * 2) {
        m_phase = 4;
        m_step = 0;
        m_currentRho = CENTER;
      } else {
        int sizeIdx = m_step / 2;
        bool goingOut = (m_step % 2 == 0);
        double moveSize = MOVE_SIZES[sizeIdx];

        if (goingOut) {
          m_currentRho = CENTER + moveSize;
        } else {
          m_currentRho = CENTER;
        }
        m_step++;
        return {0, m_currentRho};
      }
    }

    if (m_phase == 4) {
      // Quick random-ish reversals at different amplitudes
      static const double QUICK_SIZES[] = {10.0, 50.0, 5.0, 100.0, 20.0, 75.0, 2.0, 30.0, 1.0, 60.0};
      static const int NUM_QUICK = sizeof(QUICK_SIZES) / sizeof(QUICK_SIZES[0]);

      if (m_step >= NUM_QUICK * 2) {
        return {std::nan(""), std::nan("")};  // Done
      }

      int sizeIdx = m_step / 2;
      bool goingOut = (m_step % 2 == 0);
      double moveSize = QUICK_SIZES[sizeIdx];

      if (goingOut) {
        m_currentRho = CENTER + moveSize;
      } else {
        m_currentRho = CENTER;
      }
      m_step++;
      return {0, m_currentRho};
    }

    return {std::nan(""), std::nan("")};
  }

private:
  double m_maxRho;
  double m_currentRho = 0;
  int m_phase;
  int m_step;
};

void PolarControl::testThetaMotor() {
  if (m_state != IDLE) {
    LOG("Cannot test: not IDLE\r\n");
    return;
  }

  LOG("Starting theta motor test via motion planner...\r\n");

  // Get current rho position to hold it constant
  double currentTheta, currentRho;
  m_planner.getCurrentPosition(currentTheta, currentRho);

  // Use a safe rho position (middle of range)
  double testRho = (currentRho > 50 && currentRho < R_MAX - 50) ? currentRho : R_MAX / 2;

  // Reset theta to 0 for clean test
  resetTheta();

  // Create and start the test pattern
  start(std_patch::make_unique<TestThetaPosGen>(testRho));

  LOG("Theta test pattern started\r\n");
}

void PolarControl::testRhoMotor() {
  if (m_state != IDLE) {
    LOG("Cannot test: not IDLE\r\n");
    return;
  }

  LOG("Starting rho motor test via motion planner...\r\n");

  // Reset theta to 0 for clean test
  resetTheta();

  // Create and start the test pattern
  start(std_patch::make_unique<TestRhoPosGen>(R_MAX));

  LOG("Rho test pattern started\r\n");
}
