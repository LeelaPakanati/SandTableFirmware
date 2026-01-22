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

PolarControl::PolarControl() {
    // Initialize default driver settings
    m_tDriverSettings.runCurrent = 1200;   // Theta motor - higher current (mA)
    m_tDriverSettings.holdCurrent = 300;
    m_tDriverSettings.microsteps = 256;
    m_rDriverSettings.runCurrent = 500;    // Rho motor - lower current (mA)
    m_rDriverSettings.holdCurrent = 200;
    m_rDriverSettings.microsteps = 2;
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

    // Load saved tuning settings (if any)
    loadTuningSettings();

    // Setup TMC2209 drivers with serial connection
    // Using ESP32 variant of setup() with alternate pins
    m_tDriver.setup(Serial1, 115200, TMC2209::SERIAL_ADDRESS_2, RX_PIN, TX_PIN);
    m_rDriver.setup(Serial1, 115200, TMC2209::SERIAL_ADDRESS_1, RX_PIN, TX_PIN);
    m_rCDriver.setup(Serial1, 115200, TMC2209::SERIAL_ADDRESS_0, RX_PIN, TX_PIN);

    delay(100);  // Allow drivers to initialize

    // Initialize motion planner with separate axis limits
    m_planner.init(
        getStepsPerMm(),
        getStepsPerRadian(),
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

    // Apply settings to all drivers
    applyDriverSettings(m_tDriver, m_tDriverSettings);
    applyDriverSettings(m_rDriver, m_rDriverSettings);
    applyDriverSettings(m_rCDriver, m_rDriverSettings);

    // Enable all drivers
    m_tDriver.enable();
    m_rDriver.enable();
    m_rCDriver.enable();

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

void PolarControl::applyDriverSettings(TMC2209 &driver, const DriverSettings &settings) {
    // Set microstepping first
    driver.setMicrostepsPerStep(settings.microsteps);

    // Current settings conversion (mA to percent)
    // Assuming 0.12ohm sense resistors -> Max RMS current ~1900mA
    const float MAX_CURRENT_MA = 1900.0f;

    uint8_t runPercent = (uint8_t)((settings.runCurrent / MAX_CURRENT_MA) * 100.0f);
    if (runPercent > 100) runPercent = 100;

    uint8_t holdPercent = (uint8_t)((settings.holdCurrent / MAX_CURRENT_MA) * 100.0f);
    if (holdPercent > 100) holdPercent = 100;

    driver.setRunCurrent(runPercent);
    driver.setHoldCurrent(holdPercent);

    // Hold delay (0-15 mapped to 0-100%)
    uint8_t delayPercent = (uint8_t)((settings.holdDelay * 100) / 15);
    if (delayPercent > 100) delayPercent = 100;
    driver.setHoldDelay(delayPercent);

    // Use external sense resistors
    driver.useExternalSenseResistors();

    // StealthChop / SpreadCycle mode
    if (settings.stealthChopEnabled) {
        driver.enableStealthChop();
        driver.setStealthChopDurationThreshold(settings.stealthChopThreshold);

        driver.enableAutomaticCurrentScaling();
        driver.enableAutomaticGradientAdaptation();
    } else {
        // SpreadCycle mode (louder but more torque at high speeds)
        driver.disableStealthChop();
    }

    // CoolStep settings (reduces current at low load for efficiency)
    if (settings.coolStepEnabled) {
        driver.enableCoolStep(settings.coolStepLowerThreshold, settings.coolStepUpperThreshold);
        driver.setCoolStepCurrentIncrement(static_cast<TMC2209::CurrentIncrement>(settings.coolStepCurrentIncrement));
        driver.setCoolStepMeasurementCount(static_cast<TMC2209::MeasurementCount>(settings.coolStepMeasurementCount));
        driver.setCoolStepDurationThreshold(settings.coolStepThreshold);
    } else {
        driver.disableCoolStep();
    }

    // Use step/dir interface for motion (not UART velocity mode)
    driver.moveUsingStepDirInterface();
}

void PolarControl::homeDriver(TMC2209 &driver, int speed) {
    // Move forward briefly
    driver.moveAtVelocity(500);
    delay(500);
    driver.moveAtVelocity(0);
    driver.moveAtVelocity(speed);

    delay(100);
    int sg_sum = driver.getStallGuardResult();
    int cnt = 1;

    while (true) {
        int sg = driver.getStallGuardResult();
        if (sg <= (sg_sum / cnt) * 0.75 && cnt > 15) {
            driver.moveAtVelocity(0);
            LOG("Hit Endstop, sg=%d\r\n", sg);
            break;
        }
        sg_sum += sg;
        cnt++;
        delay(10);
    }
}

void PolarControl::homeDriver(TMC2209 &driver) {
    LOG("Preparing Homing\r\n");
    homeDriver(driver, 1000);

    // Reverse direction
    driver.enableInverseMotorDirection();
    driver.moveAtVelocity(250);
    delay(1000);
    driver.moveAtVelocity(0);

    // Restore direction
    driver.disableInverseMotorDirection();
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

int PolarControl::getProgressPercent() const {
    if (!m_posGen) return -1;
    return m_posGen->getProgressPercent();
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
            // End of pattern - tell planner to decelerate to stop
            m_planner.setEndOfPattern(true);
            break;
        }

        // More segments coming - maintain velocity at end of buffer
        m_planner.setEndOfPattern(false);

        // Add segment to planner (planner handles theta normalization internally)
        m_planner.addSegment(next.theta, next.rho);
    }
    m_planner.recalculate();
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
        getStepsPerMm(),
        getStepsPerRadian(),
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

    // Reinitialize planner if microsteps changed
    m_planner.init(
        getStepsPerMm(),
        getStepsPerRadian(),
        R_MAX,
        m_motionSettings.rMaxVelocity,
        m_motionSettings.rMaxAccel,
        m_motionSettings.rMaxJerk,
        m_motionSettings.tMaxVelocity,
        m_motionSettings.tMaxAccel,
        m_motionSettings.tMaxJerk
    );

    LOG("Theta driver settings updated\r\n");
}

void PolarControl::setRhoDriverSettings(const DriverSettings& settings) {
    m_rDriverSettings = settings;
    applyDriverSettings(m_rDriver, m_rDriverSettings);
    applyDriverSettings(m_rCDriver, m_rDriverSettings);

    // Reinitialize planner if microsteps changed
    m_planner.init(
        getStepsPerMm(),
        getStepsPerRadian(),
        R_MAX,
        m_motionSettings.rMaxVelocity,
        m_motionSettings.rMaxAccel,
        m_motionSettings.rMaxJerk,
        m_motionSettings.tMaxVelocity,
        m_motionSettings.tMaxAccel,
        m_motionSettings.tMaxJerk
    );

    LOG("Rho driver settings updated\r\n");
}

// ============================================================================
// Settings Persistence
// ============================================================================

static const char* TUNING_FILE = "/tuning.json";

// Helper to save driver settings to JSON object
static void saveDriverSettingsToJson(JsonObject& obj, const DriverSettings& settings) {
    // Current settings (mA)
    obj["runCurrent"] = settings.runCurrent;
    obj["holdCurrent"] = settings.holdCurrent;
    obj["holdDelay"] = settings.holdDelay;

    // Microstepping
    obj["microsteps"] = settings.microsteps;

    // StealthChop settings
    obj["stealthChopEnabled"] = settings.stealthChopEnabled;
    obj["stealthChopThreshold"] = settings.stealthChopThreshold;

    // CoolStep settings
    obj["coolStepEnabled"] = settings.coolStepEnabled;
    obj["coolStepLowerThreshold"] = settings.coolStepLowerThreshold;
    obj["coolStepUpperThreshold"] = settings.coolStepUpperThreshold;
    obj["coolStepCurrentIncrement"] = settings.coolStepCurrentIncrement;
    obj["coolStepMeasurementCount"] = settings.coolStepMeasurementCount;
    obj["coolStepThreshold"] = settings.coolStepThreshold;
}

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

    // Driver settings
    JsonObject theta = doc["thetaDriver"].to<JsonObject>();
    saveDriverSettingsToJson(theta, m_tDriverSettings);

    JsonObject rho = doc["rhoDriver"].to<JsonObject>();
    saveDriverSettingsToJson(rho, m_rDriverSettings);

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

// Helper to load driver settings from JSON object
static void loadDriverSettingsFromJson(JsonObject& obj, DriverSettings& settings) {
    // Current settings (mA)
    settings.runCurrent = obj["runCurrent"] | settings.runCurrent;
    settings.holdCurrent = obj["holdCurrent"] | settings.holdCurrent;
    settings.holdDelay = obj["holdDelay"] | settings.holdDelay;

    // Microstepping
    settings.microsteps = obj["microsteps"] | settings.microsteps;

    // StealthChop settings
    settings.stealthChopEnabled = obj["stealthChopEnabled"] | settings.stealthChopEnabled;
    settings.stealthChopThreshold = obj["stealthChopThreshold"] | settings.stealthChopThreshold;

    // CoolStep settings
    settings.coolStepEnabled = obj["coolStepEnabled"] | settings.coolStepEnabled;
    settings.coolStepLowerThreshold = obj["coolStepLowerThreshold"] | settings.coolStepLowerThreshold;
    settings.coolStepUpperThreshold = obj["coolStepUpperThreshold"] | settings.coolStepUpperThreshold;
    settings.coolStepCurrentIncrement = obj["coolStepCurrentIncrement"] | settings.coolStepCurrentIncrement;
    settings.coolStepMeasurementCount = obj["coolStepMeasurementCount"] | settings.coolStepMeasurementCount;
    settings.coolStepThreshold = obj["coolStepThreshold"] | settings.coolStepThreshold;
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

    // Driver settings
    if (doc["thetaDriver"].is<JsonObject>()) {
        JsonObject theta = doc["thetaDriver"];
        loadDriverSettingsFromJson(theta, m_tDriverSettings);
    }

    if (doc["rhoDriver"].is<JsonObject>()) {
        JsonObject rho = doc["rhoDriver"];
        loadDriverSettingsFromJson(rho, m_rDriverSettings);
    }

    LOG("Tuning settings loaded from %s\r\n", TUNING_FILE);
    return true;
}

// ============================================================================
// Motor Stress Tests (using Motion Planner)
// ============================================================================

class TestThetaContinuousGen : public PosGen {
public:
    TestThetaContinuousGen(double fixedRho) : m_rho(fixedRho), m_phase(0) {}

    PolarCord_t getNextPos() override {
        // Phase 0: Rotate 5 full turns forward in one go
        // Phase 1: Rotate 5 full turns back in one go

        const double FULL_ROTATION = 2.0 * PI;
        const int ROTATIONS = 5;

        if (m_phase == 0) {
            m_phase = 1;
            return {0, m_rho};
        }

        if (m_phase == 1) {
            m_phase = 2;
            return {FULL_ROTATION * ROTATIONS, m_rho};
        }

        if (m_phase == 2) {
            m_phase = 3;
            return {0, m_rho};
        }
        return {std::nan(""), std::nan("")};
    }
private:
    double m_rho;
    int m_phase;
};

class TestThetaStressGen : public PosGen {
public:
    TestThetaStressGen(double fixedRho) : m_rho(fixedRho), m_phase(0), m_step(0) {}

    PolarCord_t getNextPos() override {
        const double DEGREES_TO_RADIANS = PI / 180.0;

        // Varying move sizes in degrees
        static const double MOVE_SIZES[] = {
            0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0,
            90.0, 60.0, 45.0, 30.0, 20.0, 15.0, 10.0, 5.0, 2.0, 1.0, 0.5
        };
        static const int NUM_MOVE_SIZES = sizeof(MOVE_SIZES) / sizeof(MOVE_SIZES[0]);

        if (m_phase == 0) {
            // Varying size moves - go out and back for each size
            if (m_step >= NUM_MOVE_SIZES * 2) {
                m_phase = 1;
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

        if (m_phase == 1) {
            // Quick random-ish reversals at different amplitudes
            static const double QUICK_SIZES[] = {5.0, 45.0, 2.0, 90.0, 10.0, 30.0, 1.0, 60.0, 15.0, 0.5};
            static const int NUM_QUICK = sizeof(QUICK_SIZES) / sizeof(QUICK_SIZES[0]);

            if (m_step >= NUM_QUICK * 2) {
                return {std::nan(""), std::nan("")};
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
    double m_currentTheta = 0;
    int m_phase;
    int m_step;
};

class TestRhoContinuousGen : public PosGen {
public:
    TestRhoContinuousGen(double maxRho) : m_maxRho(maxRho), m_phase(0) {}

    PolarCord_t getNextPos() override {
        const double CENTER = m_maxRho / 2;
        const double MIN_RHO = 20.0;

        if (m_phase == 0) {
            // Move outward to max in one go
            m_phase = 1;
            return {0, m_maxRho};
        }

        if (m_phase == 1) {
            // Move inward to min in one go
            m_phase = 2;
            return {0, MIN_RHO};
        }

        if (m_phase == 2) {
            // Move back to center in one go
            m_phase = 3;
            return {0, CENTER};
        }
        return {std::nan(""), std::nan("")};
    }
private:
    double m_maxRho;
    int m_phase;
};

class TestRhoStressGen : public PosGen {
public:
    TestRhoStressGen(double maxRho) : m_maxRho(maxRho), m_phase(0), m_step(0) {
        m_currentRho = maxRho / 2;
    }

    PolarCord_t getNextPos() override {
        const double CENTER = m_maxRho / 2;

        // Varying move sizes in mm
        static const double MOVE_SIZES[] = {
            1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 50.0, 75.0, 100.0,
            100.0, 75.0, 50.0, 30.0, 20.0, 10.0, 5.0, 2.0, 1.0
        };
        static const int NUM_MOVE_SIZES = sizeof(MOVE_SIZES) / sizeof(MOVE_SIZES[0]);

        if (m_phase == 0) {
            // Varying size moves - go out and back for each size
            if (m_step >= NUM_MOVE_SIZES * 2) {
                m_phase = 1;
                m_step = 0;
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

        if (m_phase == 1) {
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
    double m_currentRho;
    int m_phase;
    int m_step;
};

void PolarControl::testThetaContinuous() {
    if (m_state != IDLE) return;
    LOG("Starting theta continuous test...\r\n");

    double currentTheta, currentRho;
    m_planner.getCurrentPosition(currentTheta, currentRho);
    double testRho = (currentRho > 50 && currentRho < R_MAX - 50) ? currentRho : R_MAX / 2;

    resetTheta();
    start(std_patch::make_unique<TestThetaContinuousGen>(testRho));
}

void PolarControl::testThetaStress() {
    if (m_state != IDLE) return;
    LOG("Starting theta stress test...\r\n");

    double currentTheta, currentRho;
    m_planner.getCurrentPosition(currentTheta, currentRho);
    double testRho = (currentRho > 50 && currentRho < R_MAX - 50) ? currentRho : R_MAX / 2;

    resetTheta();
    start(std_patch::make_unique<TestThetaStressGen>(testRho));
}

void PolarControl::testRhoContinuous() {
    if (m_state != IDLE) return;
    LOG("Starting rho continuous test...\r\n");
    resetTheta();
    start(std_patch::make_unique<TestRhoContinuousGen>(R_MAX));
}

void PolarControl::testRhoStress() {
    if (m_state != IDLE) return;
    LOG("Starting rho stress test...\r\n");
    resetTheta();
    start(std_patch::make_unique<TestRhoStressGen>(R_MAX));
}

// ============================================================================
// Driver Diagnostics
// ============================================================================

// Helper to dump driver info to JSON
static String dumpDriverToJson(TMC2209& driver, const char* name) {
    JsonDocument doc;

    doc["name"] = name;
    doc["communicating"] = driver.isCommunicating();
    doc["setupOk"] = driver.isSetupAndCommunicating();

    if (driver.isCommunicating()) {
        // Get settings from driver
        TMC2209::Settings settings = driver.getSettings();
        JsonObject settingsObj = doc["settings"].to<JsonObject>();
        settingsObj["softwareEnabled"] = settings.software_enabled;
        settingsObj["microstepsPerStep"] = settings.microsteps_per_step;
        settingsObj["inverseMotorDirection"] = settings.inverse_motor_direction_enabled;
        settingsObj["stealthChopEnabled"] = settings.stealth_chop_enabled;
        settingsObj["standstillMode"] = settings.standstill_mode;
        settingsObj["irunPercent"] = settings.irun_percent;
        settingsObj["irunRegister"] = settings.irun_register_value;
        settingsObj["iholdPercent"] = settings.ihold_percent;
        settingsObj["iholdRegister"] = settings.ihold_register_value;
        settingsObj["iholdDelayPercent"] = settings.iholddelay_percent;
        settingsObj["iholdDelayRegister"] = settings.iholddelay_register_value;
        settingsObj["coolStepEnabled"] = settings.cool_step_enabled;
        settingsObj["analogCurrentScaling"] = settings.analog_current_scaling_enabled;
        settingsObj["internalSenseResistors"] = settings.internal_sense_resistors_enabled;

        // Get status from driver
        TMC2209::Status status = driver.getStatus();
        JsonObject statusObj = doc["status"].to<JsonObject>();
        statusObj["overTempWarning"] = (bool)status.over_temperature_warning;
        statusObj["overTempShutdown"] = (bool)status.over_temperature_shutdown;
        statusObj["shortToGroundA"] = (bool)status.short_to_ground_a;
        statusObj["shortToGroundB"] = (bool)status.short_to_ground_b;
        statusObj["lowSideShortA"] = (bool)status.low_side_short_a;
        statusObj["lowSideShortB"] = (bool)status.low_side_short_b;
        statusObj["openLoadA"] = (bool)status.open_load_a;
        statusObj["openLoadB"] = (bool)status.open_load_b;
        statusObj["overTemp120c"] = (bool)status.over_temperature_120c;
        statusObj["overTemp143c"] = (bool)status.over_temperature_143c;
        statusObj["overTemp150c"] = (bool)status.over_temperature_150c;
        statusObj["overTemp157c"] = (bool)status.over_temperature_157c;
        statusObj["currentScaling"] = status.current_scaling;
        statusObj["stealthChopMode"] = (bool)status.stealth_chop_mode;
        statusObj["standstill"] = (bool)status.standstill;

        // Get global status
        TMC2209::GlobalStatus gstatus = driver.getGlobalStatus();
        JsonObject globalObj = doc["globalStatus"].to<JsonObject>();
        globalObj["reset"] = (bool)gstatus.reset;
        globalObj["drvErr"] = (bool)gstatus.drv_err;
        globalObj["uvCp"] = (bool)gstatus.uv_cp;

        // Get dynamic values
        JsonObject dynamicObj = doc["dynamic"].to<JsonObject>();
        dynamicObj["stallGuardResult"] = driver.getStallGuardResult();
        dynamicObj["pwmScaleSum"] = driver.getPwmScaleSum();
        dynamicObj["pwmScaleAuto"] = driver.getPwmScaleAuto();
        dynamicObj["pwmOffsetAuto"] = driver.getPwmOffsetAuto();
        dynamicObj["pwmGradientAuto"] = driver.getPwmGradientAuto();
        dynamicObj["microstepCounter"] = driver.getMicrostepCounter();
    }

    String output;
    serializeJsonPretty(doc, output);
    return output;
}

String PolarControl::dumpThetaDriverSettings() {
    return dumpDriverToJson(m_tDriver, "theta");
}

String PolarControl::dumpRhoDriverSettings() {
    return dumpDriverToJson(m_rDriver, "rho");
}
