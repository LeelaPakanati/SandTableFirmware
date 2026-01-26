#include "PolarControl.hpp"
#include "PolarUtils.hpp"
#include "MakeUnique.hpp"
#include "Logger.hpp"
#include "ErrorLog.hpp"
#include <cmath>
#include <cstdlib>
#include <SD.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// ============================================================================
// Constructor / Destructor
// ============================================================================

PolarControl::PolarControl() {
    // Initialize default driver settings
    m_tDriverSettings.runCurrent = 1200;   // Theta motor - higher current (mA)
    m_tDriverSettings.holdCurrent = 300;
    m_tDriverSettings.microsteps = 64;
    m_rDriverSettings.runCurrent = 500;    // Rho motor - lower current (mA)
    m_rDriverSettings.holdCurrent = 200;
    m_rDriverSettings.microsteps = 2;
}

PolarControl::~PolarControl() {
}

constexpr float PolarControl::R_MAX;
constexpr float PolarControl::R_SENSE;

static TMC2209::SerialAddress toSerialAddress(uint8_t address) {
    switch (address) {
        case 0:
            return TMC2209::SERIAL_ADDRESS_0;
        case 1:
            return TMC2209::SERIAL_ADDRESS_1;
        case 2:
            return TMC2209::SERIAL_ADDRESS_2;
        case 3:
            return TMC2209::SERIAL_ADDRESS_3;
        default:
            return TMC2209::SERIAL_ADDRESS_0;
    }
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
    m_tDriver.setup(Serial1, 115200, toSerialAddress(T_ADDR), RX_PIN, TX_PIN);
    m_rDriver.setup(Serial1, 115200, toSerialAddress(R_ADDR), RX_PIN, TX_PIN);
    m_rCDriver.setup(Serial1, 115200, toSerialAddress(RC_ADDR), RX_PIN, TX_PIN);

    delay(100);  // Allow drivers to initialize

    // Initialize LittleFS
    if (!LittleFS.begin(true)) {
        LOG("ERROR: LittleFS Mount Failed\r\n");
        ErrorLog::instance().log("ERROR", "FS", "LITTLEFS_MOUNT_FAILED",
                                 "LittleFS mount failed");
    } else {
        LOG("LittleFS Mounted\r\n");
    }

    // Create queues for async file reading
    m_coordQueue = xQueueCreate(256, sizeof(PolarCord_t));
    m_cmdQueue = xQueueCreate(5, sizeof(FileCommand));

    // Create file reader task on Core 0 (System Core)
    xTaskCreatePinnedToCore(
        fileReadTask,
        "FileReadTask",
        8192,
        this,
        1,
        &m_fileTaskHandle,
        0 // Core 0
    );

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
    float speedFactor = m_speed / 10.0f;
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
    // Only allow homing if system is relatively idle/safe to do so
    if (m_state != IDLE && m_state != INITIALIZED) {
        LOG("Cannot home: system not IDLE/INITIALIZED\r\n");
        return;
    }
    
    // Create async task for homing
    xTaskCreate(homingTask, "HomingTask", 4096, this, 1, NULL);
    LOG("Homing task started\r\n");
}

void PolarControl::homingTask(void* arg) {
    PolarControl* self = static_cast<PolarControl*>(arg);
    
    // Take mutex to protect driver access
    xSemaphoreTake(self->m_mutex, portMAX_DELAY);
    
    // Perform homing
    self->homeDriver(self->m_rDriver);
    LOG("R Homing Done\r\n");

    // Reset state
    self->m_state = IDLE;
    xSemaphoreGive(self->m_mutex);
    
    // Self-delete
    vTaskDelete(NULL);
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

    m_clearingSpeedActive = true;
    m_planner.setSpeedMultiplier(1.0f);

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

bool PolarControl::loadAndRunFile(String filePath, float maxRho) {
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    if (m_state != IDLE) {
        LOG("ERROR: Cannot load file - system not IDLE\r\n");
        ErrorLog::instance().log("ERROR", "FILE", "LOAD_NOT_IDLE",
                                 "Cannot load file - system not IDLE");
        xSemaphoreGive(m_mutex);
        return false;
    }

    LOG("Loading pattern file (Async): %s\r\n", filePath.c_str());

    // Clear any existing generator
    m_posGen.reset();

    // Send load command to file task
    FileCommand cmd;
    cmd.type = FileCommand::CMD_LOAD;
    strncpy(cmd.filename, filePath.c_str(), sizeof(cmd.filename) - 1);
    cmd.filename[sizeof(cmd.filename) - 1] = '\0';
    cmd.maxRho = maxRho;

    // Set flag explicitly BEFORE command to prevent race condition with feedPlanner
    m_fileLoading = true;

    // Reset planner stats
    m_planner.resetCompletedCount();

    // Send command
    if (xQueueSend(m_cmdQueue, &cmd, 100) != pdTRUE) {
        LOG("ERROR: Failed to send load command\r\n");
        ErrorLog::instance().log("ERROR", "FILE", "QUEUE_SEND_FAILED",
                                 "Failed to send load command");
        m_fileLoading = false; // Reset if send fails
        xSemaphoreGive(m_mutex);
        return false;
    }
    LOG("Load command queued\r\n");

    // Wait briefly for file task to start filling queue
    vTaskDelay(10);
    feedPlanner();

    // Start the motion planner
    m_planner.start();

    m_state = PREPARING;
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

    if (m_state == PAUSED || m_state == RUNNING || m_state == CLEARING || m_state == PREPARING) {
        m_posGen.reset();

        // Send stop command to file task
        FileCommand cmd;
        cmd.type = FileCommand::CMD_STOP;
        xQueueSend(m_cmdQueue, &cmd, 0);

        m_planner.stop();

        if (m_state == CLEARING && m_clearingSpeedActive) {
            m_clearingSpeedActive = false;
            updateSpeedSettings();
        }

        m_state = IDLE;
        LOG("Stopped\r\n");

        xSemaphoreGive(m_mutex);
        return true;
    }
    xSemaphoreGive(m_mutex);
    return false;
}

void PolarControl::setSpeed(uint8_t speed) {
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_speed = speed;
    if (!m_clearingSpeedActive) {
        updateSpeedSettings();
    }
    xSemaphoreGive(m_mutex);
}

void PolarControl::resetTheta() {
    m_planner.resetTheta();
}

PolarControl::State_t PolarControl::getState() {
    return m_state;
}

PolarCord_t PolarControl::getCurrentPosition() const {
    float theta, rho;
    // Note: casting away const for the planner call - it's thread-safe
    const_cast<MotionPlanner&>(m_planner).getCurrentPosition(theta, rho);
    return {theta, rho};
}

PolarCord_t PolarControl::getActualPosition() {
    float theta, rho;
    m_planner.getCurrentPosition(theta, rho);
    return {theta, rho};
}

PolarVelocity_t PolarControl::getActualVelocity() {
    float thetaVel, rhoVel;
    m_planner.getCurrentVelocity(thetaVel, rhoVel);
    return {thetaVel, rhoVel};
}

int PolarControl::getProgressPercent() const {
    // Cast away constness to use mutex (it's safe as we don't modify logical state)
    // Alternatively, make m_mutex mutable, but C++ style cast is easier here for existing code
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    int progress = -1;
    if (m_posGen) {
        progress = m_posGen->getProgressPercent();
    } else {
        uint32_t size = m_lastFileSize.load();
        if (size > 0) {
            uint32_t pos = m_lastFilePos.load();
            if (pos > size) pos = size;
            progress = static_cast<int>((pos * 100) / size);
        }
    }
    xSemaphoreGive(m_mutex);
    return progress;
}

void PolarControl::forceStop() {
    m_planner.stop();
}

uint32_t PolarControl::getFileTaskHighWater() const {
    if (!m_fileTaskHandle) {
        return 0;
    }
    return static_cast<uint32_t>(uxTaskGetStackHighWaterMark(m_fileTaskHandle));
}

// ============================================================================
// Feed segments to planner
// ============================================================================

void PolarControl::feedPlanner() {
    // Mode 1: Generator (Testing/Clear)
    if (m_posGen) {
        bool addedAny = false;
        while (m_planner.hasSpace()) {
            PolarCord_t next = m_posGen->getNextPos();

            if (next.isNan()) {
                m_planner.setEndOfPattern(true);
                break;
            }

            m_planner.setEndOfPattern(false);
            m_planner.addSegment(next.theta, next.rho);
            addedAny = true;
        }
        if (addedAny) {
            m_planner.recalculate();
            if (!m_planner.isRunning()) {
                m_planner.start();
            }
        }
        return;
    }

    // Mode 2: File Queue (Async)
    // Consume as much as possible from the queue - popping is fast!
    bool addedAny = false;
    while (m_planner.hasSpace()) {
        PolarCord_t next;
        if (xQueueReceive(m_coordQueue, &next, 0) == pdTRUE) {
            if (m_state == PREPARING) {
                m_state = RUNNING;
                LOG("feedPlanner: First coord received, state -> RUNNING\r\n");
            }
            m_planner.setEndOfPattern(false);
            m_planner.addSegment(next.theta, next.rho);
            addedAny = true;
        } else {
            // Queue empty
            // Check if file task is still loading
            // We use a safe peek or just assume if queue is empty and we aren't told it's done...
            // But we don't have a reliable "done" flag from the task easily visible here without shared state.
            // m_fileLoading is volatile bool, set by task.
            if (!m_fileLoading) {
                // File done and queue empty -> End of Pattern
                LOG("feedPlanner: Queue empty and fileLoading=false -> End of Pattern\r\n");
                m_planner.setEndOfPattern(true);
            }
            break;
        }
    }
    if (addedAny) {
        m_planner.recalculate();
        if (!m_planner.isRunning()) {
            m_planner.start();
        }
    }
}

// ============================================================================
// Main Processing Loop
// ============================================================================

bool PolarControl::processNextMove() {
    uint32_t waitStart = micros();
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_mutexWaitProfiler.addSample(micros() - waitStart);

    // Let planner process (handles timer internally)
    m_planner.process();

    if (m_state == RUNNING || m_state == CLEARING || m_state == PREPARING) {
        // Feed more segments to the planner
        feedPlanner();

        // Check if pattern is complete
        if (m_planner.isIdle()) {
            if (m_state == PREPARING && m_fileLoading) {
                xSemaphoreGive(m_mutex);
                return false;
            }
            if (m_state == CLEARING && m_clearingSpeedActive) {
                m_clearingSpeedActive = false;
                updateSpeedSettings();
            }
            m_posGen.reset();
            m_state = IDLE;
            LOG("Pattern Complete (idle state detected in processNextMove)\r\n");
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
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_motionSettings = settings;

    // Update the motion planner with new limits (doesn't reset positions)
    m_planner.setMotionLimits(
        m_motionSettings.rMaxVelocity,
        m_motionSettings.rMaxAccel,
        m_motionSettings.rMaxJerk,
        m_motionSettings.tMaxVelocity,
        m_motionSettings.tMaxAccel,
        m_motionSettings.tMaxJerk
    );

    LOG("Motion settings updated\r\n");
    xSemaphoreGive(m_mutex);
}

void PolarControl::setThetaDriverSettings(const DriverSettings& settings) {
    xSemaphoreTake(m_mutex, portMAX_DELAY);
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
    xSemaphoreGive(m_mutex);
}

void PolarControl::setRhoDriverSettings(const DriverSettings& settings) {
    xSemaphoreTake(m_mutex, portMAX_DELAY);
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
    xSemaphoreGive(m_mutex);
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
        ErrorLog::instance().log("ERROR", "TUNING", "WRITE_OPEN_FAILED",
                                 "Failed to open tuning file for writing");
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
        ErrorLog::instance().log("ERROR", "TUNING", "READ_OPEN_FAILED",
                                 "Failed to open tuning file for reading");
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        LOG("Failed to parse tuning file: %s\r\n", error.c_str());
        ErrorLog::instance().log("ERROR", "TUNING", "PARSE_FAILED",
                                 "Failed to parse tuning file", error.c_str());
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
    TestThetaContinuousGen(float fixedRho) : m_rho(fixedRho), m_phase(0) {}

    PolarCord_t getNextPos() override {
        // Phase 0: Rotate 5 full turns forward in one go
        // Phase 1: Rotate 5 full turns back in one go

        const float FULL_ROTATION = 2.0 * PI;
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
    float m_rho;
    int m_phase;
};

class TestThetaStressGen : public PosGen {
public:
    TestThetaStressGen(float fixedRho) : m_rho(fixedRho), m_phase(0), m_step(0) {}

    PolarCord_t getNextPos() override {
        const float DEGREES_TO_RADIANS = PI / 180.0f;

        // Varying move sizes in degrees
        static const float MOVE_SIZES[] = {
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
                float moveSize = MOVE_SIZES[sizeIdx] * DEGREES_TO_RADIANS;

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
            static const float QUICK_SIZES[] = {5.0f, 45.0f, 2.0f, 90.0f, 10.0f, 30.0f, 1.0f, 60.0f, 15.0f, 0.5f};
            static const int NUM_QUICK = sizeof(QUICK_SIZES) / sizeof(QUICK_SIZES[0]);

            if (m_step >= NUM_QUICK * 2) {
                return {std::nan(""), std::nan("")};
            }

            int sizeIdx = m_step / 2;
            bool goingOut = (m_step % 2 == 0);
            float moveSize = QUICK_SIZES[sizeIdx] * DEGREES_TO_RADIANS;

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
    float m_rho;
    float m_currentTheta = 0.0f;
    int m_phase;
    int m_step;
};

class TestRhoContinuousGen : public PosGen {
public:
    TestRhoContinuousGen(float maxRho) : m_maxRho(maxRho), m_phase(0) {}

    PolarCord_t getNextPos() override {
        const float CENTER = m_maxRho / 2.0f;
        const float MIN_RHO = 20.0f;

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
    float m_maxRho;
    int m_phase;
};

class TestRhoStressGen : public PosGen {
public:
    TestRhoStressGen(float maxRho) : m_maxRho(maxRho), m_phase(0), m_step(0) {
        m_currentRho = maxRho / 2;
    }

    PolarCord_t getNextPos() override {
        const float CENTER = m_maxRho / 2.0;

        // Varying move sizes in mm
        static const float MOVE_SIZES[] = {
            1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 30.0f, 50.0f, 75.0f, 100.0f,
            100.0f, 75.0f, 50.0f, 30.0f, 20.0f, 10.0f, 5.0f, 2.0f, 1.0f
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
                float moveSize = MOVE_SIZES[sizeIdx];

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
            static const float QUICK_SIZES[] = {10.0f, 50.0f, 5.0f, 100.0f, 20.0f, 75.0f, 2.0f, 30.0f, 1.0f, 60.0f};
            static const int NUM_QUICK = sizeof(QUICK_SIZES) / sizeof(QUICK_SIZES[0]);

            if (m_step >= NUM_QUICK * 2) {
                return {std::nan(""), std::nan("")};  // Done
            }

            int sizeIdx = m_step / 2;
            bool goingOut = (m_step % 2 == 0);
            float moveSize = QUICK_SIZES[sizeIdx];

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
    float m_maxRho;
    float m_currentRho;
    int m_phase;
    int m_step;
};

void PolarControl::testThetaContinuous() {
    if (m_state != IDLE) return;
    LOG("Starting theta continuous test...\r\n");

    float currentTheta, currentRho;
    m_planner.getCurrentPosition(currentTheta, currentRho);
    float testRho = (currentRho > 50.0f && currentRho < R_MAX - 50.0f) ? currentRho : R_MAX / 2.0f;

    resetTheta();
    start(std_patch::make_unique<TestThetaContinuousGen>(testRho));
}

void PolarControl::testThetaStress() {
    if (m_state != IDLE) return;
    LOG("Starting theta stress test...\r\n");

    float currentTheta, currentRho;
    m_planner.getCurrentPosition(currentTheta, currentRho);
    float testRho = (currentRho > 50.0f && currentRho < R_MAX - 50.0f) ? currentRho : R_MAX / 2.0f;

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
static void fillDriverJson(TMC2209& driver, const char* name, JsonDocument& doc) {
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
}

void PolarControl::writeThetaDriverSettings(Print& out) {
    JsonDocument doc;
    fillDriverJson(m_tDriver, "theta", doc);
    serializeJson(doc, out);
}

void PolarControl::writeRhoDriverSettings(Print& out) {
    JsonDocument doc;
    fillDriverJson(m_rDriver, "rho", doc);
    serializeJson(doc, out);
}

// Parse a coordinate line (theta, rho format)
static bool parseLine(const char* line, float maxRho, PolarCord_t& out) {
    const char* p = line;
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    if (*p == '\0' || *p == '#') {
        return false;
    }
    if (*p == '/' && *(p + 1) == '/') {
        return false;
    }

    char* end = nullptr;
    float theta = strtof(p, &end);
    if (end == p) {
        return false;
    }
    const char* q = end;
    while (*q == ' ' || *q == '\t') {
        ++q;
    }
    if (*q == ',') {
        ++q;
    }
    while (*q == ' ' || *q == '\t') {
        ++q;
    }
    if (*q == '\0') {
        return false;
    }

    float rho = strtof(q, &end);
    if (end == q) {
        return false;
    }

    out.theta = theta;
    out.rho = rho * maxRho;
    return true;
}

static bool readLineFromBuffer(File& file, char* buffer, size_t& bufLen, size_t& bufPos, bool& eof,
                               char* lineBuf, size_t lineCap, size_t& lineLen, bool& overflow) {
    lineLen = 0;
    overflow = false;
    while (true) {
        if (bufPos >= bufLen) {
            if (eof) {
                if (lineLen > 0) {
                    lineBuf[lineLen] = '\0';
                    return true;
                }
                return false;
            }
            int readBytes = file.read(reinterpret_cast<uint8_t*>(buffer), 4096);
            if (readBytes <= 0) {
                eof = true;
                if (lineLen > 0) {
                    lineBuf[lineLen] = '\0';
                    return true;
                }
                return false;
            }
            bufLen = static_cast<size_t>(readBytes);
            bufPos = 0;
        }

        char c = buffer[bufPos++];
        if (c == '\n') {
            lineBuf[lineLen] = '\0';
            return true;
        }
        if (c == '\r') {
            continue;
        }
        if (lineLen + 1 < lineCap) {
            lineBuf[lineLen++] = c;
        } else {
            overflow = true;
        }
    }
}

void PolarControl::fileReadTask(void* arg) {
    PolarControl* pc = static_cast<PolarControl*>(arg);
    FileCommand cmd;
    File directFile;
    float directMaxRho = 0.0f;
    bool directActive = false;
    char directBuffer[4096];
    size_t directBufLen = 0;
    size_t directBufPos = 0;
    bool directEof = false;
    char lineBuffer[128];
    size_t lineLen = 0;
    bool lineOverflow = false;
    bool overflowLogged = false;
    char currentFilename[sizeof(cmd.filename)] = {0};
    uint32_t yieldCounter = 0;
    
    // State for pending line handling
    PolarCord_t pendingPos;
    bool hasPendingPos = false;

    while (true) {
        // Check for commands (non-blocking if reading, blocking if idle)
        // If we have a pending pos, we MUST check for STOP commands but ignore LOAD
        // (though LOAD shouldn't happen while active usually)
        if (xQueueReceive(pc->m_cmdQueue, &cmd, (directActive || hasPendingPos) ? 0 : portMAX_DELAY) == pdTRUE) {
            LOG("FileTask: Received command %d\r\n", cmd.type);
            if (cmd.type == FileCommand::CMD_LOAD) {
                LOG("FileTask: Opening %s with maxRho=%.2f\r\n", cmd.filename, cmd.maxRho);
                strncpy(currentFilename, cmd.filename, sizeof(currentFilename) - 1);
                currentFilename[sizeof(currentFilename) - 1] = '\0';
                overflowLogged = false;
                directFile = SD.open(cmd.filename, FILE_READ);
                if (directFile) {
                    directActive = true;
                    directMaxRho = cmd.maxRho;
                    pc->m_lastFileLine.store(0);
                    pc->m_lastFilePos.store(0);
                    pc->m_lastFileSize.store(static_cast<uint32_t>(directFile.size()));
                    hasPendingPos = false; // Reset pending
                    directBufLen = 0;
                    directBufPos = 0;
                    directEof = false;
                    LOG("FileTask: Direct file open, active=true\r\n");
                } else {
                    LOG("FileTask: Failed to open file\r\n");
                    ErrorLog::instance().log("ERROR", "FILE", "OPEN_FAILED",
                                             "File task failed to open file", cmd.filename);
                    pc->m_fileLoading = false;
                    directActive = false;
                    pc->m_lastFilePos.store(0);
                    pc->m_lastFileSize.store(0);
                }
            } else if (cmd.type == FileCommand::CMD_STOP) {
                LOG("FileTask: Stopping\r\n");
                if (directFile) {
                    directFile.close();
                }
                directActive = false;
                hasPendingPos = false;
                pc->m_fileLoading = false;
                pc->m_lastFilePos.store(0);
                pc->m_lastFileSize.store(0);
                // Clear queue
                PolarCord_t dummy;
                while (xQueueReceive(pc->m_coordQueue, &dummy, 0) == pdTRUE);
            }
        }

        if (directActive) {
            // Only read next line if we don't have one pending
            if (!hasPendingPos) {
                bool hasLine = readLineFromBuffer(
                    directFile,
                    directBuffer,
                    directBufLen,
                    directBufPos,
                    directEof,
                    lineBuffer,
                    sizeof(lineBuffer),
                    lineLen,
                    lineOverflow);
                size_t unreadBytes = 0;
                if (directBufLen >= directBufPos) {
                    unreadBytes = directBufLen - directBufPos;
                }
                size_t filePos = directFile.position();
                size_t consumedPos = (filePos >= unreadBytes) ? (filePos - unreadBytes) : 0;

                if (!hasLine && directEof) {
                    directFile.close();
                    directActive = false;
                    pc->m_fileLoading = false;
                    pc->m_lastFilePos.store(pc->m_lastFileSize.load());
                    LOG("Direct file: EOF reached\r\n");
                } else {
                    pc->m_lastFilePos.store(static_cast<uint32_t>(consumedPos));
                    if (hasLine && lineOverflow && !overflowLogged) {
                        LOG("FileTask: Line overflow, skipping long line\r\n");
                        ErrorLog::instance().log("ERROR", "FILE", "LINE_OVERFLOW",
                                                 "Pattern line exceeded buffer", currentFilename);
                        overflowLogged = true;
                    }
                    if (!lineOverflow && parseLine(lineBuffer, directMaxRho, pendingPos)) {
                        hasPendingPos = true;
                    }
                    // If parse failed (comment/empty), loop continues to read next line
                }
            }

            // If we have a position, try to send it
            if (hasPendingPos) {
                if (xQueueSend(pc->m_coordQueue, &pendingPos, 0) == pdTRUE) {
                    // Success
                    hasPendingPos = false;
                    pc->m_lastFileLine.fetch_add(1);
                    // Throttle if the queue is near full to keep Core 0 responsive.
                    UBaseType_t spaces = uxQueueSpacesAvailable(pc->m_coordQueue);
                    if (spaces <= 64) {
                        vTaskDelay(2);
                    } else {
                        taskYIELD();
                    }
                } else {
                    // Queue full - yield and retry next loop
                    vTaskDelay(2);
                }
            } else {
                // Yield to allow other tasks on Core 0 (like WebServer) to run
                // Only yield if we didn't just push data (to keep throughput high when queue is open)
                vTaskDelay(1);
            }

            // Periodic cooperative yield to avoid starving the webserver.
            if ((++yieldCounter % 16) == 0) {
                vTaskDelay(1);
            }
        }
    }
}
