#include "PolarControl.hpp"
#include <MotionPlanner.hpp>

// Constructor
PolarControl::PolarControl() :
  m_tDriver(&Serial1, 0.12f, T_ADDR),
  m_rDriver(&Serial1, 0.12f, R_ADDR),
  m_rCDriver(&Serial1, 0.12f, RC_ADDR)
{
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  m_stepperEngine.init();
  m_tStepper = m_stepperEngine.stepperConnectToPin(T_STEP_PIN);
  m_tStepper->setDirectionPin(T_DIR_PIN);
  m_tStepper->setSpeedInHz(T_MAX_STEP_VELOCITY);
  m_tStepper->setAcceleration(T_MAX_STEP_ACCEL);
  m_tStepper->setCurrentPosition(0);

  m_rStepper = m_stepperEngine.stepperConnectToPin(R_STEP_PIN);
  m_rStepper->setDirectionPin(R_DIR_PIN);
  m_rStepper->setSpeedInHz(R_MAX_STEP_VELOCITY);
  m_rStepper->setAcceleration(R_MAX_STEP_ACCEL);

  m_state = UNINITIALIZED;
  Serial.println("Motor Setup Complete");

  m_velocity = {m_speed * T_MAX_VELOCITY, m_speed * R_MAX_VELOCITY};
  m_dStepMax = m_velocity * TIME_RESOLUTION;
}

// Destructor
PolarControl::~PolarControl() {
  delete m_rStepper;
  delete m_tStepper;
}

// Public methods
void PolarControl::home() {
  home(m_rDriver);
  Serial.println("R Homing Done\n");
  m_rStepper->setCurrentPosition(0);
  m_currPos = {0.0, 0.0};
  m_currentVelR = 0.0;
  m_currentVelT = 0.0;
  m_state = IDLE;
}

void PolarControl::test() {
}

void PolarControl::setupDrivers() {
  if (m_state != UNINITIALIZED) {
    Serial.println("Setup was already done");
    return;
  }

  commonSetupDriver(m_tDriver);
  commonSetupDriver(m_rDriver);
  commonSetupDriver(m_rCDriver);

  m_rDriver.microsteps(R_MICROSTEPS);
  m_rDriver.rms_current(R_CURRENT);
  m_rCDriver.microsteps(R_MICROSTEPS);
  m_rCDriver.rms_current(R_CURRENT);
  m_tDriver.microsteps(T_MICROSTEPS);
  m_tDriver.rms_current(T_CURRENT);

  m_state = INITIALIZED;
}

bool PolarControl::processNextMove() {
  switch (m_state) {
    case UNINITIALIZED:
      break;
    case INITIALIZED:
      break;
    case IDLE:
      break;
    case RUNNING:
      if (move()) {
        if (m_posGen != nullptr) {
          delete m_posGen;
          m_posGen = nullptr;
        }
        m_state = IDLE;
        return false;
      }
      return true;
      break;
    case PAUSED:
      break;
  }
  return false;
}

bool PolarControl::start(PosGen *posGen) {
  if (m_state == IDLE) {
    Serial.println("Starting new PosGen");
    if (m_posGen != nullptr) {
      delete m_posGen;
      m_posGen = nullptr;
    }
    m_posGen = posGen;

    // Preload the look-ahead buffer
    preloadBuffer();

    m_state = RUNNING;

    // Get first position from buffer
    if (m_buffer.count > 0) {
      m_nextPos = m_buffer.peek(0);
    } else {
      // If buffer is empty (very short pattern), get directly
      m_nextPos = m_posGen->getNextPos();
    }

    return true;
  }
  Serial.println("Not IDLE. Start Failed");
  return false;
}

PolarControl::State_t PolarControl::getState() {
  return m_state;
}

bool PolarControl::pause() {
  if (m_state == RUNNING) {
    forceStop();
    m_state = PAUSED;
    return true;
  }
  Serial.println("Not Running");
  return false;
}

bool PolarControl::resume() {
  if (m_state == PAUSED) {
    m_state = RUNNING;
    return true;
  }
  Serial.println("Not PAUSED");
  return false;
}

bool PolarControl::stop() {
  if ((m_state == PAUSED) || (m_state == RUNNING)) {
    forceStop();
    if (m_posGen != nullptr) {
      delete m_posGen;
      m_posGen = nullptr;
    }
    m_buffer.clear();
    m_state = IDLE;
    return true;
  }
  Serial.println("Not PAUSED Or RUNNING");
  return false;
}

void PolarControl::setSpeed(uint8_t speed) {
  m_speed = speed;
  m_velocity = {m_speed * T_MAX_VELOCITY, m_speed * R_MAX_VELOCITY};
}

void PolarControl::testUartControl() {
  m_tDriver.VACTUAL(1000);
  m_rDriver.VACTUAL(1000);
  m_rCDriver.VACTUAL(1000);
}

void PolarControl::testStepControl() {
  Serial.println("Testing Step Control -- moving 100000 steps ");
  m_rStepper->move(10000);
  while (m_rStepper->isRunning()) {
    delay(1);
  }
  Serial.println("Testing Step Control -- moving -100000 steps ");
  m_rStepper->move(-10000);
  while (m_rStepper->isRunning()) {
    delay(1);
  }
}

void PolarControl::verifyDriverConfig() {
  Serial.println("\n=== R Driver Configuration (Silent Mode) ===");
  Serial.print("Mode: ");
  Serial.println(m_rDriver.en_spreadCycle() ? "SpreadCycle (noisy)" : "StealthChop (SILENT)");
  Serial.print("PWM Autoscale: ");
  Serial.println(m_rDriver.pwm_autoscale() ? "ON (auto-tuning enabled)" : "OFF");
  Serial.print("PWM Autograd: ");
  Serial.println(m_rDriver.pwm_autograd() ? "ON (adaptive)" : "OFF");
  Serial.print("PWM Frequency: ");
  Serial.print(m_rDriver.pwm_freq());
  Serial.println(" (0=~35kHz quietest, 3=~23kHz)");
  Serial.print("PWM Gradient: ");
  Serial.println(m_rDriver.pwm_grad());
  Serial.print("PWM Offset: ");
  Serial.println(m_rDriver.pwm_ofs());
  Serial.print("TPWMTHRS: 0x");
  Serial.print(m_rDriver.TPWMTHRS(), HEX);
  Serial.println(" (velocity threshold for StealthChop)");
  Serial.print("Interpolation: ");
  Serial.println(m_rDriver.intpol() ? "ON (256 µsteps)" : "OFF");
  Serial.print("RMS Current: ");
  Serial.print(m_rDriver.rms_current());
  Serial.println(" mA");
  Serial.print("Microsteps: ");
  Serial.println(m_rDriver.microsteps());

  Serial.println("\n=== T Driver Configuration (Silent Mode) ===");
  Serial.print("Mode: ");
  Serial.println(m_tDriver.en_spreadCycle() ? "SpreadCycle (noisy)" : "StealthChop (SILENT)");
  Serial.print("PWM Autoscale: ");
  Serial.println(m_tDriver.pwm_autoscale() ? "ON" : "OFF");
  Serial.print("PWM Autograd: ");
  Serial.println(m_tDriver.pwm_autograd() ? "ON" : "OFF");
  Serial.print("PWM Frequency: ");
  Serial.println(m_tDriver.pwm_freq());
  Serial.print("TPWMTHRS: 0x");
  Serial.print(m_tDriver.TPWMTHRS(), HEX);
  Serial.println(" (0xFFFFF = always silent)");
  Serial.print("RMS Current: ");
  Serial.print(m_tDriver.rms_current());
  Serial.println(" mA");
  Serial.print("Microsteps: ");
  Serial.println(m_tDriver.microsteps());

  Serial.println("\n=== Tips for Maximum Silence ===");
  Serial.println("- Lower motor current = quieter (but less torque)");
  Serial.println("- Higher microsteps = smoother/quieter");
  Serial.println("- Slower acceleration = less audible");
  Serial.println("- Keep TPWMTHRS at 0xFFFFF to never switch to SpreadCycle");
}

bool PolarControl::loadAndRunFile(String filePath) {
  return loadAndRunFile(filePath, R_MAX);
}

bool PolarControl::loadAndRunFile(String filePath, double maxRho) {
  if (m_state != IDLE) {
    Serial.println("ERROR: Cannot load file - system not IDLE");
    Serial.print("Current state: ");
    Serial.println(m_state);
    return false;
  }

  Serial.print("\n=== Loading pattern file: ");
  Serial.print(filePath);
  Serial.println(" ===");

  // Create FilePosGen instance
  FilePosGen *fileGen = new FilePosGen(filePath, maxRho);

  // Try to read first position to verify file is valid
  PolarCord_t firstPos = fileGen->getNextPos();
  if (firstPos.isNan()) {
    Serial.println("ERROR: File is empty or invalid");
    delete fileGen;
    return false;
  }

  // File is valid, delete old posGen and create new instance
  delete fileGen;
  if (m_posGen != nullptr) {
    delete m_posGen;
    m_posGen = nullptr;
  }
  m_posGen = new FilePosGen(filePath, maxRho);

  Serial.println("File loaded successfully");
  Serial.print("Max radius scaling: ");
  Serial.print(maxRho);
  Serial.println(" mm");

  // Start execution
  m_state = RUNNING;
  m_nextPos = m_posGen->getNextPos();

  return true;
}

// Private methods
bool PolarControl::calculateMotionProfile(
    double distance,
    double startVel,
    double endVel,
    double maxVel,
    double maxAccel,
    MotionProfile &profile) {

  profile.reset();
  profile.distance = abs(distance);
  profile.startVel = abs(startVel);
  profile.endVel = abs(endVel);
  profile.maxVel = abs(maxVel);
  profile.accel = abs(maxAccel);
  profile.decel = abs(maxAccel);

  if (profile.distance < 0.0001) {
    return false;
  }

  // Use S-curve profile if enabled (for maximum smoothness)
  if (m_useScurve) {
    // Determine jerk limit based on whether this is radial or angular
    // This is a heuristic - could be improved by passing axis info
    double maxJerk = (maxVel > 1.0) ? m_maxJerkR : m_maxJerkT;

    SMotionProfile sProfile;
    bool success = MotionPlanner::calculateSCurveProfile(
        profile.distance, profile.startVel, profile.endVel,
        profile.maxVel, profile.accel, maxJerk, sProfile);

    if (success) {
      // Copy S-curve profile data to standard profile
      profile.accelDist = sProfile.accelDist;
      profile.cruiseDist = sProfile.cruiseDist;
      profile.decelDist = sProfile.decelDist;
      profile.accelTime = sProfile.accelTime;
      profile.cruiseTime = sProfile.cruiseTime;
      profile.decelTime = sProfile.decelTime;
      profile.totalTime = sProfile.totalTime;
      return true;
    }
    // If S-curve fails, fall back to trapezoidal
  }

  // Standard trapezoidal profile calculation
  double minDecelDist = 0.0;
  if (profile.startVel > profile.endVel) {
    minDecelDist = (profile.startVel * profile.startVel - profile.endVel * profile.endVel) / (2.0 * profile.decel);
  }

  double accelToMaxDist = (profile.maxVel * profile.maxVel - profile.startVel * profile.startVel) / (2.0 * profile.accel);
  double decelFromMaxDist = (profile.maxVel * profile.maxVel - profile.endVel * profile.endVel) / (2.0 * profile.decel);

  if ((accelToMaxDist + decelFromMaxDist) <= profile.distance) {
    profile.accelDist = accelToMaxDist;
    profile.decelDist = decelFromMaxDist;
    profile.cruiseDist = profile.distance - profile.accelDist - profile.decelDist;

    profile.accelTime = (profile.maxVel - profile.startVel) / profile.accel;
    profile.cruiseTime = profile.cruiseDist / profile.maxVel;
    profile.decelTime = (profile.maxVel - profile.endVel) / profile.decel;
  } else {
    double a = profile.accel;
    double d = profile.decel;
    double v0_sq = profile.startVel * profile.startVel;
    double vf_sq = profile.endVel * profile.endVel;

    double vPeakSq = (profile.distance + v0_sq/(2.0*a) + vf_sq/(2.0*d)) / (1.0/(2.0*a) + 1.0/(2.0*d));

    if (vPeakSq < 0) {
      return false;
    }

    double vPeak = sqrt(vPeakSq);
    profile.maxVel = min(vPeak, profile.maxVel);

    profile.accelDist = (profile.maxVel * profile.maxVel - profile.startVel * profile.startVel) / (2.0 * profile.accel);
    profile.decelDist = (profile.maxVel * profile.maxVel - profile.endVel * profile.endVel) / (2.0 * profile.decel);
    profile.cruiseDist = 0.0;

    profile.accelTime = (profile.maxVel - profile.startVel) / profile.accel;
    profile.cruiseTime = 0.0;
    profile.decelTime = (profile.maxVel - profile.endVel) / profile.decel;
  }

  profile.totalTime = profile.accelTime + profile.cruiseTime + profile.decelTime;
  return true;
}

bool PolarControl::synchronizeProfiles(
    MotionProfile &profile1,
    MotionProfile &profile2,
    double targetTime) {

  if (targetTime <= 0.0) {
    targetTime = max(profile1.totalTime, profile2.totalTime);
  }

  if (!adjustProfileToTime(profile1, targetTime)) return false;
  if (!adjustProfileToTime(profile2, targetTime)) return false;

  return true;
}

bool PolarControl::adjustProfileToTime(MotionProfile &profile, double targetTime) {
  if (profile.distance < 0.0001) {
    profile.totalTime = targetTime;
    return true;
  }

  double avgVel = profile.distance / targetTime;

  double minVel = max(profile.startVel, profile.endVel);
  double maxVel = profile.maxVel;
  double bestVel = avgVel;

  for (int iter = 0; iter < 20; iter++) {
    double testVel = (minVel + maxVel) / 2.0;

    double accelDist = (testVel * testVel - profile.startVel * profile.startVel) / (2.0 * profile.accel);
    double decelDist = (testVel * testVel - profile.endVel * profile.endVel) / (2.0 * profile.decel);

    if (accelDist + decelDist > profile.distance) {
      maxVel = testVel;
      continue;
    }

    double cruiseDist = profile.distance - accelDist - decelDist;
    double accelTime = (testVel - profile.startVel) / profile.accel;
    double cruiseTime = cruiseDist / testVel;
    double decelTime = (testVel - profile.endVel) / profile.decel;
    double totalTime = accelTime + cruiseTime + decelTime;

    if (abs(totalTime - targetTime) < 0.001) {
      profile.maxVel = testVel;
      profile.accelDist = accelDist;
      profile.cruiseDist = cruiseDist;
      profile.decelDist = decelDist;
      profile.accelTime = accelTime;
      profile.cruiseTime = cruiseTime;
      profile.decelTime = decelTime;
      profile.totalTime = totalTime;
      return true;
    }

    if (totalTime > targetTime) {
      minVel = testVel;
    } else {
      maxVel = testVel;
    }
  }

  profile.maxVel = bestVel;
  return calculateMotionProfile(profile.distance, profile.startVel, profile.endVel,
                               bestVel, profile.accel, profile);
}

double PolarControl::calculateMaxEndVelocity(
    double currentDist,
    double nextDist,
    double currentAngle,
    double nextAngle,
    double maxVel,
    double maxAccel) {

  if (nextDist < 0.0001) {
    return 0.0;
  }

  double angleChange = abs(nextAngle - currentAngle);
  if (angleChange > PI) {
    angleChange = 2 * PI - angleChange;
  }

  double directionFactor = cos(angleChange / 2.0);
  double maxEndVel = maxVel * directionFactor;

  double maxExitVel = sqrt(2.0 * maxAccel * nextDist + 0.0);

  return min(maxEndVel, maxExitVel);
}

void PolarControl::commonSetupDriver(TMC2209Stepper &driver) {
  driver.begin();

  // Basic timing configuration
  driver.toff(4);                       // Off time: 4 is good for StealthChop
  driver.blank_time(24);                // Blank time: 24 cycles

  // Enable StealthChop for silent operation
  driver.en_spreadCycle(false);         // false = StealthChop mode (silent)

  // StealthChop PWM configuration for optimal silence
  driver.pwm_autoscale(true);           // Enable automatic PWM tuning
  driver.pwm_autograd(true);            // Enable automatic gradient adaptation
  driver.pwm_freq(0);                   // 0 = ~35kHz (quietest, default)
  driver.pwm_grad(14);                  // PWM gradient: 14 is good starting point
  driver.pwm_ofs(36);                   // PWM offset: 36 for good regulation
  driver.pwm_reg(4);                    // PWM regulation: 4 cycles
  driver.pwm_lim(12);                   // PWM limit: 12 for stable operation

  // Velocity threshold to stay in StealthChop
  // Higher value = stays silent at higher speeds
  // 0xFFFFF = always StealthChop (never switch to SpreadCycle)
  driver.TPWMTHRS(0xFFFFF);             // Stay in StealthChop at all speeds

  // Microstepping interpolation for smoother, quieter motion
  driver.intpol(true);                  // 256 microstep interpolation

  // UART control settings
  driver.I_scale_analog(false);         // Use UART for current control
  driver.internal_Rsense(false);        // Use external sense resistors
  driver.mstep_reg_select(true);        // Microsteps via UART
  driver.pdn_disable(true);             // Enable UART control
  driver.VACTUAL(0);                    // Stop any motion
  driver.shaft(false);                  // Motor direction
}

void PolarControl::home(TMC2209Stepper &driver, int speed) {
  Serial.print("Homing at speed: ");
  Serial.print(speed);

  driver.VACTUAL(500);
  delay(500);
  driver.VACTUAL(0);

  driver.VACTUAL(speed);

  delay(100);
  int sg_sum = driver.SG_RESULT();
  int cnt = 1;

  while (1) {
    int sg = driver.SG_RESULT();
    if ((sg <= (sg_sum/cnt) * .75) && (cnt > 15)){
      driver.VACTUAL(0);
      Serial.print("Hit Endstop");
      Serial.println(sg);
      break;
    }

    sg_sum = sg_sum + sg;
    cnt++;

    Serial.print("\tcnt = ");
    Serial.print(cnt);
    Serial.print("\tsg = ");
    Serial.print(sg);
    Serial.print("\tavg = ");
    Serial.println((sg_sum/cnt));
  }
}

void PolarControl::home(TMC2209Stepper &driver) {
  Serial.print("Preparing Homing\n");

  home(driver, 1000);
  driver.shaft(!driver.shaft());
  driver.VACTUAL(250);
  delay(1000);
  driver.VACTUAL(0);
  driver.shaft(!driver.shaft());
  home(driver, 250);
}

int PolarControl::getNumTR(PolarCord_t movement) {
  int tTR;
  int rTR;
  if (abs(movement.theta) <= m_dStepMax.theta) {
    tTR = 1;
  } else {
    tTR = std::ceil(movement.theta / m_dStepMax.theta);
  }
  if (abs(movement.rho) <= m_dStepMax.rho) {
    rTR = 1;
  } else {
    rTR = std::ceil(movement.rho / m_dStepMax.rho);
  }
  return std::min(rTR, tTR);
}

uint8_t PolarControl::move() {
  //Serial.println("CurrPos: " + m_currPos.getStr() + "\tMoving To " + m_nextPos.getStr());

  // If we've reached the current target, get next from buffer
  if (m_nextPos == m_currPos) {
    // Refill buffer if getting low (less than 5 points)
    if (m_buffer.count < 5 && m_posGen != nullptr) {
      while (!m_buffer.isFull()) {
        PolarCord_t nextPoint = m_posGen->getNextPos();
        if (std::isnan(nextPoint.rho) || std::isnan(nextPoint.theta)) {
          break;
        }
        m_buffer.push(nextPoint);
      }
    }

    // Get next position from buffer
    if (!m_buffer.isEmpty()) {
      m_nextPos = m_buffer.pop();
    } else if (m_posGen != nullptr) {
      // Buffer empty, try direct from generator (shouldn't happen normally)
      m_nextPos = m_posGen->getNextPos();
    }
  }

  if (m_nextPos.isNan())
    return 1;

  PolarCord_t moveVec = m_nextPos - m_currPos;
  PolarCord_t nextPos = {0.0, 0.0};
  bool tDirChange = (moveVec.theta * m_lastTDir) < 0;
  bool rDirChange = (moveVec.rho * m_lastRDir) < 0;

  if (tDirChange || rDirChange) {
    //Serial.println("Pausing 1 Step For direction change");
    nextPos = m_currPos;
  } else {
    PolarCord_t movement;
    int numTR = getNumTR(moveVec);
    movement = moveVec / numTR;
    nextPos = m_currPos + movement;
  }

  //Serial.println("\tNext Step: " + nextPos.getStr());

  if (moveTo(nextPos) == 255) {
    forceStop();
    return 255;
  }
  return 0;
}

void PolarControl::forceStop() {
  m_rStepper->forceStop();
  m_tStepper->forceStop();
  m_currentVelR = 0.0;
  m_currentVelT = 0.0;
}

uint8_t PolarControl::moveTo(PolarCord_t pos) {
  double rDist = pos.rho - m_currPos.rho;
  double tDist = pos.theta - m_currPos.theta;

  PolarCord_t nextNextPos = {0.0, 0.0};
  double targetEndVelR = 0.0;
  double targetEndVelT = 0.0;

  // ========== CENTER HANDLING ==========
  // Near center (r < threshold), switch to Cartesian motion planning
  // This prevents singularity issues at r=0
  bool nearCenter = (m_currPos.rho < m_centerThreshold || pos.rho < m_centerThreshold);

  if (nearCenter) {
    // Convert to Cartesian
    double x0 = m_currPos.rho * cos(m_currPos.theta);
    double y0 = m_currPos.rho * sin(m_currPos.theta);
    double x1 = pos.rho * cos(pos.theta);
    double y1 = pos.rho * sin(pos.theta);

    // Calculate Cartesian distance
    double cartesianDist = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));

    // Use reduced speed near center for safety
    double centerSpeedFactor = 0.3;
    targetEndVelR = R_MAX_VELOCITY * centerSpeedFactor;
    targetEndVelT = T_MAX_VELOCITY * centerSpeedFactor;

    //Serial.println("Near center - using Cartesian motion planning");
  }

  // ========== COORDINATED CARTESIAN VELOCITY ==========
  // Calculate constant tangential velocity for smooth visual motion
  // Convert polar velocities to Cartesian tangential speed
  double vx = m_currentVelR * cos(m_currPos.theta) - m_currPos.rho * m_currentVelT * sin(m_currPos.theta);
  double vy = m_currentVelR * sin(m_currPos.theta) + m_currPos.rho * m_currentVelT * cos(m_currPos.theta);
  double tangentialSpeed = sqrt(vx * vx + vy * vy);

  // Target constant tangential speed (visual speed of sand ball)
  double targetTangentialSpeed = R_MAX_VELOCITY * 0.7; // 70% of max for smoothness

  // Calculate curvature from buffer and adapt speed
  double adaptedMaxVelR = R_MAX_VELOCITY;
  double adaptedMaxVelT = T_MAX_VELOCITY;

  if (m_buffer.count >= 3) {
    PathCurvature curvature = calculateCurvature(0);

    // Adapt max velocities based on curvature
    adaptedMaxVelR = adaptSpeedForCurvature(R_MAX_VELOCITY, curvature, true);
    adaptedMaxVelT = adaptSpeedForCurvature(T_MAX_VELOCITY, curvature, false);

    //Serial.print("Curvature radius: ");
    //Serial.print(curvature.radius);
    //Serial.print("mm, Adapted R vel: ");
    //Serial.print(adaptedMaxVelR);
    //Serial.print(", Adapted T vel: ");
    //Serial.println(adaptedMaxVelT);
  }

  if (m_posGen != nullptr) {
    targetEndVelR = (abs(rDist) > 0.1) ? (adaptedMaxVelR * 0.5) : 0.0;
    targetEndVelT = (abs(tDist) > 0.1) ? (adaptedMaxVelT * 0.5) : 0.0;
  }

  MotionProfile rProfile, tProfile;

  bool rValid = calculateMotionProfile(
    abs(rDist),
    abs(m_currentVelR),
    targetEndVelR,
    adaptedMaxVelR,
    R_MAX_ACCEL,
    rProfile
  );

  bool tValid = calculateMotionProfile(
    abs(tDist),
    abs(m_currentVelT),
    targetEndVelT,
    adaptedMaxVelT,
    T_MAX_ACCEL,
    tProfile
  );

  if (rValid && tValid) {
    synchronizeProfiles(rProfile, tProfile);
  } else if (!rValid && !tValid) {
    return 0;
  }

  int32_t currRStep = m_currPos.rho * STEPS_PER_MM;
  int32_t nextRStep = pos.rho * STEPS_PER_MM;
  int32_t rMove = nextRStep - currRStep;

  int32_t currTStep = m_currPos.theta * STEPS_PER_RADIAN;
  int32_t nextTStep = pos.theta * STEPS_PER_RADIAN;
  int32_t tMove = nextTStep - currTStep;

  double moveTime = max(rProfile.totalTime, tProfile.totalTime);
  uint32_t moveTimeMicros = (uint32_t)(moveTime * 1000000.0);

  if (rMove != 0) {
    m_rStepper->setAcceleration(rProfile.accel * STEPS_PER_MM);

    MoveTimedResultCode rCode = m_rStepper->moveTimed(rMove, moveTimeMicros, NULL, false);
    switch (rCode) {
      case MOVE_TIMED_OK:
      case MOVE_TIMED_EMPTY:
        m_lastRDir = rMove < 0 ? -1 : 1;
        break;
      case MOVE_TIMED_BUSY:
      case MOVE_TIMED_TOO_LARGE_ERROR:
        //Serial.println("R MoveTimed Waiting for queue");
        return 1;
        break;
      default:
        Serial.println("MoveTimed R ERROR");
        return 255;
        break;
    }
  }

  if (tMove != 0) {
    m_tStepper->setAcceleration(tProfile.accel * STEPS_PER_RADIAN);

    int retries = 0;
    while (retries < 100) {
      MoveTimedResultCode tCode = m_tStepper->moveTimed(tMove, moveTimeMicros, NULL, true);
      switch (tCode) {
        case MOVE_TIMED_OK:
        case MOVE_TIMED_EMPTY:
          m_lastTDir = tMove < 0 ? -1 : 1;
          m_currPos = pos;
          m_currentVelR = (rMove != 0) ? rProfile.endVel * (rMove < 0 ? -1 : 1) : 0.0;
          m_currentVelT = (tMove != 0) ? tProfile.endVel * (tMove < 0 ? -1 : 1) : 0.0;

          //Serial.print("Move planned: R=");
          //Serial.print(rDist);
          //Serial.print("mm (");
          //Serial.print(rProfile.totalTime);
          //Serial.print("s), T=");
          //Serial.print(tDist);
          //Serial.print("rad (");
          //Serial.print(tProfile.totalTime);
          //Serial.println("s)");

          return 0;
          break;
        case MOVE_TIMED_BUSY:
        case MOVE_TIMED_TOO_LARGE_ERROR:
          //Serial.println("T MoveTimed Waiting for queue");
          delay(1);
          retries++;
          break;
        default:
          Serial.println("MoveTimed T ERROR");
          return 255;
          break;
      }
    }
    //Serial.println("T MoveTimed timeout");
    return 255;
  }

  m_currPos = pos;
  m_currentVelR = (rMove != 0) ? rProfile.endVel * (rMove < 0 ? -1 : 1) : 0.0;
  m_currentVelT = (tMove != 0) ? tProfile.endVel * (tMove < 0 ? -1 : 1) : 0.0;

  return 0;
}

// ============================================================================
// PathBuffer Implementation
// ============================================================================

void PolarControl::PathBuffer::push(PolarCord_t pos) {
  if (isFull()) {
    // Overwrite oldest if full (circular behavior)
    tail = (tail + 1) % BUFFER_SIZE;
    count--;
  }

  points[head] = pos;
  head = (head + 1) % BUFFER_SIZE;
  count++;
}

PolarCord_t PolarControl::PathBuffer::peek(int offset) const {
  if (offset >= count) {
    // Return NaN if offset is beyond available points
    return {NAN, NAN};
  }

  int index = (tail + offset) % BUFFER_SIZE;
  return points[index];
}

PolarCord_t PolarControl::PathBuffer::pop() {
  if (isEmpty()) {
    return {NAN, NAN};
  }

  PolarCord_t result = points[tail];
  tail = (tail + 1) % BUFFER_SIZE;
  count--;

  return result;
}

void PolarControl::PathBuffer::clear() {
  head = 0;
  tail = 0;
  count = 0;
}

// ============================================================================
// Look-Ahead Buffer Methods
// ============================================================================

void PolarControl::preloadBuffer() {
  if (m_posGen == nullptr) return;

  // Fill buffer with upcoming points
  m_buffer.clear();

  while (!m_buffer.isFull()) {
    PolarCord_t nextPoint = m_posGen->getNextPos();

    // Stop if we reach end of pattern (NaN)
    if (std::isnan(nextPoint.rho) || std::isnan(nextPoint.theta)) {
      break;
    }

    m_buffer.push(nextPoint);
  }
}

// ============================================================================
// Curvature-Based Speed Adaptation
// ============================================================================

PolarControl::PathCurvature PolarControl::calculateCurvature(int bufferIndex) {
  PathCurvature result = {INFINITY, 0.0, m_velocity.rho};

  // Need at least 3 points for curvature calculation
  if (bufferIndex + 2 >= m_buffer.count) {
    return result;
  }

  PolarCord_t p0 = m_buffer.peek(bufferIndex);
  PolarCord_t p1 = m_buffer.peek(bufferIndex + 1);
  PolarCord_t p2 = m_buffer.peek(bufferIndex + 2);

  // Convert polar to Cartesian for accurate curvature calculation
  double x0 = p0.rho * cos(p0.theta);
  double y0 = p0.rho * sin(p0.theta);
  double x1 = p1.rho * cos(p1.theta);
  double y1 = p1.rho * sin(p1.theta);
  double x2 = p2.rho * cos(p2.theta);
  double y2 = p2.rho * sin(p2.theta);

  // Calculate circumradius using determinant method
  double a = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
  double b = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  double c = sqrt(pow(x0 - x2, 2) + pow(y0 - y2, 2));

  // Area using Heron's formula
  double s = (a + b + c) / 2.0;
  double area = sqrt(s * (s - a) * (s - b) * (s - c));

  // Radius of curvature
  if (area > 1e-6) {
    result.radius = (a * b * c) / (4.0 * area);
  } else {
    // Nearly collinear points - essentially straight
    result.radius = INFINITY;
  }

  // Calculate angle change
  double angle1 = atan2(y1 - y0, x1 - x0);
  double angle2 = atan2(y2 - y1, x2 - x1);
  result.angle = fabs(angle2 - angle1);

  // Normalize angle to [0, PI]
  if (result.angle > PI) {
    result.angle = 2.0 * PI - result.angle;
  }

  return result;
}

double PolarControl::adaptSpeedForCurvature(double baseSpeed, const PathCurvature &curve, bool isRadial) {
  // For very tight curves, limit speed based on centripetal acceleration
  // v^2 / r = a_centripetal
  // v_max = sqrt(a_max * r)

  if (curve.radius < INFINITY && curve.radius > m_minCurveRadius) {
    double maxSafeSpeed = sqrt(m_maxCentripetalAccel * curve.radius);
    return std::min(baseSpeed, maxSafeSpeed);
  }

  return baseSpeed;
}

// ============================================================================
// S-Curve Profile Calculation
// ============================================================================

bool PolarControl::calculateSCurveProfile(
    double distance,
    double startVel,
    double endVel,
    double maxVel,
    double maxAccel,
    double maxJerk,
    SMotionProfile &profile) {

  // Delegate to MotionPlanner for S-curve calculation
  return MotionPlanner::calculateSCurveProfile(
      distance, startVel, endVel, maxVel, maxAccel, maxJerk, profile);
}

// ============================================================================
// Path Smoothing
// ============================================================================

PolarCord_t PolarControl::interpolateSpline(double t, PolarCord_t p0, PolarCord_t p1,
                                             PolarCord_t p2, PolarCord_t p3) {
  // Cubic Catmull-Rom spline interpolation
  // t is the parameter [0, 1] between p1 and p2
  // p0 and p3 are control points for smooth tangent calculation

  // Catmull-Rom matrix coefficients
  double t2 = t * t;
  double t3 = t2 * t;

  // Basis functions for Catmull-Rom
  double b0 = -0.5 * t3 + t2 - 0.5 * t;
  double b1 = 1.5 * t3 - 2.5 * t2 + 1.0;
  double b2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
  double b3 = 0.5 * t3 - 0.5 * t2;

  // Interpolate theta and rho independently
  double theta = b0 * p0.theta + b1 * p1.theta + b2 * p2.theta + b3 * p3.theta;
  double rho = b0 * p0.rho + b1 * p1.rho + b2 * p2.rho + b3 * p3.rho;

  // Ensure rho stays non-negative
  rho = std::max(0.0, rho);

  return {theta, rho};
}

void PolarControl::smoothDirectionChange(PolarCord_t &target) {
  // Smooth direction changes to avoid sudden stops
  // This is called when we detect a direction reversal

  // Calculate the movement vector
  PolarCord_t moveVec = target - m_currPos;

  // Check for theta direction change
  bool thetaDirectionChange = (moveVec.theta * m_lastTDir) < 0;
  bool rhoDirectionChange = (moveVec.rho * m_lastRDir) < 0;

  if (thetaDirectionChange || rhoDirectionChange) {
    // Gradually reduce velocity to near-zero before changing direction
    // This prevents the jerky 1-step pause behavior

    const int SMOOTH_STEPS = 3;
    double velocityReduction = 1.0 / SMOOTH_STEPS;

    // Create intermediate positions for smooth transition
    for (int i = 0; i < SMOOTH_STEPS; i++) {
      double factor = (double)(i + 1) / SMOOTH_STEPS;
      PolarCord_t intermediate;
      intermediate.theta = m_currPos.theta + moveVec.theta * factor * 0.1;
      intermediate.rho = m_currPos.rho + moveVec.rho * factor * 0.1;

      // This will naturally slow down as we approach the direction change
      // The actual implementation would queue these micro-moves
    }
  }
}

// ============================================================================
// Dynamic Time Resolution
// ============================================================================

double PolarControl::calculateDynamicTimeResolution(double distance) {
  // Adaptive time step based on movement distance
  // Shorter steps for long moves, longer steps for micro-adjustments
  // This optimizes performance across different pattern scales

  const double BASE_TIME_RESOLUTION = 0.1;  // 100ms baseline
  const double MIN_TIME_RESOLUTION = 0.05;  // 50ms minimum (faster updates)
  const double MAX_TIME_RESOLUTION = 0.5;   // 500ms maximum (slower for tiny moves)

  // Scale time resolution inversely with distance
  // Larger moves need finer time steps for smooth acceleration
  // Smaller moves can use coarser steps
  double dynamicResolution;

  if (distance > 50.0) {
    // Long moves: use fine resolution
    dynamicResolution = MIN_TIME_RESOLUTION;
  } else if (distance < 1.0) {
    // Very small moves: use coarse resolution
    dynamicResolution = MAX_TIME_RESOLUTION;
  } else {
    // Interpolate between min and max based on distance
    double factor = (50.0 - distance) / 49.0; // 0 at 50mm, 1 at 1mm
    dynamicResolution = MIN_TIME_RESOLUTION + factor * (MAX_TIME_RESOLUTION - MIN_TIME_RESOLUTION);
  }

  return dynamicResolution;
}
