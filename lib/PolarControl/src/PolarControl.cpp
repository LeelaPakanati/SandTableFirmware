#include "PolarControl.hpp"
#include <MotionPlanner.hpp>

// ============================================================================
// Constructor / Destructor
// ============================================================================

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

  // Initialize queue processing
  m_tStepper->moveTimed(0, 0, NULL, true);
  m_rStepper->moveTimed(0, 0, NULL, true);

  m_state = UNINITIALIZED;
  Serial.println("Motor Setup Complete");
}

PolarControl::~PolarControl() {
  delete m_rStepper;
  delete m_tStepper;
}

// ============================================================================
// Public Interface
// ============================================================================

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

void PolarControl::home() {
  home(m_rDriver);
  Serial.println("R Homing Done");
  m_rStepper->setCurrentPosition(0);
  m_currPos = {0.0, 0.0};
  m_currentVelR = 0.0;
  m_currentVelT = 0.0;
  m_state = IDLE;
}

bool PolarControl::start(PosGen *posGen) {
  if (m_state != IDLE) {
    Serial.println("Not IDLE. Start Failed");
    return false;
  }

  Serial.println("Starting new PosGen");

  if (m_posGen != nullptr) {
    delete m_posGen;
  }
  m_posGen = posGen;

  m_buffer.clear();
  fillBuffer();

  m_state = RUNNING;
  return true;
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
  if (m_state == PAUSED || m_state == RUNNING) {
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
}

bool PolarControl::loadAndRunFile(String filePath) {
  return loadAndRunFile(filePath, R_MAX);
}

bool PolarControl::loadAndRunFile(String filePath, double maxRho) {
  if (m_state != IDLE) {
    Serial.println("ERROR: Cannot load file - system not IDLE");
    return false;
  }

  Serial.print("Loading pattern file: ");
  Serial.println(filePath);

  FilePosGen *testGen = new FilePosGen(filePath, maxRho);
  PolarCord_t firstPos = testGen->getNextPos();
  if (firstPos.isNan()) {
    Serial.println("ERROR: File is empty or invalid");
    delete testGen;
    return false;
  }
  delete testGen;

  m_posGen = new FilePosGen(filePath, maxRho);
  m_buffer.clear();
  fillBuffer();

  m_state = RUNNING;
  Serial.println("File loaded and running");
  return true;
}

// ============================================================================
// Main Processing Loop
// ============================================================================

bool PolarControl::processNextMove() {
  if (m_state != RUNNING) {
    return false;
  }

  fillBuffer();

  if (m_buffer.isEmpty()) {
    if (m_posGen != nullptr) {
      delete m_posGen;
      m_posGen = nullptr;
    }
    m_state = IDLE;
    Serial.println("Pattern complete");
    return false;
  }

  if (!planAndQueueNextMove()) {
    return true;  // Queue full, retry next iteration
  }

  return true;
}

// ============================================================================
// Buffer Management
// ============================================================================

void PolarControl::fillBuffer() {
  if (m_posGen == nullptr) return;

  while (!m_buffer.isFull()) {
    PolarCord_t nextPoint = m_posGen->getNextPos();
    if (nextPoint.isNan()) break;
    m_buffer.push(nextPoint);
  }
}

void PolarControl::PathBuffer::push(PolarCord_t pos) {
  if (isFull()) {
    tail = (tail + 1) % BUFFER_SIZE;
    count--;
  }
  points[head] = pos;
  head = (head + 1) % BUFFER_SIZE;
  count++;
}

PolarCord_t PolarControl::PathBuffer::peek(int offset) const {
  if (offset >= count) return {NAN, NAN};
  return points[(tail + offset) % BUFFER_SIZE];
}

PolarCord_t PolarControl::PathBuffer::pop() {
  if (isEmpty()) return {NAN, NAN};
  PolarCord_t result = points[tail];
  tail = (tail + 1) % BUFFER_SIZE;
  count--;
  return result;
}

void PolarControl::PathBuffer::clear() {
  head = tail = count = 0;
}

// ============================================================================
// Motion Planning
// ============================================================================

bool PolarControl::planAndQueueNextMove() {
  PolarCord_t target = m_buffer.peek(0);
  if (target.isNan()) return false;

  PolarCord_t moveVec = target - m_currPos;
  double rDist = moveVec.rho;
  double tDist = moveVec.theta;

  // Handle direction changes smoothly
  bool rDirChange = (rDist * m_lastRDir) < 0;
  bool tDirChange = (tDist * m_lastTDir) < 0;

  if (rDirChange || tDirChange) {
    if (!handleDirectionChange(target, rDirChange, tDirChange)) {
      return true;
    }
  }

  // Apply spline smoothing if we have enough points
  PolarCord_t smoothTarget = (m_buffer.count >= 4) ? calculateSmoothedTarget() : target;

  // Calculate distance and dynamic time resolution
  double cartesianDist = calculateCartesianDistance(m_currPos, smoothTarget);
  double timeRes = calculateDynamicTimeResolution(cartesianDist);
  double maxStepDist = R_MAX_VELOCITY * timeRes * m_speed / 5.0;

  // Subdivide large moves
  bool shouldPop = false;
  if (cartesianDist > maxStepDist) {
    double ratio = maxStepDist / cartesianDist;
    smoothTarget.rho = m_currPos.rho + (smoothTarget.rho - m_currPos.rho) * ratio;
    smoothTarget.theta = m_currPos.theta + (smoothTarget.theta - m_currPos.theta) * ratio;
  } else {
    shouldPop = true;
  }

  // Calculate curvature-adapted speeds
  double maxVelR = R_MAX_VELOCITY * m_speed / 5.0;
  double maxVelT = T_MAX_VELOCITY * m_speed / 5.0;

  if (m_buffer.count >= 3) {
    PathCurvature curve = calculateCurvature(0);
    maxVelR = adaptSpeedForCurvature(maxVelR, curve, true);
    maxVelT = adaptSpeedForCurvature(maxVelT, curve, false);
  }

  // Slow down near center
  if (m_currPos.rho < m_centerThreshold || smoothTarget.rho < m_centerThreshold) {
    maxVelR *= 0.3;
    maxVelT *= 0.3;
  }

  // Calculate look-ahead end velocities
  double endVelR = 0.0, endVelT = 0.0;

  if (m_buffer.count >= 2) {
    PolarCord_t nextTarget = m_buffer.peek(1);
    if (!nextTarget.isNan()) {
      double nextRDist = nextTarget.rho - smoothTarget.rho;
      double nextTDist = nextTarget.theta - smoothTarget.theta;
      bool sameRDir = (nextRDist * (smoothTarget.rho - m_currPos.rho)) >= 0;
      bool sameTDir = (nextTDist * (smoothTarget.theta - m_currPos.theta)) >= 0;
      endVelR = sameRDir ? maxVelR * 0.5 : 0.0;
      endVelT = sameTDir ? maxVelT * 0.5 : 0.0;
    }
  }

  if (queueCoordinatedMove(smoothTarget, maxVelR, maxVelT, endVelR, endVelT)) {
    if (shouldPop) m_buffer.pop();
    return true;
  }
  return false;
}

bool PolarControl::handleDirectionChange(PolarCord_t target, bool rChange, bool tChange) {
  PolarCord_t moveVec = target - m_currPos;
  PolarCord_t intermediate = m_currPos;

  if (rChange && abs(m_currentVelR) > 0.1) {
    double decelDist = (m_currentVelR * m_currentVelR) / (2.0 * R_MAX_ACCEL);
    intermediate.rho += m_lastRDir * std::min(decelDist, abs(moveVec.rho) * 0.1);
  }

  if (tChange && abs(m_currentVelT) > 0.1) {
    double decelDist = (m_currentVelT * m_currentVelT) / (2.0 * T_MAX_ACCEL);
    intermediate.theta += m_lastTDir * std::min(decelDist, abs(moveVec.theta) * 0.1);
  }

  return queueCoordinatedMove(intermediate,
                              R_MAX_VELOCITY * m_speed / 10.0,
                              T_MAX_VELOCITY * m_speed / 10.0,
                              0.0, 0.0);
}

PolarCord_t PolarControl::calculateSmoothedTarget() {
  PolarCord_t p0 = m_currPos;
  PolarCord_t p1 = m_currPos;
  PolarCord_t p2 = m_buffer.peek(0);
  PolarCord_t p3 = m_buffer.peek(1);

  if (p2.isNan() || p3.isNan()) return m_buffer.peek(0);

  return interpolateSpline(0.3, p0, p1, p2, p3);
}

double PolarControl::calculateCartesianDistance(PolarCord_t from, PolarCord_t to) {
  double x0 = from.rho * cos(from.theta);
  double y0 = from.rho * sin(from.theta);
  double x1 = to.rho * cos(to.theta);
  double y1 = to.rho * sin(to.theta);
  return sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
}

// ============================================================================
// Move Execution
// ============================================================================

bool PolarControl::queueCoordinatedMove(PolarCord_t target,
                                         double maxVelR, double maxVelT,
                                         double endVelR, double endVelT) {
  double rDist = target.rho - m_currPos.rho;
  double tDist = target.theta - m_currPos.theta;

  if (abs(rDist) < 0.001 && abs(tDist) < 0.001) return true;

  MotionProfile rProfile, tProfile;
  bool rValid = calculateMotionProfile(abs(rDist), abs(m_currentVelR), endVelR, maxVelR, R_MAX_ACCEL, rProfile);
  bool tValid = calculateMotionProfile(abs(tDist), abs(m_currentVelT), endVelT, maxVelT, T_MAX_ACCEL, tProfile);

  if (!rValid && !tValid) return true;

  synchronizeProfiles(rProfile, tProfile);

  int32_t rSteps = round(rDist * STEPS_PER_MM);
  int32_t tSteps = round(tDist * STEPS_PER_RADIAN);
  double moveTime = std::max(rProfile.totalTime, tProfile.totalTime);
  uint32_t moveTimeMicros = std::max(1000u, (uint32_t)(moveTime * 1000000.0));

  // Queue R axis
  if (rSteps != 0) {
    m_rStepper->setAcceleration(rProfile.accel * STEPS_PER_MM);
    for (int retries = 1000; retries > 0; retries--) {
      MoveTimedResultCode rCode = m_rStepper->moveTimed(rSteps, moveTimeMicros, NULL, false);
      if (rCode == MOVE_TIMED_OK || rCode == MOVE_TIMED_EMPTY) {
        m_lastRDir = rSteps < 0 ? -1 : 1;
        break;
      }
      if (rCode != MOVE_TIMED_BUSY && rCode != MOVE_TIMED_TOO_LARGE_ERROR && rCode != MoveTimedResultCode::QueueFull) {
        Serial.printf("R moveTimed error: %d\n", (int)rCode);
        return false;
      }
      if (retries == 1) {
        Serial.println("R moveTimed timeout");
        return false;
      }
      delay(1);
    }
  }

  // Queue T axis with commit
  if (tSteps != 0) {
    m_tStepper->setAcceleration(tProfile.accel * STEPS_PER_RADIAN);
    for (int retries = 1000; retries > 0; retries--) {
      MoveTimedResultCode tCode = m_tStepper->moveTimed(tSteps, moveTimeMicros, NULL, true);
      if (tCode == MOVE_TIMED_OK || tCode == MOVE_TIMED_EMPTY) {
        m_lastTDir = tSteps < 0 ? -1 : 1;
        break;
      }
      if (tCode != MOVE_TIMED_BUSY && tCode != MOVE_TIMED_TOO_LARGE_ERROR && tCode != MoveTimedResultCode::QueueFull) {
        Serial.printf("T moveTimed error: %d\n", (int)tCode);
        return false;
      }
      if (retries == 1) {
        Serial.println("T moveTimed timeout");
        return false;
      }
      delay(1);
    }
  } else if (rSteps != 0) {
    m_rStepper->moveTimed(0, 0, NULL, true);
  }

  m_currPos = target;
  m_currentVelR = rValid ? rProfile.endVel * (rDist < 0 ? -1 : 1) : 0.0;
  m_currentVelT = tValid ? tProfile.endVel * (tDist < 0 ? -1 : 1) : 0.0;

  return true;
}

void PolarControl::forceStop() {
  m_rStepper->forceStop();
  m_tStepper->forceStop();
  m_currentVelR = 0.0;
  m_currentVelT = 0.0;
}

// ============================================================================
// Motion Profile Calculation
// ============================================================================

bool PolarControl::calculateMotionProfile(double distance, double startVel, double endVel,
                                          double maxVel, double maxAccel, MotionProfile &profile) {
  profile.reset();
  profile.distance = abs(distance);
  profile.startVel = abs(startVel);
  profile.endVel = abs(endVel);
  profile.maxVel = abs(maxVel);
  profile.accel = abs(maxAccel);
  profile.decel = abs(maxAccel);

  if (profile.distance < 0.0001) return false;

  // Try S-curve first
  if (m_useScurve) {
    double maxJerk = (maxVel > 1.0) ? m_maxJerkR : m_maxJerkT;
    SMotionProfile sProfile;

    if (MotionPlanner::calculateSCurveProfile(profile.distance, profile.startVel, profile.endVel,
                                               profile.maxVel, profile.accel, maxJerk, sProfile)) {
      profile.accelDist = sProfile.accelDist;
      profile.cruiseDist = sProfile.cruiseDist;
      profile.decelDist = sProfile.decelDist;
      profile.accelTime = sProfile.accelTime;
      profile.cruiseTime = sProfile.cruiseTime;
      profile.decelTime = sProfile.decelTime;
      profile.totalTime = sProfile.totalTime;
      return true;
    }
  }

  // Fall back to trapezoidal
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
    double a = profile.accel, d = profile.decel;
    double v0_sq = profile.startVel * profile.startVel;
    double vf_sq = profile.endVel * profile.endVel;
    double vPeakSq = (profile.distance + v0_sq/(2.0*a) + vf_sq/(2.0*d)) / (1.0/(2.0*a) + 1.0/(2.0*d));

    if (vPeakSq < 0) return false;

    profile.maxVel = std::min(sqrt(vPeakSq), profile.maxVel);
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

bool PolarControl::synchronizeProfiles(MotionProfile &p1, MotionProfile &p2, double targetTime) {
  if (targetTime <= 0.0) targetTime = std::max(p1.totalTime, p2.totalTime);
  adjustProfileToTime(p1, targetTime);
  adjustProfileToTime(p2, targetTime);
  return true;
}

bool PolarControl::adjustProfileToTime(MotionProfile &profile, double targetTime) {
  if (profile.distance < 0.0001) {
    profile.totalTime = targetTime;
    return true;
  }

  double minVel = std::max(profile.startVel, profile.endVel);
  double maxVel = profile.maxVel;

  for (int i = 0; i < 20; i++) {
    double testVel = (minVel + maxVel) / 2.0;
    double accelDist = (testVel * testVel - profile.startVel * profile.startVel) / (2.0 * profile.accel);
    double decelDist = (testVel * testVel - profile.endVel * profile.endVel) / (2.0 * profile.decel);

    if (accelDist + decelDist > profile.distance) {
      maxVel = testVel;
      continue;
    }

    double cruiseDist = profile.distance - accelDist - decelDist;
    double totalTime = (testVel - profile.startVel) / profile.accel +
                       cruiseDist / testVel +
                       (testVel - profile.endVel) / profile.decel;

    if (abs(totalTime - targetTime) < 0.001) {
      profile.maxVel = testVel;
      profile.accelDist = accelDist;
      profile.cruiseDist = cruiseDist;
      profile.decelDist = decelDist;
      profile.accelTime = (testVel - profile.startVel) / profile.accel;
      profile.cruiseTime = cruiseDist / testVel;
      profile.decelTime = (testVel - profile.endVel) / profile.decel;
      profile.totalTime = totalTime;
      return true;
    }

    if (totalTime > targetTime) minVel = testVel;
    else maxVel = testVel;
  }

  return true;
}

// ============================================================================
// Speed Adaptation
// ============================================================================

PolarControl::PathCurvature PolarControl::calculateCurvature(int bufferIndex) {
  PathCurvature result = {INFINITY, 0.0, R_MAX_VELOCITY};

  if (bufferIndex + 2 >= m_buffer.count) return result;

  PolarCord_t p0 = m_buffer.peek(bufferIndex);
  PolarCord_t p1 = m_buffer.peek(bufferIndex + 1);
  PolarCord_t p2 = m_buffer.peek(bufferIndex + 2);

  // Convert to Cartesian
  double x0 = p0.rho * cos(p0.theta), y0 = p0.rho * sin(p0.theta);
  double x1 = p1.rho * cos(p1.theta), y1 = p1.rho * sin(p1.theta);
  double x2 = p2.rho * cos(p2.theta), y2 = p2.rho * sin(p2.theta);

  double a = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
  double b = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  double c = sqrt(pow(x0 - x2, 2) + pow(y0 - y2, 2));

  double s = (a + b + c) / 2.0;
  double area = sqrt(s * (s - a) * (s - b) * (s - c));

  if (area > 1e-6) result.radius = (a * b * c) / (4.0 * area);

  double angle1 = atan2(y1 - y0, x1 - x0);
  double angle2 = atan2(y2 - y1, x2 - x1);
  result.angle = fabs(angle2 - angle1);
  if (result.angle > PI) result.angle = 2.0 * PI - result.angle;

  return result;
}

double PolarControl::adaptSpeedForCurvature(double baseSpeed, const PathCurvature &curve, bool isRadial) {
  if (curve.radius < INFINITY && curve.radius > m_minCurveRadius) {
    return std::min(baseSpeed, sqrt(m_maxCentripetalAccel * curve.radius));
  }
  return baseSpeed;
}

double PolarControl::calculateDynamicTimeResolution(double distance) {
  const double MIN_RES = 0.05, MAX_RES = 0.3;
  if (distance > 50.0) return MIN_RES;
  if (distance < 1.0) return MAX_RES;
  return MIN_RES + (50.0 - distance) / 49.0 * (MAX_RES - MIN_RES);
}

// ============================================================================
// Path Smoothing
// ============================================================================

PolarCord_t PolarControl::interpolateSpline(double t, PolarCord_t p0, PolarCord_t p1,
                                             PolarCord_t p2, PolarCord_t p3) {
  double t2 = t * t, t3 = t2 * t;
  double b0 = -0.5 * t3 + t2 - 0.5 * t;
  double b1 = 1.5 * t3 - 2.5 * t2 + 1.0;
  double b2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
  double b3 = 0.5 * t3 - 0.5 * t2;

  double theta = b0 * p0.theta + b1 * p1.theta + b2 * p2.theta + b3 * p3.theta;
  double rho = std::max(0.0, b0 * p0.rho + b1 * p1.rho + b2 * p2.rho + b3 * p3.rho);

  return {theta, rho};
}

// ============================================================================
// Driver Setup and Homing
// ============================================================================

void PolarControl::commonSetupDriver(TMC2209Stepper &driver) {
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  driver.pwm_freq(0);
  driver.pwm_grad(14);
  driver.pwm_ofs(36);
  driver.pwm_reg(4);
  driver.pwm_lim(12);
  driver.TPWMTHRS(0xFFFFF);
  driver.intpol(true);
  driver.I_scale_analog(false);
  driver.internal_Rsense(false);
  driver.mstep_reg_select(true);
  driver.pdn_disable(true);
  driver.VACTUAL(0);
  driver.shaft(false);
}

void PolarControl::home(TMC2209Stepper &driver, int speed) {
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
      Serial.print("Hit Endstop, sg=");
      Serial.println(sg);
      break;
    }
    sg_sum += sg;
    cnt++;
    delay(10);
  }
}

void PolarControl::home(TMC2209Stepper &driver) {
  Serial.println("Preparing Homing");
  home(driver, 1000);
  driver.shaft(!driver.shaft());
  driver.VACTUAL(250);
  delay(1000);
  driver.VACTUAL(0);
  driver.shaft(!driver.shaft());
  home(driver, 250);
}
