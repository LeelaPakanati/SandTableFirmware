#include "PolarControl.hpp"
#include "Logger.hpp"
#include <cmath>

// ============================================================================
// Constructor / Destructor
// ============================================================================

PolarControl::PolarControl() :
  m_tDriver(&Serial1, 0.12f, T_ADDR),
  m_rDriver(&Serial1, 0.12f, R_ADDR),
  m_rCDriver(&Serial1, 0.12f, RC_ADDR)
{
}

PolarControl::~PolarControl() {
  delete m_rStepper;
  delete m_tStepper;
}

// ============================================================================
// Helpers
// ============================================================================

double PolarControl::normalizeThetaDiff(double diff) {
  while (diff > PI) diff -= 2.0 * PI;
  while (diff < -PI) diff += 2.0 * PI;
  return diff;
}

double PolarControl::calcCartesianDist(PolarCord_t from, PolarCord_t to) {
  double x0 = from.rho * cos(from.theta);
  double y0 = from.rho * sin(from.theta);
  double x1 = to.rho * cos(to.theta);
  double y1 = to.rho * sin(to.theta);
  return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

// ============================================================================
// Lifecycle
// ============================================================================

void PolarControl::begin() {
  if (m_mutex == NULL) {
    m_mutex = xSemaphoreCreateMutex();
  }

  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  m_stepperEngine.init();

  m_rStepper = m_stepperEngine.stepperConnectToPin(R_STEP_PIN);
  if (m_rStepper == nullptr) {
    LOG("ERROR: Failed to connect R stepper\r\n");
    return;
  }
  m_rStepper->setDirectionPin(R_DIR_PIN);
  m_rStepper->setSpeedInHz((uint32_t)(STEPS_PER_MM * R_MAX_VELOCITY));
  m_rStepper->setAcceleration((uint32_t)(STEPS_PER_MM * R_MAX_ACCEL));
  m_rStepper->setCurrentPosition(0);

  m_tStepper = m_stepperEngine.stepperConnectToPin(T_STEP_PIN);
  if (m_tStepper == nullptr) {
    LOG("ERROR: Failed to connect T stepper\r\n");
    return;
  }
  m_tStepper->setDirectionPin(T_DIR_PIN);
  m_tStepper->setSpeedInHz((uint32_t)(STEPS_PER_RADIAN * T_MAX_VELOCITY));
  m_tStepper->setAcceleration((uint32_t)(STEPS_PER_RADIAN * T_MAX_ACCEL));
  m_tStepper->setCurrentPosition(0);

  // Initialize moveTimed queue processing for both steppers
  delay(10);
  auto rInit = m_rStepper->moveTimed(0, 0, NULL, true);
  auto tInit = m_tStepper->moveTimed(0, 0, NULL, true);
  LOG("Motor Setup Complete (R init=%d, T init=%d)\r\n", (int)rInit, (int)tInit);
}

void PolarControl::setupDrivers() {
  if (m_state != UNINITIALIZED) {
    LOG("Setup was already done\r\n");
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
  xSemaphoreTake(m_mutex, portMAX_DELAY);
  homeDriver(m_rDriver);
  LOG("R Homing Done\r\n");
  m_rStepper->setCurrentPosition(0);
  m_currPos = {0.0, 0.0};
  m_currVelR = 0.0;
  m_currVelT = 0.0;
  m_state = IDLE;
  xSemaphoreGive(m_mutex);
}

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

bool PolarControl::start(PosGen *posGen) {
  xSemaphoreTake(m_mutex, portMAX_DELAY);

  if (m_state != IDLE) {
    LOG("Not IDLE. Start Failed\r\n");
    xSemaphoreGive(m_mutex);
    return false;
  }

  // Normalize theta to prevent windup
  double oldTheta = m_currPos.theta;
  m_currPos.theta = fmod(oldTheta, 2.0 * PI);
  int32_t stepDiff = round((m_currPos.theta - oldTheta) * STEPS_PER_RADIAN);
  if (stepDiff != 0) {
    m_tStepper->setCurrentPosition(m_tStepper->getCurrentPosition() + stepDiff);
  }

  LOG("Starting new PosGen\r\n");

  if (m_posGen != nullptr) {
    delete m_posGen;
  }
  m_posGen = posGen;

  m_buffer.clear();
  m_currVelR = 0.0;
  m_currVelT = 0.0;
  fillBuffer();

  m_state = RUNNING;
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

  // Test if file is valid
  FilePosGen *testGen = new FilePosGen(filePath, maxRho);
  PolarCord_t firstPos = testGen->getNextPos();
  if (firstPos.isNan()) {
    LOG("ERROR: File is empty or invalid\r\n");
    delete testGen;
    xSemaphoreGive(m_mutex);
    return false;
  }
  delete testGen;

  // Create the actual generator
  if (m_posGen != nullptr) {
    delete m_posGen;
  }
  m_posGen = new FilePosGen(filePath, maxRho);

  m_buffer.clear();
  m_currVelR = 0.0;
  m_currVelT = 0.0;
  fillBuffer();

  m_state = RUNNING;
  LOG("File loaded and running\r\n");
  xSemaphoreGive(m_mutex);
  return true;
}

bool PolarControl::pause() {
  xSemaphoreTake(m_mutex, portMAX_DELAY);
  if (m_state == RUNNING) {
    forceStop();
    m_state = PAUSED;
    xSemaphoreGive(m_mutex);
    return true;
  }
  LOG("Not Running\r\n");
  xSemaphoreGive(m_mutex);
  return false;
}

bool PolarControl::resume() {
  xSemaphoreTake(m_mutex, portMAX_DELAY);
  if (m_state == PAUSED) {
    m_currVelR = 0.0;
    m_currVelT = 0.0;
    m_state = RUNNING;
    xSemaphoreGive(m_mutex);
    return true;
  }
  LOG("Not PAUSED\r\n");
  xSemaphoreGive(m_mutex);
  return false;
}

bool PolarControl::stop() {
  xSemaphoreTake(m_mutex, portMAX_DELAY);

  if (m_state == PAUSED || m_state == RUNNING) {
    forceStop();

    // Normalize theta
    double oldTheta = m_currPos.theta;
    m_currPos.theta = fmod(oldTheta, 2.0 * PI);
    int32_t stepDiff = round((m_currPos.theta - oldTheta) * STEPS_PER_RADIAN);
    if (stepDiff != 0) {
      m_tStepper->setCurrentPosition(m_tStepper->getCurrentPosition() + stepDiff);
    }

    if (m_posGen != nullptr) {
      delete m_posGen;
      m_posGen = nullptr;
    }
    m_buffer.clear();
    m_state = IDLE;
    xSemaphoreGive(m_mutex);
    return true;
  }
  LOG("Not PAUSED Or RUNNING\r\n");
  xSemaphoreGive(m_mutex);
  return false;
}

void PolarControl::setSpeed(uint8_t speed) {
  m_speed = speed;
}

PolarControl::State_t PolarControl::getState() {
  return m_state;
}

void PolarControl::forceStop() {
  m_rStepper->forceStop();
  m_tStepper->forceStop();
  m_currVelR = 0.0;
  m_currVelT = 0.0;
}

// ============================================================================
// Buffer Management
// ============================================================================

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

void PolarControl::fillBuffer() {
  if (m_posGen == nullptr) return;

  while (!m_buffer.isFull()) {
    PolarCord_t nextPoint = m_posGen->getNextPos();
    if (nextPoint.isNan()) break;

    // Filter near-duplicate points
    PolarCord_t lastPoint = m_buffer.count > 0 ? m_buffer.peek(m_buffer.count - 1) : m_currPos;
    double thetaDiff = normalizeThetaDiff(nextPoint.theta - lastPoint.theta);
    if (abs(lastPoint.rho - nextPoint.rho) < 0.01 && abs(thetaDiff) < 0.001) {
      continue;
    }

    m_buffer.push(nextPoint);
  }
}

// ============================================================================
// Main Processing Loop
// ============================================================================

bool PolarControl::processNextMove() {
  xSemaphoreTake(m_mutex, portMAX_DELAY);

  if (m_state != RUNNING) {
    xSemaphoreGive(m_mutex);
    return false;
  }

  fillBuffer();

  if (m_buffer.isEmpty()) {
    if (m_posGen != nullptr) {
      delete m_posGen;
      m_posGen = nullptr;
    }
    m_state = IDLE;
    LOG("Pattern complete\r\n");
    xSemaphoreGive(m_mutex);
    return false;
  }

  bool result = planNextMove();
  xSemaphoreGive(m_mutex);
  return result;
}

// ============================================================================
// Motion Planning
// ============================================================================

double PolarControl::calcLookAheadEndVel(int axis) {
  // Look at upcoming moves to determine target end velocity
  // Gradually decelerate as we approach a direction change for smooth motion

  if (m_buffer.count < 2) return 0.0;

  PolarCord_t curr = m_buffer.peek(0);
  double currDist = (axis == 0) ? (curr.rho - m_currPos.rho)
                                : normalizeThetaDiff(curr.theta - m_currPos.theta);

  if (abs(currDist) < 0.001) return 0.0;

  int currDir = currDist > 0 ? 1 : -1;
  double maxVel = (axis == 0) ? R_MAX_VELOCITY : T_MAX_VELOCITY;

  // Calculate cumulative distance until direction change
  double distToChange = abs(currDist);
  int dirChangeAt = -1;  // Index where direction changes (-1 = no change found)

  int lookAhead = std::min(8, m_buffer.count - 1);  // Look further ahead
  for (int i = 0; i < lookAhead; i++) {
    PolarCord_t from = m_buffer.peek(i);
    PolarCord_t to = m_buffer.peek(i + 1);

    double nextDist = (axis == 0) ? (to.rho - from.rho)
                                  : normalizeThetaDiff(to.theta - from.theta);

    if (abs(nextDist) < 0.001) continue;  // Skip tiny moves

    int nextDir = nextDist > 0 ? 1 : -1;
    if (nextDir != currDir) {
      dirChangeAt = i;
      break;
    }
    distToChange += abs(nextDist);
  }

  if (dirChangeAt < 0) {
    // No direction change in sight - maintain good velocity
    return maxVel * 0.8;
  }

  // Direction change coming - calculate smooth deceleration
  // Use kinematic equation: v² = 2*a*d  =>  v = sqrt(2*a*d)
  // This gives the velocity we can have now to decelerate to 0 over distance d
  double accel = (axis == 0) ? R_MAX_ACCEL : T_MAX_ACCEL;
  double safeVel = sqrt(2.0 * accel * distToChange);

  // Clamp to max velocity and apply smoothing factor
  safeVel = std::min(safeVel, maxVel * 0.8);

  // If very close to direction change, ensure we're slowing down significantly
  if (dirChangeAt == 0) {
    safeVel = std::min(safeVel, maxVel * 0.2);
  } else if (dirChangeAt == 1) {
    safeVel = std::min(safeVel, maxVel * 0.4);
  }

  return safeVel;
}

bool PolarControl::planNextMove() {
  PolarCord_t target = m_buffer.peek(0);
  if (target.isNan()) return false;

  // Normalize theta for shortest path
  target.theta = m_currPos.theta + normalizeThetaDiff(target.theta - m_currPos.theta);

  double rDist = target.rho - m_currPos.rho;
  double tDist = target.theta - m_currPos.theta;

  // Skip if we're already there
  if (abs(rDist) < 0.01 && abs(tDist) < 0.001) {
    m_buffer.pop();
    return true;
  }

  // Calculate target Cartesian velocity: speed 10 = 20mm/s
  double targetCartVel = m_speed * 2.0;

  // Calculate Cartesian distance
  double cartDist = calcCartesianDist(m_currPos, target);
  if (cartDist < 0.001) {
    // Extremely short move (< 1 micron) - skip it
    m_currPos = target;
    m_buffer.pop();
    return true;
  }

  // Enforce minimum Cartesian distance for timing purposes
  // This prevents very fast moves for tiny distances
  cartDist = std::max(cartDist, 0.5);  // Treat as at least 0.5mm for timing

  // Segment large moves - max 10mm Cartesian distance per segment
  const double MAX_SEGMENT_DIST = 10.0;
  bool shouldPop = true;
  PolarCord_t moveTarget = target;

  if (cartDist > MAX_SEGMENT_DIST) {
    double ratio = MAX_SEGMENT_DIST / cartDist;
    moveTarget.rho = m_currPos.rho + rDist * ratio;
    moveTarget.theta = m_currPos.theta + tDist * ratio;
    rDist *= ratio;
    tDist *= ratio;
    cartDist = MAX_SEGMENT_DIST;
    shouldPop = false;
  }

  // Calculate move time at target Cartesian velocity
  double moveTime = cartDist / targetCartVel;

  // Calculate required polar velocities for this move
  double reqVelR = abs(rDist) / moveTime;
  double reqVelT = abs(tDist) / moveTime;

  // Clamp to motor limits, scaling proportionally
  double scale = 1.0;
  if (reqVelR > R_MAX_VELOCITY) scale = std::min(scale, R_MAX_VELOCITY / reqVelR);
  if (reqVelT > T_MAX_VELOCITY) scale = std::min(scale, T_MAX_VELOCITY / reqVelT);

  // Also limit angular velocity based on tangential speed at current radius
  // At large radii, even small angular velocities create fast tangential motion
  double avgRho = (m_currPos.rho + moveTarget.rho) / 2.0;
  if (avgRho > 10.0 && reqVelT > 0.001) {  // Only if not near center
    double tangentialVel = reqVelT * avgRho;  // v = omega * r
    double maxTangentialVel = targetCartVel * 1.2;  // Allow 20% headroom
    if (tangentialVel > maxTangentialVel) {
      double tangentScale = maxTangentialVel / tangentialVel;
      scale = std::min(scale, tangentScale);
    }
  }

  double maxVelR = reqVelR * scale;
  double maxVelT = reqVelT * scale;

  // Determine end velocities using look-ahead
  double endVelR = shouldPop ? calcLookAheadEndVel(0) : maxVelR * 0.9;
  double endVelT = shouldPop ? calcLookAheadEndVel(1) : maxVelT * 0.9;

  // Scale end velocities to match current move's velocity ratio
  if (maxVelR > 0.01) endVelR = std::min(endVelR, maxVelR);
  if (maxVelT > 0.01) endVelT = std::min(endVelT, maxVelT);

  // Execute the move
  if (executeMove(rDist, tDist, m_currVelR, m_currVelT, endVelR, endVelT, maxVelR, maxVelT)) {
    m_currPos = moveTarget;
    m_currVelR = (rDist >= 0 ? 1 : -1) * endVelR;
    m_currVelT = (tDist >= 0 ? 1 : -1) * endVelT;
    if (shouldPop) m_buffer.pop();
    return true;
  }

  return false;
}

// ============================================================================
// Move Execution
// ============================================================================

bool PolarControl::executeMove(double rDist, double tDist,
                                double startVelR, double startVelT,
                                double endVelR, double endVelT,
                                double maxVelR, double maxVelT) {
  // Use moveTimed with our own motion planning
  // Break move into small time segments for smooth accel/decel

  // Convert to steps
  int32_t totalRSteps = round(rDist * STEPS_PER_MM);
  int32_t totalTSteps = round(tDist * STEPS_PER_RADIAN);

  if (totalRSteps == 0 && totalTSteps == 0) {
    return true;
  }

  // Calculate total move time at max velocity
  double rTime = (totalRSteps != 0) ? abs(rDist) / maxVelR : 0;
  double tTime = (totalTSteps != 0) ? abs(tDist) / maxVelT : 0;
  double moveTime = std::max(rTime, tTime);
  if (moveTime < 0.025) moveTime = 0.025;  // Minimum 25ms per move

  // Use small time segments (25ms) for smooth motion
  const double SEGMENT_TIME = 0.025;  // 25ms segments
  int numSegments = std::max(1, (int)(moveTime / SEGMENT_TIME));
  double actualSegmentTime = moveTime / numSegments;

  // Calculate steps per segment
  double rStepsPerSeg = (double)totalRSteps / numSegments;
  double tStepsPerSeg = (double)totalTSteps / numSegments;

  // Accumulate fractional steps to avoid drift
  double rAccum = 0.0;
  double tAccum = 0.0;

  for (int seg = 0; seg < numSegments; seg++) {
    // Calculate integer steps for this segment
    rAccum += rStepsPerSeg;
    tAccum += tStepsPerSeg;
    int16_t rSteps = (int16_t)round(rAccum);
    int16_t tSteps = (int16_t)round(tAccum);
    rAccum -= rSteps;
    tAccum -= tSteps;

    // Calculate segment time in microseconds
    // Ensure minimum 200us per step for FastAccelStepper
    uint32_t segTimeMicros = (uint32_t)(actualSegmentTime * 1000000.0);
    uint32_t minTimeR = (rSteps != 0) ? abs(rSteps) * 200 : 0;
    uint32_t minTimeT = (tSteps != 0) ? abs(tSteps) * 200 : 0;
    segTimeMicros = std::max(segTimeMicros, std::max(minTimeR, minTimeT));
    segTimeMicros = std::max(segTimeMicros, (uint32_t)1000);  // Min 1ms

    // Queue moves for both axes
    bool rOk = (rSteps == 0);
    bool tOk = (tSteps == 0);

    for (int retry = 0; retry < 50 && (!rOk || !tOk); retry++) {
      if (!rOk) {
        auto code = m_rStepper->moveTimed(rSteps, segTimeMicros, NULL, false);
        if (code == MOVE_TIMED_OK || code == MOVE_TIMED_EMPTY) {
          rOk = true;
        } else if (code == MoveTimedResultCode::QueueFull || code == MOVE_TIMED_BUSY) {
          m_rStepper->moveTimed(0, 0, NULL, true);
        }
      }
      if (!tOk) {
        auto code = m_tStepper->moveTimed(tSteps, segTimeMicros, NULL, false);
        if (code == MOVE_TIMED_OK || code == MOVE_TIMED_EMPTY) {
          tOk = true;
        } else if (code == MoveTimedResultCode::QueueFull || code == MOVE_TIMED_BUSY) {
          m_tStepper->moveTimed(0, 0, NULL, true);
        }
      }
      if (!rOk || !tOk) {
        vTaskDelay(1);
      }
    }

    // Start processing after each segment is queued
    m_rStepper->moveTimed(0, 0, NULL, true);
    m_tStepper->moveTimed(0, 0, NULL, true);

    if (!rOk || !tOk) {
      // Continue anyway - partial execution is better than stopping
    }
  }

  return true;
}
