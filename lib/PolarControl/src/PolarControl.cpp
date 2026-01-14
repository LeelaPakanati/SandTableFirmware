#include "PolarControl.hpp"
#include "Logger.hpp"
#include <cmath>

// ============================================================================ 
// Constructor / Destructor
// ============================================================================ 

PolarControl::PolarControl() :
  m_stepper(2),
  m_tDriver(&Serial1, 0.12f, T_ADDR),
  m_rDriver(&Serial1, 0.12f, R_ADDR),
  m_rCDriver(&Serial1, 0.12f, RC_ADDR)
{
}

PolarControl::~PolarControl() {
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
  
  // Initialize Steppers
  m_stepper.init_stepper(0, R_STEP_PIN); // Index 0 = R
  m_stepper.init_stepper(1, T_STEP_PIN); // Index 1 = T
  
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(T_DIR_PIN, OUTPUT);
  
  // Initialize positions
  m_rPos = 0;
  m_tPos = 0;
  m_rTargetPos = 0;
  m_tTargetPos = 0;

  delay(10);
  LOG("Motor Setup Complete\r\n");
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
  
  // Reset positions
  m_rPos = 0;
  m_rTargetPos = 0;
  // Theta is not homed physically, just reset coordinates
  m_tPos = 0; 
  m_tTargetPos = 0; 
  
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
    m_tPos += stepDiff;
    m_tTargetPos = m_tPos;
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
    // Don't force stop - let queued moves finish gracefully
    // Just stop generating new moves
    if (m_posGen != nullptr) {
      delete m_posGen;
      m_posGen = nullptr;
    }
    m_buffer.clear();
    m_currVelR = 0.0;
    m_currVelT = 0.0;

    // Transition to STOPPING state - processNextMove will transition to IDLE
    // when motors finish their queued moves
    m_state = STOPPING;
    LOG("Stopping - waiting for queued moves to complete\r\n");

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

PolarCord_t PolarControl::getActualPosition() const {
  // Calculate actual position from tracked target and remaining steps
  // current = target - (remaining * dir) 

  // Note: m_stepper is not const, but getActualPosition is const. 
  // MultiStepperLite::get_remaining_steps is not const. We cast away constness for this call as it shouldn't modify state.
  MultiStepperLite* stepper = const_cast<MultiStepperLite*>(&m_stepper);
  
  uint32_t rRemaining = stepper->get_remaining_steps(0);
  uint32_t tRemaining = stepper->get_remaining_steps(1);
  
  int32_t rCurrentSteps = m_rTargetPos - (int32_t)rRemaining * m_rDir;
  int32_t tCurrentSteps = m_tTargetPos - (int32_t)tRemaining * m_tDir;

  double rho = (double)rCurrentSteps / STEPS_PER_MM;
  double theta = (double)tCurrentSteps / STEPS_PER_RADIAN;
  return {theta, rho};
}

void PolarControl::forceStop() {
  m_stepper.pause(0);
  m_stepper.pause(1);
  
  // Update positions to where they stopped
  PolarCord_t pos = getActualPosition();
  m_rPos = round(pos.rho * STEPS_PER_MM);
  m_tPos = round(pos.theta * STEPS_PER_RADIAN);
  m_rTargetPos = m_rPos;
  m_tTargetPos = m_tPos;
  
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
  
  // Must call this frequently!
  m_stepper.do_tasks();

  // Handle STOPPING state - wait for motors to finish queued moves
  if (m_state == STOPPING) {
    bool rRunning = m_stepper.is_running(0);
    bool tRunning = m_stepper.is_running(1);

    if (!rRunning && !tRunning) {
      // Motors finished - update position and transition to IDLE
      PolarCord_t actualPos = getActualPosition();
      m_currPos = actualPos;

      // Normalize theta
      double oldTheta = m_currPos.theta;
      m_currPos.theta = fmod(oldTheta, 2.0 * PI);
      int32_t stepDiff = round((m_currPos.theta - oldTheta) * STEPS_PER_RADIAN);
      if (stepDiff != 0) {
        m_tPos += stepDiff;
        m_tTargetPos = m_tPos;
      }

      m_state = IDLE;
      LOG("Stop complete - now IDLE\r\n");
    }

    xSemaphoreGive(m_mutex);
    return false;
  }

  if (m_state != RUNNING) {
    xSemaphoreGive(m_mutex);
    return false;
  }

  // Wait for motors to finish current moves before planning new ones
  // This ensures timing is honored - we don't queue faster than execution
  bool rRunning = m_stepper.is_running(0);
  bool tRunning = m_stepper.is_running(1);
  if (rRunning || tRunning) {
    xSemaphoreGive(m_mutex);
    return true;  // Still working, check again later
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

  // Simple velocity model: use motor speed limits directly
  // Speed slider (1-10) scales the max velocities: speed 10 = full speed
  double speedFactor = m_speed / 10.0;
  double maxVelR = R_MAX_VELOCITY * speedFactor;
  double maxVelT = T_MAX_VELOCITY * speedFactor;

  // Calculate time needed for each axis at their max velocities
  double timeR = (abs(rDist) > 0.01) ? abs(rDist) / maxVelR : 0;
  double timeT = (abs(tDist) > 0.001) ? abs(tDist) / maxVelT : 0;

  // Use the longer time (slower axis dominates) to keep axes synchronized
  double moveTime = std::max(timeR, timeT);
  moveTime = std::max(moveTime, 0.025);  // Minimum 25ms per move

  // Calculate actual velocities used (may be slower than max for the faster axis)
  double actualVelR = (abs(rDist) > 0.01) ? abs(rDist) / moveTime : 0;
  double actualVelT = (abs(tDist) > 0.001) ? abs(tDist) / moveTime : 0;

  // Debug: log velocity calculations and positions with timestamp
  PolarCord_t actualPos = getActualPosition();
  LOG("[%lu] plan: rD=%.1f tD=%.2f t=%.2fs | queued(r=%.1f t=%.2f) actual(r=%.1f t=%.2f)\r\n",
      millis(), rDist, tDist, moveTime, m_currPos.rho, m_currPos.theta, actualPos.rho, actualPos.theta);

  // Determine end velocities using look-ahead
  double endVelR = calcLookAheadEndVel(0);
  double endVelT = calcLookAheadEndVel(1);

  // Cap end velocities to current move's velocities
  endVelR = std::min(endVelR, actualVelR);
  endVelT = std::min(endVelT, actualVelT);

  // Execute the move
  if (executeMove(rDist, tDist, m_currVelR, m_currVelT, endVelR, endVelT, actualVelR, actualVelT)) {
    m_currPos = target;
    m_currVelR = (rDist >= 0 ? 1 : -1) * endVelR;
    m_currVelT = (tDist >= 0 ? 1 : -1) * endVelT;
    m_buffer.pop();
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
  // Convert to steps
  int32_t totalRSteps = round(rDist * STEPS_PER_MM);
  int32_t totalTSteps = round(tDist * STEPS_PER_RADIAN);

  if (totalRSteps == 0 && totalTSteps == 0) {
    return true;
  }

  // Calculate total move time - slower axis dominates
  double rTime = (abs(rDist) > 0.01) ? abs(rDist) / maxVelR : 0;
  double tTime = (abs(tDist) > 0.001) ? abs(tDist) / maxVelT : 0;
  double moveTime = std::max(rTime, tTime);
  if (moveTime < 0.010) moveTime = 0.010;  // Minimum 10ms per move

  // Calculate the actual speed needed for each axis to finish in moveTime
  // Speed in Hz = steps / seconds
  uint32_t rSpeedHz = (totalRSteps != 0) ? (uint32_t)(abs(totalRSteps) / moveTime) : 1;
  uint32_t tSpeedHz = (totalTSteps != 0) ? (uint32_t)(abs(totalTSteps) / moveTime) : 1;

  // Ensure minimum speed (1 Hz) to avoid divide by zero
  rSpeedHz = std::max(rSpeedHz, (uint32_t)1);
  tSpeedHz = std::max(tSpeedHz, (uint32_t)1);

  // Debug: log the move parameters
  LOG("exec: rS=%d tS=%d t=%.2fs rHz=%lu tHz=%lu\r\n",
      totalRSteps, totalTSteps, moveTime, rSpeedHz, tSpeedHz);

  // Determine direction and set pins
  m_rDir = (totalRSteps >= 0) ? 1 : -1;
  m_tDir = (totalTSteps >= 0) ? 1 : -1;
  digitalWrite(R_DIR_PIN, (m_rDir == 1) ? HIGH : LOW);
  digitalWrite(T_DIR_PIN, (m_tDir == 1) ? HIGH : LOW);

  // Calculate interval in microseconds
  uint32_t rInterval = 1000000 / rSpeedHz;
  uint32_t tInterval = 1000000 / tSpeedHz;

  // Update target positions (committed position)
  // We commit the move now. getActualPosition will interpolate based on remaining steps.
  // Wait! Current m_rPos/m_tPos are where we are NOW (start of this move).
  // We need to advance them AFTER the move? 
  // No, the logic in getActualPosition uses m_rTargetPos.
  // m_rTargetPos should be the END of the move we are about to start.
  m_rTargetPos += totalRSteps; // Advance target
  m_tTargetPos += totalTSteps;

  // Start motors
  if (totalRSteps != 0) {
    m_stepper.start_finite(0, rInterval, abs(totalRSteps));
  }
  if (totalTSteps != 0) {
    m_stepper.start_finite(1, tInterval, abs(totalTSteps));
  }

  return true;
}
